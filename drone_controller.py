#DroneController node manages a UAV's mission execution by receiving planned fire response waypoints, uploading them to the autopilot, 
# and handling autonomous or pilot-controlled operation modes. It listens for GPS updates, task assignments, and completion messages, 
# and interacts with MAVROS services to arm the drone, set flight modes, and initiate takeoff when in autonomous mode. 
# The controller also logs mission progress and publishes visited waypoints for monitoring.


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
import json
import sys
import math
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, WaypointPush, WaypointClear
from mavros_msgs.msg import Waypoint
import csv
from datetime import datetime

class DroneController(Node):
    def __init__(self, drone_id, mode):
        super().__init__(f'{drone_id}_controller')
        self.drone_id = drone_id
        self.mode = mode  # 'autonomous' or 'pilot'
    
        # Drone state
        self.current_gps = None
        self.got_initial_gps = False
        self.assignment = None
        self.armed = False

        # Path state - fire planner waypoints
        self.fire_planner_path = []  
        self.has_pending_mission = False  # Track if we have a mission waiting to upload

        # Service clients - create once and reuse
        self.wp_push_client = None
        self.wp_clear_client = None
        self.set_mode_client = None
        self.arming_client = None
        self.takeoff_client = None

        # QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers
        self.done_pub = self.create_publisher(String, '/task_done', qos)
        self.visited_waypoints_pub = self.create_publisher(
            String, f'/visited_waypoints_{drone_id}', qos
        )

        # Subscribers
        self.create_subscription(NavSatFix,
                                 f'/{drone_id}/mavros/global_position/global',
                                 self.gps_callback,
                                 qos)
        self.create_subscription(String, '/assignments', self.assignment_callback, qos)
        self.create_subscription(String, '/task_done', self.task_done_callback, qos)
        self.create_subscription(String, f'/fire_planner_path_{drone_id}', self.planned_path_callback, qos)
        self.log_pub = self.create_publisher(String, '/fire_planner_log', qos)

        # Timer to check task completion and handle pending missions
        self.timer = self.create_timer(1.0, self.check_task_status)

        # Initialise service clients
        self.init_service_clients()

        mode_str = "AUTONOMOUS" if self.mode == 'autonomous' else "PILOT IN THE LOOP"
        self.get_logger().info(f" {drone_id} Multi-Fire Drone Controller started in {mode_str} mode")
        self.get_logger().info(f" Waiting for path from planner on /fire_planner_path_{drone_id}")
        if self.mode == 'autonomous':
            self.get_logger().info(f" Will upload waypoints and AUTO-ARM/TAKEOFF")
        else:
            self.get_logger().info(f" Will upload waypoints ONLY (pilot controls arm/takeoff/mode)")

    def publish_log(self, text):
        #Publish log message to UI and console, tagged with drone ID
        tagged_text = f"[{self.drone_id}] {text}"
        msg = String()
        msg.data = tagged_text
        self.log_pub.publish(msg)
        self.get_logger().info(tagged_text)



    def init_service_clients(self):
        #Initialise all service clients
        self.wp_push_client = self.create_client(WaypointPush, f'/{self.drone_id}/mavros/mission/push')
        self.wp_clear_client = self.create_client(WaypointClear, f'/{self.drone_id}/mavros/mission/clear')
        
        # Only create these clients in autonomous mode
        if self.mode == 'autonomous':
            self.set_mode_client = self.create_client(SetMode, f'/{self.drone_id}/mavros/set_mode')
            self.arming_client = self.create_client(CommandBool, f'/{self.drone_id}/mavros/cmd/arming')
            self.takeoff_client = self.create_client(CommandTOL, f'/{self.drone_id}/mavros/cmd/takeoff')

    def gps_callback(self, msg: NavSatFix):
        if msg.status.status >= 0:  
            self.current_gps = msg
            if not self.got_initial_gps:
                self.get_logger().info(f" Initial GPS fix: {msg.latitude:.8f}, {msg.longitude:.8f}")
                self.got_initial_gps = True

    def assignment_callback(self, msg):
        data = json.loads(msg.data)
        if data['drone_id'] != self.drone_id:
            return

        self.publish_log(f"[Controller] Assignment received: {data}")
        self.assignment = data
        self.task_simulation_start = None

        # If we already have a pending mission, upload it now
        if self.has_pending_mission and self.fire_planner_path:
            self.publish_log(f"[Controller] Assignment received, uploading pending mission with {len(self.fire_planner_path)} waypoints")
            self.upload_fire_planner_path_async()
            self.has_pending_mission = False

        # Only do autonomous actions in autonomous mode
        if self.mode == 'autonomous':
            self.set_auto_mode()
            self.arm_drone()
            self.set_takeoff()
        else:
            self.publish_log(f"[Controller] PILOT MODE: Waypoints uploaded. Pilot must manually arm/takeoff/set mode")

    def task_done_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if (data['drone_id'] == self.drone_id and
                self.assignment and
                data['task_id'] == self.assignment['task_id']):
                self.get_logger().info(f" Task {data['task_id']} complete")
                # DON'T clear assignment immediately - let the planner send new path first
        except Exception as e:
            self.publish_log(f"[Controller] Error in task_done_callback: {e}")

    def planned_path_callback(self, msg: String):
        #Receive fire planner path and upload it as a mission
        try:
            new_path = json.loads(msg.data)  # list of [lat, lon, alt]

            # Convert all values to float and default alt if None
            self.fire_planner_path = [
                (float(lat), float(lon), float(alt if alt is not None else 20.0))
                for lat, lon, alt in new_path
                if lat is not None and lon is not None
            ]

            if self.fire_planner_path:
                self.publish_log(f"[Controller] Received FIRE PLANNER path with {len(self.fire_planner_path)} GPS waypoints")
                
                # Always try to upload immediately, regardless of assignment status
                self.upload_fire_planner_path_async()
                
                try:
                    self.save_mission_waypoints()
                    self.save_mission_json()
                except Exception as e:
                    self.publish_log(f"[Controller] Save mission file failed: {e}")

        except Exception as e:
            self.publish_log(f"[Controller] Error parsing fire planner path JSON: {e}")

    def save_mission_waypoints(self, filename=None):
        #Save mission in Mission Planner .waypoints format (QGC WPL 110)
        if not self.fire_planner_path:
            self.get_logger().warn("⚠️ No path to save to Mission Planner file")
            return

        if filename is None:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"{self.drone_id}_{ts}.waypoints"

        try:
            with open(filename, 'w', newline='') as f:
                writer = csv.writer(f, delimiter='\t')
                # Required header
                writer.writerow(["QGC WPL 110"])

                for i, (lat, lon, alt) in enumerate(self.fire_planner_path):
                    current = 1 if i == 0 else 0
                    frame = 3  # MAV_FRAME_GLOBAL_RELATIVE_ALT
                    command = 22 if i==0 else 16   # MAV_CMD_NAV_WAYPOINT
                    params = [0.0, 0.0, 0.0, 0.0, float(lat), float(lon), float(alt)]
                    autocontinue = 1

                    row = [i, current, frame, command, *params, autocontinue]
                    writer.writerow(row)

            self.get_logger().info(f" Saved Mission Planner waypoints → {filename}")

        except Exception as e:
            self.publish_log(f"[Controller]❌ Failed to save mission file: {e}")
  

    def save_mission_json(self, filename=None):
        #Save mission waypoints in JSON format matching your example.

        if not self.fire_planner_path:
            self.get_logger().warn("⚠️ No path to save mission JSON")
            return

        if filename is None:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"{self.drone_id}_{ts}.json"

        mission_items = []
        for i, (lat, lon, alt) in enumerate(self.fire_planner_path):
            # For first item, command 22 (takeoff), else 16 (waypoint)
            command = 22 if i == 0 else 16
            
            # Params array follows your example:
            # [param1=altitude, param2=0, param3=0, param4=0 or null, lat, lon, alt]
            
            param4 = 0 if command == 22 else 0 
            
            params = [
                float(alt) if command == 22 else 0,
                0,
                0,
                param4,
                float(lat),
                float(lon),
                float(alt)
            ]

            item = {
                "AMSLAltAboveTerrain": alt,
                "Altitude": alt,
                "AltitudeMode": 1,
                "autoContinue": True,
                "command": command,
                "doJumpId": i + 1,
                "frame": 3,
                "params": params,
                "type": "SimpleItem"
            }
            mission_items.append(item)

        
        last_lat, last_lon, _ = self.fire_planner_path[-1]
        land_item = {
            "AMSLAltAboveTerrain": 0,
            "Altitude": 0,
            "AltitudeMode": 1,
            "autoContinue": True,
            "command": 21,  # MAV_CMD_LAND
            "doJumpId": len(mission_items) + 1,
            "frame": 3,
            "params": [0, 0, 0, 0, last_lat, last_lon, 0],
            "type": "SimpleItem"
        }
        mission_items.append(land_item)

        mission_json = {
            "fileType": "Plan",
            "geoFence": {
                "circles": [],
                "polygons": [],
                "version": 2
            },
            "groundStation": "QGroundControl",
            "mission": {
                "cruiseSpeed": 15,
                "firmwareType": 0,
                "hoverSpeed": 5,
                "items": mission_items,
                "plannedHomePosition": [
                    self.fire_planner_path[0][0],  # home lat
                    self.fire_planner_path[0][1],  # home lon
                    188  
                ],
                "vehicleType": 2,
                "version": 2
            },
            "rallyPoints": {
                "points": [],
                "version": 2
            },
            "version": 1
        }

        try:
            with open(filename, 'w') as f:
                json.dump(mission_json, f, indent=4)
            self.get_logger().info(f"Saved mission JSON → {filename}")
        except Exception as e:
            self.publish_log(f"[Controller]❌ Failed to save mission JSON: {e}")


    def upload_fire_planner_path_async(self):
        """Upload fire planner path asynchronously without blocking"""
        if not self.fire_planner_path:
            self.publish_log(f"[Controller]⚠️ No path to upload")
            return

        # Check if clients are ready
        if not self.wp_clear_client or not self.wp_push_client:
            self.publish_log(f"[Controller]❌ Service clients not initialized")
            self.has_pending_mission = True
            return

        # Wait for services with shorter timeout
        self.publish_log(f"[Controller] Checking service availability...")
        
        if not self.wp_clear_client.wait_for_service(timeout_sec=2.0):
            self.publish_log(f"[Controller]⚠️ WaypointClear service not ready, marking as pending")
            self.has_pending_mission = True
            return

        if not self.wp_push_client.wait_for_service(timeout_sec=2.0):
            self.publish_log(f"[Controller]⚠️ WaypointPush service not ready, marking as pending")
            self.has_pending_mission = True
            return

        # Clear old mission first
        self.publish_log(f"[Controller] Clearing old mission...")
        req_clear = WaypointClear.Request()
        future_clear = self.wp_clear_client.call_async(req_clear)
        future_clear.add_done_callback(self.on_clear_complete)

    def on_clear_complete(self, future):
        """Callback when mission clear is complete"""
        try:
            result = future.result()
            if result and result.success:
                self.publish_log(f"[Controller]✅ Mission cleared successfully")
                # Now upload the new mission
                self.upload_waypoints()
            else:
                self.publish_log(f"[Controller]⚠️ Mission clear failed, but continuing with upload...")
                # Try to upload anyway
                self.upload_waypoints()
        except Exception as e:
            self.publish_log(f"[Controller]❌ Error in clear callback: {e}")
            # Try to upload anyway
            self.upload_waypoints()

    def upload_waypoints(self):
        """Upload the waypoints to autopilot"""
        # Build waypoints
        waypoints = []
        for i, (lat, lon, alt) in enumerate(self.fire_planner_path):
            try:
                lat_f = float(lat)
                lon_f = float(lon)
                alt_f = float(alt if alt is not None else 20.0)
            except Exception as e:
                self.get_logger().error(f"Invalid waypoint {i}: {lat}, {lon}, {alt} ({e})")
                continue
                
            wp = Waypoint()
            wp.frame = 3  # GLOBAL_REL_ALT
            wp.command = 22 if i == 0 else 16  # TAKEOFF for first, WAYPOINT for the rest
            wp.is_current = (i == 0)
            wp.autocontinue = True
            wp.param1 = 0.0
            wp.param2 = 0.0
            wp.param3 = 0.0
            wp.param4 = 0.0
            wp.x_lat = lat_f
            wp.y_long = lon_f
            wp.z_alt = alt_f
            waypoints.append(wp)

        if not waypoints:
            self.publish_log(f"[Controller]❌ No valid waypoints to upload")
            return

        # Push mission
        try:
            self.publish_log(f"[Controller] Uploading {len(waypoints)} waypoints to autopilot...")
            req = WaypointPush.Request()
            req.start_index = 0
            req.waypoints = waypoints

            # Log first few waypoints for debugging
            for i, wp in enumerate(req.waypoints[:3]):
                self.get_logger().info(
                    f"  WP {i}: lat={wp.x_lat:.8f}, lon={wp.y_long:.8f}, alt={wp.z_alt:.2f}, "
                    f"cmd={wp.command}, current={wp.is_current}"
                )

            future = self.wp_push_client.call_async(req)
            future.add_done_callback(self.on_waypoints_uploaded)

        except Exception as e:
            self.publish_log(f"[Controller]❌ Mission push failed: {e}")

    def on_waypoints_uploaded(self, future):
        """Callback when waypoints are uploaded"""
        try:
            result = future.result()
            if result and getattr(result, "success", False):
                count = getattr(result, "wp_transfered", len(self.fire_planner_path))
                self.get_logger().info(f" Mission uploaded successfully! {count} waypoints transferred")

                # Publish log for UI
                self.publish_log(f"[Controller]  Mission uploaded successfully ({count} waypoints)")

                # If in pilot mode, just notify
                if self.mode == 'pilot':
                    self.get_logger().info(" PILOT MODE: Mission ready. Pilot can now arm and takeoff")
                    self.publish_log(f"[Controller]  Pilot mode: mission ready for manual start")

                # If in autonomous mode, immediately request AUTO mode
                elif self.mode == 'autonomous':
                    self.get_logger().info(" Autonomous mode detected — switching to AUTO.MISSION")
                    self.publish_log(f"[Controller] Requesting AUTO.MISSION mode...")
                    self.set_auto_mode()

            else:
                error_msg = getattr(result, 'error_msg', 'Unknown error') if result else 'No result'
                self.publish_log(f"[Controller] ❌ Mission upload failed: {error_msg}")
                self.get_logger().warn(f"Mission upload failed: {error_msg}")

        except Exception as e:
            self.publish_log(f"[Controller] ❌ Error in upload callback: {e}")
            self.get_logger().error(f"Error in upload callback: {e}")


    def check_task_status(self):
        """Simulate task completion and handle pending missions"""
        # Handle pending mission uploads
        if self.has_pending_mission and self.fire_planner_path:
            self.publish_log(f"[Controller]Retrying pending mission upload...")
            self.upload_fire_planner_path_async()
            self.has_pending_mission = False

        if not self.assignment or not self.current_gps:
            return
        
        # Only check armed status in autonomous mode
        if self.mode == 'autonomous' and not self.armed:
            return

        current_time = time.time()

        if self.task_simulation_start is not None:
            elapsed = current_time - self.task_simulation_start
            if elapsed >= self.task_simulation_duration:
                done_msg = {"task_id": self.assignment['task_id'], "drone_id": self.drone_id}
                self.done_pub.publish(String(data=json.dumps(done_msg)))
                self.get_logger().info(f" Task completed: {self.assignment['task_id']}")
                # Clear assignment after publishing completion
                self.assignment = None
                self.task_simulation_start = None
            else:
                remaining = self.task_simulation_duration - elapsed
                if int(remaining) != int(remaining + 1):  # Log every second
                    self.get_logger().info(f" Task progress: {remaining:.1f}s remaining")

    def publish_visited_fire_waypoint(self, waypoint):
        visited_data = {
            "drone_id": self.drone_id,
            "waypoint": waypoint
        }
        json_msg = json.dumps(visited_data)
        self.visited_waypoints_pub.publish(String(data=json_msg))
        self.get_logger().info(f" Published visited FIRE waypoint: ({waypoint[0]:.6f}, {waypoint[1]:.6f}, {waypoint[2]})")

    # MAVROS Services - simplified and non-blocking (only used in autonomous mode)
    def set_auto_mode(self):
        if self.mode != 'autonomous':
            return
            
        if not self.set_mode_client:
            self.get_logger().error("❌ Set mode client not initialized")
            return
            
        if not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(" Set mode service not available")
            return
            
        request = SetMode.Request()
        request.custom_mode = "AUTO"
        future = self.set_mode_client.call_async(request)
        self.publish_log(f"[Controller] Requested AUTO.MISSION mode")

    def arm_drone(self):
        if self.mode != 'autonomous':
            return
            
        if not self.arming_client:
            self.get_logger().error("❌ Arming client not initialized")
            return
            
        if not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.publish_log(f"[Controller] Arming service not available")
            return
            
        request = CommandBool.Request()
        request.value = True
        future = self.arming_client.call_async(request)
        self.armed = True
        self.publish_log(f"[Controller] Requested drone arming")

    def set_takeoff(self):
        if self.mode != 'autonomous':
            return
            
        if not self.takeoff_client:
            self.get_logger().error(" Takeoff client not initialized")
            return
            
        if not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(" Takeoff service not available")
            return
            
        request = CommandTOL.Request()
        request.altitude = 50.0
        future = self.takeoff_client.call_async(request)
        self.get_logger().info(" Requested takeoff")

def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 2:
        print("Usage: python drone_controller.py <drone_id> [mode]")
        print("  mode: 'autonomous' (default) or 'pilot'")
        sys.exit(1)
    
    drone_id = sys.argv[1]
    
    # Get mode from command line or prompt user
    if len(sys.argv) >= 3:
        mode = sys.argv[2].lower()
        if mode not in ['autonomous', 'pilot']:
            print(f"Invalid mode: {mode}")
            print("Mode must be 'autonomous' or 'pilot'")
            sys.exit(1)
    else:
        print("\n" + "="*50)
        print(" DRONE CONTROLLER MODE SELECTION")
        print("="*50)
        print("1. Autonomous Mode (default)")
        print("   - Waypoints uploaded automatically")
        print("   - Drone arms and takes off automatically")
        print("   - Sets AUTO/GUIDED mode automatically")
        print()
        print("2. Pilot in the Loop Mode")
        print("   - Waypoints uploaded to autopilot")
        print("   - Pilot manually arms drone")
        print("   - Pilot manually initiates takeoff")
        print("   - Pilot manually sets flight mode")
        print("="*50)
        
        while True:
            choice = input("\nSelect mode (1 for autonomous, 2 for pilot) [1]: ").strip()
            if choice == '' or choice == '1':
                mode = 'autonomous'
                break
            elif choice == '2':
                mode = 'pilot'
                break
            else:
                print("Invalid choice. Please enter 1 or 2.")
    
    print(f"\n Starting {drone_id} in {mode.upper()} mode...\n")
    
    node = DroneController(drone_id, mode)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f"\n Shutting down {drone_id} controller...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()