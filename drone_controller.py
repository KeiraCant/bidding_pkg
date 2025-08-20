import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
import json
import sys
import math
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from mavros_msgs.msg import GlobalPositionTarget

class DroneController(Node):
    def __init__(self, drone_id):
        super().__init__(f'{drone_id}_controller')
        self.drone_id = drone_id

        # Drone state
        self.current_gps = None
        self.got_initial_gps = False
        self.assignment = None
        self.armed = False

        # Path state - only track fire planner waypoints
        self.fire_planner_path = []  # Only waypoints from fire planner
        self.current_waypoint = 0

        # Task simulation
        self.task_simulation_start = None
        self.task_simulation_duration = 5.0

        # QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers
        self.setpoint_pub = self.create_publisher(
            GlobalPositionTarget,
            f'/{drone_id}/mavros/setpoint_raw/global',
            qos
        )
        self.done_pub = self.create_publisher(String, '/task_done', qos)
        
        # Publisher for visited fire planner waypoints only
        self.visited_waypoints_pub = self.create_publisher(
            String, 
            f'/visited_waypoints_{drone_id}', 
            qos
        )

        # Subscribers
        self.create_subscription(NavSatFix,
                                 f'/{drone_id}/mavros/global_position/global',
                                 self.gps_callback,
                                 qos)
        self.create_subscription(String, '/assignments', self.assignment_callback, qos)
        self.create_subscription(String, '/task_done', self.task_done_callback, qos)
        
        # Only subscribe to fire planner paths - these are the waypoints we track
        self.create_subscription(String, f'/fire_planner_path_{drone_id}', self.planned_path_callback, qos)

        # Timer to send waypoints
        self.timer = self.create_timer(0.2, self.send_waypoint)

        self.get_logger().info(f"✅ {drone_id} GPS Drone Controller started")
        self.get_logger().info(f"🔗 Waiting for path from planner on /fire_planner_path_{drone_id}")
        self.get_logger().info(f"📡 Will publish ONLY fire planner waypoints to /visited_waypoints_{drone_id}")

    def gps_callback(self, msg: NavSatFix):
        if msg.status.status >= 0:  # valid fix
            self.current_gps = msg
            if not self.got_initial_gps:
                self.get_logger().info(f"📡 Initial GPS fix: {msg.latitude:.8f}, {msg.longitude:.8f}")
                self.got_initial_gps = True

    def assignment_callback(self, msg):
        data = json.loads(msg.data)
        if data['drone_id'] != self.drone_id:
            return

        self.get_logger().info(f"📦 Assignment received: {data}")
        self.assignment = data
        self.current_waypoint = 0
        self.task_simulation_start = None

        self.set_guided_mode()
        self.arm_drone()
        self.set_takeoff()

    def task_done_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if (data['drone_id'] == self.drone_id and
                self.assignment and
                data['task_id'] == self.assignment['task_id']):
                self.get_logger().info(f"🔥 Task {data['task_id']} complete")
                self.assignment = None
                self.fire_planner_path = []
        except Exception as e:
            self.get_logger().error(f"Error in task_done_callback: {e}")

    def find_current_position_in_path(self, new_path):
        """Find where we are in the new path based on our current GPS position"""
        if not self.current_gps or not new_path:
            return 0
            
        current_lat = self.current_gps.latitude
        current_lon = self.current_gps.longitude
        
        min_distance = float('inf')
        closest_index = 0
        
        for i, waypoint in enumerate(new_path):
            lat, lon, alt = waypoint
            distance = math.sqrt((lat - current_lat)**2 + (lon - current_lon)**2)
            if distance < min_distance:
                min_distance = distance
                closest_index = i
                
        return closest_index

    def planned_path_callback(self, msg: String):
        """Receive fire planner path - these are the ONLY waypoints we track as visited"""
        try:
            new_path = json.loads(msg.data)  # list of [lat, lon, alt]
            if new_path:
                new_path_tuples = [tuple(wp) for wp in new_path]
                
                if not self.fire_planner_path:
                    # First path received
                    self.get_logger().info(f"📍 Received initial FIRE PLANNER path with {len(new_path)} GPS waypoints")
                    self.fire_planner_path = new_path_tuples
                    self.current_waypoint = 0
                else:
                    # Path update received - find where we are and continue from there
                    current_pos_index = self.find_current_position_in_path(new_path_tuples)
                    
                    # Only take waypoints from our current position onwards
                    remaining_path = new_path_tuples[current_pos_index:]
                    
                    self.get_logger().info(f"📍 Fire planner path update: {len(new_path)} total waypoints")
                    self.get_logger().info(f"📍 Continuing from waypoint {current_pos_index}, {len(remaining_path)} remaining")
                    
                    self.fire_planner_path = remaining_path
                    self.current_waypoint = 0
                
                if self.current_gps:
                    self.get_logger().info(
                        f"📡 Current GPS at path receipt: {self.current_gps.latitude:.8f}, {self.current_gps.longitude:.8f}"
                    )
        except Exception as e:
            self.get_logger().error(f"❌ Error parsing fire planner path JSON: {e}")

    def publish_visited_fire_waypoint(self, waypoint):
        """Publish a visited FIRE PLANNER waypoint for the planner to track"""
        visited_data = {
            "drone_id": self.drone_id,
            "waypoint": waypoint
        }
        json_msg = json.dumps(visited_data)
        
        self.get_logger().info(f"🔥 PUBLISHING FIRE WAYPOINT to /visited_waypoints_{self.drone_id}")
        self.get_logger().info(f"🔥 Message content: {json_msg}")
        
        self.visited_waypoints_pub.publish(String(data=json_msg))
        self.get_logger().info(f"📤 ✅ Published visited FIRE waypoint: ({waypoint[0]:.6f}, {waypoint[1]:.6f}, {waypoint[2]})")

    def send_waypoint(self):
        if not self.assignment or not self.current_gps or not self.armed:
            return

        current_time = time.time()

        # Handle task simulation
        if self.task_simulation_start is not None:
            if current_time - self.task_simulation_start >= self.task_simulation_duration:
                done_msg = {"task_id": self.assignment['task_id'], "drone_id": self.drone_id}
                self.done_pub.publish(String(data=json.dumps(done_msg)))
                self.get_logger().info(f"✅ Task completed")
                self.assignment = None
                self.task_simulation_start = None
            return

        # Follow the fire planner path
        if self.fire_planner_path:
            lat, lon, alt = self.fire_planner_path[0]  # always take the first waypoint

            gpt = GlobalPositionTarget()
            gpt.header.stamp = self.get_clock().now().to_msg()
            gpt.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_INT
            gpt.latitude = lat
            gpt.longitude = lon
            gpt.altitude = alt
            gpt.type_mask = 0x0FF8
            self.setpoint_pub.publish(gpt)

            # simple distance check in degrees (~lat/lon)
            dist = math.sqrt((lat - self.current_gps.latitude)**2 + (lon - self.current_gps.longitude)**2)
            if dist < 0.00001:  # ~1 meter
                visited_waypoint = (lat, lon, alt)
                self.get_logger().info(f"🎯 Reached FIRE PLANNER waypoint: ({lat:.6f}, {lon:.6f}, {alt})")
                
                # Publish the visited FIRE PLANNER waypoint - this is the key change!
                self.publish_visited_fire_waypoint(visited_waypoint)
                
                # remove the reached waypoint
                self.fire_planner_path.pop(0)

                # Log remaining waypoints
                if self.fire_planner_path:
                    self.get_logger().info(f"📍 Remaining fire planner waypoints: {len(self.fire_planner_path)}")
                else:
                    self.get_logger().info("🎯 Reached final fire planner goal, starting task...")
                    self.task_simulation_start = current_time

    # MAVROS Services
    def set_guided_mode(self):
        client = self.create_client(SetMode, f'/{self.drone_id}/mavros/set_mode')
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("❌ Set mode mode service unavailable")
            return
        request = SetMode.Request()
        request.custom_mode = "GUIDED"
        client.call_async(request)

    def arm_drone(self):
        client = self.create_client(CommandBool, f'/{self.drone_id}/mavros/cmd/arming')
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("❌ Arming service unavailable")
            return
        request = CommandBool.Request()
        request.value = True
        client.call_async(request)
        self.armed = True

    def set_takeoff(self):
        client = self.create_client(CommandTOL, f'/{self.drone_id}/mavros/cmd/takeoff')
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("❌ Takeoff service unavailable")
            return
        request = CommandTOL.Request()
        request.altitude = 50.0
        client.call_async(request)

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 2:
        print("Usage: python drone_controller.py <drone_id>")
        sys.exit(1)
    drone_id = sys.argv[1]
    node = DroneController(drone_id)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()