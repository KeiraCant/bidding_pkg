import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix
import json
import sys
import math
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from mavros_msgs.msg import GlobalPositionTarget
from pymavlink import mavutil

class DroneController(Node):
    def __init__(self, drone_id):
        super().__init__(f'{drone_id}_controller')
        self.drone_id = drone_id

        # Drone state
        self.current_gps = None  # NavSatFix
        self.got_initial_gps = False  # Track first GPS fix
        self.assignment = None
        self.armed = False

        # Path state
        self.current_path = []  # list of (lat, lon, alt)
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

        # Subscribers
        self.create_subscription(NavSatFix,
                                 f'/{drone_id}/mavros/global_position/global',
                                 self.gps_callback,
                                 qos)
        self.create_subscription(String, '/assignments', self.assignment_callback, qos)
        self.create_subscription(String, '/task_done', self.task_done_callback, qos)
        self.create_subscription(String, f'/fire_planner_path_{drone_id}', self.planned_path_callback, qos)

        # Timer to send waypoints
        self.timer = self.create_timer(0.2, self.send_waypoint)

        self.get_logger().info(f"✅ {drone_id} GPS Drone Controller started")
        self.get_logger().info(f"🔗 Waiting for path from planner on /fire_planner_path_{drone_id}")

    def gps_callback(self, msg: NavSatFix):
        if msg.status.status >= 0:  # valid fix
            self.current_gps = msg

            # Log GPS only once at first fix
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
                self.current_path = []
        except Exception as e:
            self.get_logger().error(f"Error in task_done_callback: {e}")

    def planned_path_callback(self, msg: String):
        try:
            new_path = json.loads(msg.data)  # list of [lat, lon, alt]
            if new_path:
                self.get_logger().info(f"📍 Received new path with {len(new_path)} GPS waypoints")
                self.current_path = [tuple(wp) for wp in new_path]
                self.current_waypoint = 0

                if self.current_gps:
                    self.get_logger().info(
                        f"📡 Current GPS at path receipt: {self.current_gps.latitude:.8f}, {self.current_gps.longitude:.8f}"
                    )
        except Exception as e:
            self.get_logger().error(f"❌ Error parsing path JSON: {e}")


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

        # Follow the path
        if self.current_waypoint < len(self.current_path):
            lat, lon, alt = self.current_path[self.current_waypoint]

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
                self.current_waypoint += 1
                if self.current_waypoint >= len(self.current_path):
                    self.get_logger().info("🎯 Reached final goal, starting task...")
                    self.task_simulation_start = current_time

    # MAVROS Services
    def set_guided_mode(self):
        client = self.create_client(SetMode, f'/{self.drone_id}/mavros/set_mode')
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("❌ Set mode service unavailable")
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
