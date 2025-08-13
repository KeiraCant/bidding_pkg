import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import json
import sys
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from rclpy.clock import Clock
import math
import time


class DroneController(Node):
    def __init__(self, drone_id):
        super().__init__(f'{drone_id}_controller')
        self.drone_id = drone_id
        self.current_position = None
        self.assignment = None
        self.drone_poses = {}  # dict {drone_id: PoseStamped}
        self.armed = False

        # Enhanced collision avoidance state
        self.collision_wait_start = None
        self.max_wait_time = 10.0  # Maximum time to wait for collision resolution
        self.retry_interval = 1.0  # How often to retry when waiting
        self.last_retry_time = 0

        # Task completion state
        self.task_simulation_duration = 5.0
        self.task_simulation_start = None

        # Path following
        self.current_path = []
        self.current_waypoint = 0

        # QoS Profile - FIXED: Use consistent QoS settings
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers
        self.setpoint_pub = self.create_publisher(PoseStamped, f'/{drone_id}/mavros/setpoint_position/local', qos)
        self.pose_broadcast_pub = self.create_publisher(PoseStamped, f'/{drone_id}/pose_broadcast', qos)
        #self.path_pub = self.create_publisher(Path, f'/planned_path_{drone_id}', qos)  # Optional, can be kept or removed
        self.done_pub = self.create_publisher(String, '/task_done', qos)

        # Subscribers - FIXED: Use the correct pose topic
        self.create_subscription(PoseStamped, f'/{drone_id}/mavros/local_position/pose', self.pose_callback, qos)
        self.create_subscription(String, '/assignments', self.assignment_callback, qos)
        self.create_subscription(String, '/task_done', self.task_done_callback, qos)

        # Subscribe to planned path published by planner node - FIXED: Use same QoS
        # Subscribe to planned path published by planner node
        self.create_subscription(Path, f'/fire_planner_path_{drone_id}', self.planned_path_callback, qos)

        # Subscribe to other drones' pose broadcasts
        other_drones = [f"drone_{i}" for i in range(1, 4) if f"drone_{i}" != drone_id]
        for other_id in other_drones:
            topic = f'/{other_id}/pose_broadcast'
            self.create_subscription(PoseStamped, topic, self._pose_update_callback_factory(other_id), qos)

        # Timer to send waypoints
        self.timer = self.create_timer(0.2, self.send_waypoint)

        # NEW: Add a startup timer to help with initialization
        self.startup_timer = self.create_timer(1.0, self.startup_check)
        self.startup_count = 0

        self.get_logger().info(f"✅ {drone_id} controller node started")
        self.get_logger().info(f"🔗 Waiting for path from planner on /planned_path_{drone_id}")

    def startup_check(self):
        """Help debug startup issues."""
        self.startup_count += 1
        if self.startup_count <= 5:
            self.get_logger().info(f"Startup check {self.startup_count}: pos={self.current_position is not None}, "
                                 f"assignment={self.assignment is not None}, path={len(self.current_path)}")
        elif self.startup_count == 6:
            self.startup_timer.cancel()

    def pose_callback(self, msg):
        # Broadcast own pose with timestamp
        stamped_msg = PoseStamped()
        stamped_msg.header.stamp = Clock().now().to_msg()
        stamped_msg.header.frame_id = "map"
        stamped_msg.pose = msg.pose
        self.current_position = stamped_msg.pose.position
        self.pose_broadcast_pub.publish(stamped_msg)

    def _pose_update_callback_factory(self, drone_id):
        def callback(msg):
            self.drone_poses[drone_id] = msg
        return callback

    def assignment_callback(self, msg):
        data = json.loads(msg.data)
        if data['drone_id'] != self.drone_id:
            return

        self.get_logger().info(f"📦 Assignment: {data}")
        self.assignment = data

        # Reset everything
        self.collision_wait_start = None
        self.task_simulation_start = None
        # DON'T reset current_path here - let planner handle it
        # self.current_path = []
        self.current_waypoint = 0

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

    def planned_path_callback(self, msg: Path):
        # Receive planned path and update current path
        new_path = []
        for pose_stamped in msg.poses:
            pos = pose_stamped.pose.position
            new_path.append((pos.x, pos.y, pos.z))

        if new_path:
            self.get_logger().info(f"📍 Received new path with {len(new_path)} waypoints from planner")
            self.current_path = new_path
            self.current_waypoint = 0
            self.collision_wait_start = None
            
            # Log first few waypoints for verification
            self.get_logger().info("Path preview:")
            for i in range(min(5, len(new_path))):
                wp = new_path[i]
                self.get_logger().info(f"  WP {i}: ({wp[0]:.1f}, {wp[1]:.1f}, {wp[2]:.1f})")
            if len(new_path) > 5:
                self.get_logger().info(f"  ... and {len(new_path)-5} more waypoints")

    def send_waypoint(self):
        if not self.assignment or not self.current_position or not self.armed:
            return

        current_time = time.time()

        # Handle task simulation (waiting while performing task)
        if self.task_simulation_start is not None:
            if current_time - self.task_simulation_start >= self.task_simulation_duration:
                done_msg = {"task_id": self.assignment['task_id'], "drone_id": self.drone_id}
                self.done_pub.publish(String(data=json.dumps(done_msg)))
                self.get_logger().info(f"✅ Task completed")
                self.assignment = None
                self.task_simulation_start = None
            return

        # Follow the path received via subscription
        if self.current_waypoint < len(self.current_path):
            waypoint = self.current_path[self.current_waypoint]

            # Send waypoint - FIXED: Always maintain 50m altitude
            setpoint = PoseStamped()
            setpoint.header.stamp = self.get_clock().now().to_msg()
            setpoint.header.frame_id = "map"
            setpoint.pose.position.x = float(waypoint[0])
            setpoint.pose.position.y = float(waypoint[1])
            setpoint.pose.position.z = 50.0  # Always fly at 50m altitude
            setpoint.pose.orientation.w = 1.0
            self.setpoint_pub.publish(setpoint)

            # Check if reached waypoint - FIXED: Only check X,Y distance (ignore Z)
            dist = math.sqrt(
                (waypoint[0] - self.current_position.x) ** 2 +
                (waypoint[1] - self.current_position.y) ** 2
            )

            if dist < 1.0:  # Reached waypoint
                self.get_logger().info(f"✅ Reached waypoint {self.current_waypoint+1}/{len(self.current_path)} "
                                     f"at ({waypoint[0]:.1f}, {waypoint[1]:.1f}) - staying at 50m altitude")
                self.current_waypoint += 1
                if self.current_waypoint >= len(self.current_path):
                    # Reached goal
                    if self.task_simulation_start is None:
                        self.get_logger().info(f"🎯 Reached final goal, starting task...")
                        self.task_simulation_start = current_time
        else:
            # No path available - wait for planner
            if self.assignment:
                self.get_logger().debug("⏳ Waiting for path from planner...")

    def publish_path_viz(self):
        """Publish path for RViz visualization (optional)"""
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for waypoint in self.current_path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = float(waypoint[0])
            pose.pose.position.y = float(waypoint[1])
            pose.pose.position.z = 50.0  # FIXED: Always visualize at 50m altitude
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    def set_guided_mode(self):
        client = self.create_client(SetMode, f'/{self.drone_id}/mavros/set_mode')
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("❌ Set mode service unavailable")
            return

        request = SetMode.Request()
        request.custom_mode = "GUIDED"
        future = client.call_async(request)

        def response_callback(future):
            try:
                result = future.result()
                if result.mode_sent:
                    self.get_logger().info("🧭 GUIDED mode set")
                else:
                    self.get_logger().warn("⚠️ Failed to set GUIDED mode")
            except Exception as e:
                self.get_logger().error(f"Mode change error: {e}")

        future.add_done_callback(response_callback)

    def arm_drone(self):
        client = self.create_client(CommandBool, f'/{self.drone_id}/mavros/cmd/arming')
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("❌ Arming service unavailable")
            return

        request = CommandBool.Request()
        request.value = True
        future = client.call_async(request)

        def response_callback(future):
            try:
                result = future.result()
                if result.success:
                    self.get_logger().info("✅ Drone armed")
                    self.armed = True
                else:
                    self.get_logger().warn("❌ Failed to arm")
            except Exception as e:
                self.get_logger().error(f"Arming error: {e}")

        future.add_done_callback(response_callback)

    def set_takeoff(self):
        client = self.create_client(CommandTOL, f'/{self.drone_id}/mavros/cmd/takeoff')
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("❌ Takeoff service unavailable")
            return

        request = CommandTOL.Request()
        request.altitude = 50.0  # FIXED: Takeoff to 50m altitude
        future = client.call_async(request)

        def response_callback(future):
            try:
                result = future.result()
                if result.success:
                    self.get_logger().info("🚀 Takeoff initiated")
                else:
                    self.get_logger().warn(f"❌ Takeoff failed: {result.result}")
            except Exception as e:
                self.get_logger().error(f"Takeoff error: {e}")

        future.add_done_callback(response_callback)


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("Usage: python drone_controller.py <drone_id>")
        sys.exit(1)

    drone_id = sys.argv[1]

    node = DroneController(drone_id)

    print(f"🚁 Started {drone_id} controller (external planner mode)")
    print(f"🔗 Make sure to run: python path_planner.py {drone_id}")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f"\n{drone_id} shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
