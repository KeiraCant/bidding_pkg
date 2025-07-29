import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import json
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.clock import Clock
import math
import time

class Drone1Controller(Node):
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
        self.task_simulation_start = None
        self.task_simulation_duration = 5.0

        # QoS Profile
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers
        self.setpoint_pub = self.create_publisher(PoseStamped, f'/{drone_id}/mavros/setpoint_position/local', qos)
        self.pose_broadcast_pub = self.create_publisher(PoseStamped, f'/{drone_id}/pose_broadcast', qos)
        self.path_pub = self.create_publisher(Path, f'/planned_path_{drone_id}', qos)
        self.done_pub = self.create_publisher(String, '/task_done', qos)

        # Subscribers
        self.create_subscription(PoseStamped, f'/{drone_id}/mavros/local_position/pose', self.pose_callback, qos)
        self.create_subscription(String, '/assignments', self.assignment_callback, qos)
        self.create_subscription(String, '/task_done', self.task_done_callback, qos)

        # Subscribe to other drones' pose broadcasts
        other_drones = [f"drone_{i}" for i in range(1, 4) if f"drone_{i}" != drone_id]
        for other_id in other_drones:
            topic = f'/{other_id}/pose_broadcast'
            self.create_subscription(PoseStamped, topic, self._pose_update_callback_factory(other_id), qos)

        # Timer to send waypoints
        self.timer = self.create_timer(0.2, self.send_waypoint)

        self.get_logger().info(f"✅ {drone_id} controller node started")

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
        self.get_logger().info(f"📦 Received assignment: {data}")
        self.assignment = data
        
        # Reset collision avoidance state
        self.collision_wait_start = None
        self.last_retry_time = 0
        self.task_simulation_start = None
        
        self.arm_drone()
        self.set_guided_mode()
        if self.current_position and self.is_path_safe(self.assignment['location']):
            self.publish_path(self.current_position, self.assignment['location'])
        else:
            self.get_logger().warn("⚠️ Planned path conflicts with other drone(s). Will retry.")
            self.collision_wait_start = time.time()

    def task_done_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if data['drone_id'] == self.drone_id and self.assignment and data['task_id'] == self.assignment['task_id']:
                self.get_logger().info(f"🔥 Task {data['task_id']} complete. Stopping waypoint publishing.")
                self.assignment = None
                self.collision_wait_start = None
                self.task_simulation_start = None
        except Exception as e:
            self.get_logger().error(f"Error in task_done_callback: {e}")

    def send_waypoint(self):
        if self.assignment is None:
            return  # Remove the warning to reduce log spam
            
        if self.current_position is None:
            self.get_logger().warn("No current position available.")
            return
            
        if not self.armed:
            self.get_logger().warn("Drone not armed yet.")
            return

        goal = self.assignment['location']
        current_time = time.time()
        
        # Check if we're currently simulating task completion
        if self.task_simulation_start is not None:
            if current_time - self.task_simulation_start >= self.task_simulation_duration:
                # Task simulation complete
                done_msg = {"task_id": self.assignment['task_id'], "drone_id": self.drone_id}
                self.done_pub.publish(String(data=json.dumps(done_msg)))
                self.get_logger().info(f"✅ Task {self.assignment['task_id']} completed.")
                self.assignment = None
                self.task_simulation_start = None
            return  # Don't send waypoints while simulating task
        
        # Check if path is safe
        if not self.is_path_safe(goal):
            # Start collision avoidance if not already started
            if self.collision_wait_start is None:
                self.collision_wait_start = current_time
                self.get_logger().warn("⚠️ Collision detected. Starting avoidance procedure.")
                return
            
            # Check if we should retry (non-blocking)
            if current_time - self.last_retry_time < self.retry_interval:
                return  # Not time to retry yet
                
            self.last_retry_time = current_time
            
            # Check if we've been waiting too long
            if current_time - self.collision_wait_start > self.max_wait_time:
                self.get_logger().warn(f"⏰ Collision avoidance timeout after {self.max_wait_time}s. Proceeding with caution.")
                # Reset and proceed - or implement alternative strategy
                self.collision_wait_start = None
                # Could implement alternative path planning here
            else:
                wait_time = current_time - self.collision_wait_start
                self.get_logger().warn(f"🚫 Path blocked. Waiting for clearance... ({wait_time:.1f}s)")
                return
        else:
            # Path is safe, reset collision avoidance state
            if self.collision_wait_start is not None:
                self.get_logger().info("✅ Path cleared. Proceeding to target.")
                self.collision_wait_start = None
        
        # Send waypoint
        setpoint = PoseStamped()
        setpoint.header.stamp = self.get_clock().now().to_msg()
        setpoint.header.frame_id = "map"
        setpoint.pose.position.x = float(goal[0])
        setpoint.pose.position.y = float(goal[1])
        setpoint.pose.position.z = float(goal[2])
        setpoint.pose.orientation.w = 1.0
        self.setpoint_pub.publish(setpoint)
        
        # Log waypoint (less frequently to reduce spam)
        if current_time - getattr(self, 'last_waypoint_log', 0) > 2.0:
            self.get_logger().info(f"🚁 Publishing waypoint to fire: ({goal[0]:.2f}, {goal[1]:.2f}, {goal[2]:.2f})")
            self.last_waypoint_log = current_time
        
        # Check if we've reached the goal
        dist_to_goal = math.sqrt(
            (goal[0] - self.current_position.x)**2 +
            (goal[1] - self.current_position.y)**2 +
            (goal[2] - self.current_position.z)**2
        )
        
        if dist_to_goal < 1.0:
            if self.task_simulation_start is None:
                self.get_logger().info(f"🎯 Reached task {self.assignment['task_id']}. Starting task simulation...")
                self.task_simulation_start = current_time

    def is_path_safe(self, goal_location, safety_radius=5.0):
        for drone_id, pose_msg in self.drone_poses.items():
            pos = pose_msg.pose.position
            dist = math.sqrt(
                (goal_location[0] - pos.x)**2 +
                (goal_location[1] - pos.y)**2 +
                (goal_location[2] - pos.z)**2
            )
            if dist < safety_radius:
                self.get_logger().debug(f"Collision risk: {drone_id} is {dist:.2f}m away, less than {safety_radius}m")
                return False
        return True

    def publish_path(self, start_pose, goal_location):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for point in [start_pose, goal_location]:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            if isinstance(point, (tuple, list)):
                pose.pose.position.x = float(point[0])
                pose.pose.position.y = float(point[1])
                pose.pose.position.z = float(point[2])
            else:
                pose.pose.position.x = point.x
                pose.pose.position.y = point.y
                pose.pose.position.z = point.z
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)
        self.get_logger().info(
            f"📍 Published path from [{start_pose.x:.2f}, {start_pose.y:.2f}, {start_pose.z:.2f}] "
            f"to [{goal_location[0]:.2f}, {goal_location[1]:.2f}, {goal_location[2]:.2f}]"
        )

    def arm_drone(self):
        client = self.create_client(CommandBool, f'/{self.drone_id}/mavros/cmd/arming')
        retries = 5
        while not client.wait_for_service(timeout_sec=1.0) and retries > 0:
            self.get_logger().warn(f'🕓 Waiting for /{self.drone_id}/mavros/cmd/arming service... ({retries} retries left)')
            retries -= 1
            time.sleep(1)
        if retries == 0:
            self.get_logger().error("❌ Arming service unavailable")
            return
        request = CommandBool.Request()
        request.value = True
        future = client.call_async(request)
        def response_callback(future):
            try:
                result = future.result()
                self.get_logger().debug(f"Arming response: success={result.success}, result={result.result}")
                if result.success:
                    self.get_logger().info("✅ Drone armed successfully.")
                    self.armed = True
                else:
                    self.get_logger().warn("❌ Drone failed to arm. Check flight mode and safety settings.")
            except Exception as e:
                self.get_logger().error(f"Error during arming: {e}")
        future.add_done_callback(response_callback)

    def set_guided_mode(self):
        client = self.create_client(SetMode, f'/{self.drone_id}/mavros/set_mode')
        retries = 5
        while not client.wait_for_service(timeout_sec=1.0) and retries > 0:
            self.get_logger().warn(f'🕓 Waiting for /{self.drone_id}/mavros/set_mode service... ({retries} retries left)')
            retries -= 1
            time.sleep(1)
        if retries == 0:
            self.get_logger().error("❌ Set mode service unavailable")
            return
        request = SetMode.Request()
        request.custom_mode = "GUIDED"
        future = client.call_async(request)
        self.get_logger().info("🧭 Setting flight mode to GUIDED...")

def main(args=None):
    rclpy.init(args=args)
    node = Drone1Controller('drone_1')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
