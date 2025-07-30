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

# Updated imports for drone_controller.py
from path_planner import (
    PathPlanner, DirectPlanner, WaypointPlanner, AvoidancePlanner,
    AstarPlanner, RRTStarPlanner, DijkstraPlanner
)

# Updated PLANNERS dictionary
PLANNERS = {
    'direct': DirectPlanner,
    'waypoint': WaypointPlanner, 
    'avoidance': AvoidancePlanner,
    'astar': AstarPlanner,
    'rrt_star': RRTStarPlanner,
    'rrtstar': RRTStarPlanner,  # Alternative name
    'dijkstra': DijkstraPlanner,
}
    
    
class DroneController(Node):
    def __init__(self, drone_id, planner_name):
        super().__init__(f'{drone_id}_controller')
        self.drone_id = drone_id
        self.current_position = None
        self.assignment = None
        self.drone_poses = {}  # dict {drone_id: PoseStamped}
        self.armed = False
        
        if planner_name in PLANNERS:
            self.planner = PLANNERS[planner_name](drone_id, self.get_logger())
            self.get_logger().info(f"🧭 Using {planner_name} planner")
        else:
            self.get_logger().warn(f"Unknown planner '{planner_name}', using direct")
            self.planner = DirectPlanner(drone_id, self.get_logger())
            
        
        # Enhanced collision avoidance state
        self.collision_wait_start = None
        self.max_wait_time = 10.0  # Maximum time to wait for collision resolution
        self.retry_interval = 1.0  # How often to retry when waiting
        self.last_retry_time = 0
        
        # Task completion state
        self.task_simulation_start = None
        self.task_simulation_duration = 5.0
        
        # Path following
        self.current_path = []
        self.current_waypoint = 0

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
            
        self.get_logger().info(f"📦 Assignment: {data}")
        self.assignment = data
        
        # Reset everything
        self.collision_wait_start = None
        self.task_simulation_start = None
        self.current_path = []
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

    def send_waypoint(self):
        if not self.assignment or not self.current_position or not self.armed:
            return
            
        current_time = time.time()
        goal = self.assignment['location']
        
        # Handle task simulation
        if self.task_simulation_start is not None:
            if current_time - self.task_simulation_start >= self.task_simulation_duration:
                done_msg = {"task_id": self.assignment['task_id'], "drone_id": self.drone_id}
                self.done_pub.publish(String(data=json.dumps(done_msg)))
                self.get_logger().info(f"✅ Task completed")
                self.assignment = None
                self.task_simulation_start = None
            return
        
        # Plan path if needed
        if not self.current_path:
            if not self.planner.is_safe(self.current_position, goal, self.drone_poses):
                # Handle collision avoidance
                if self.collision_wait_start is None:
                    self.collision_wait_start = current_time
                    self.get_logger().warn("⚠️ Path blocked, waiting...")
                    
                if current_time - self.collision_wait_start > self.max_wait_time:
                    self.get_logger().warn("⏰ Timeout, forcing path...")
                    self.collision_wait_start = None
                else:
                    return  # Keep waiting
            
            # Plan the path
            try:
                self.current_path = self.planner.plan(self.current_position, goal)
                self.current_waypoint = 0
                self.collision_wait_start = None
                
                self.get_logger().info(f"🗺️ Planned path with {len(self.current_path)} waypoints")
                self.publish_path_viz()
                
            except Exception as e:
                self.get_logger().error(f"❌ Planning failed: {e}")
                return
        
        # Follow the path
        if self.current_waypoint < len(self.current_path):
            waypoint = self.current_path[self.current_waypoint]
            
            # Send waypoint
            setpoint = PoseStamped()
            setpoint.header.stamp = self.get_clock().now().to_msg()
            setpoint.header.frame_id = "map"
            setpoint.pose.position.x = float(waypoint[0])
            setpoint.pose.position.y = float(waypoint[1])
            setpoint.pose.position.z = float(waypoint[2])
            setpoint.pose.orientation.w = 1.0
            self.setpoint_pub.publish(setpoint)
            
            # Check if reached waypoint
            dist = math.sqrt(
                (waypoint[0] - self.current_position.x)**2 +
                (waypoint[1] - self.current_position.y)**2 +
                (waypoint[2] - self.current_position.z)**2
            )
            
            if dist < 1.0:  # Reached waypoint
                self.current_waypoint += 1
                if self.current_waypoint >= len(self.current_path):
                    # Reached goal
                    if self.task_simulation_start is None:
                        self.get_logger().info(f"🎯 Reached goal, starting task...")
                        self.task_simulation_start = current_time

    def publish_path_viz(self):
        """Publish path for RViz visualization"""
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for waypoint in self.current_path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = float(waypoint[0])
            pose.pose.position.y = float(waypoint[1])
            pose.pose.position.z = float(waypoint[2])
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
        request.altitude = 10.0
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
        print("Usage: python drone_controller.py <drone_id> [planner]")
        print("Available planners:", list(PLANNERS.keys()))
        sys.exit(1)
    
    drone_id = sys.argv[1]
    planner_name = sys.argv[2] if len(sys.argv) > 2 else 'direct'
    
    node = DroneController(drone_id, planner_name)
    
    print(f"🚁 Started {drone_id} with '{planner_name}' planner")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f"\n{drone_id} shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

