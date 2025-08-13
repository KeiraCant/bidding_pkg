import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
import json
import random
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math

class BiddingNode(Node):
    def __init__(self, drone_id):
        super().__init__('bidding_node_' + drone_id)
        self.drone_id = drone_id
        self.current_task = None
        self.position = None  # Drone's local pose [x, y, z]
        self.initialized = False

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Subscribe to drone local position pose
        self.create_subscription(
            PoseStamped,
            f'/{drone_id}/mavros/local_position/pose',
            self.pose_callback,
            best_effort_qos
        )

        # Subscribe to fire tasks with GPS locations
        self.create_subscription(
            String,
            '/fire_tasks',
            self.fire_callback,
            best_effort_qos
        )

        # Subscribe to assignments
        self.create_subscription(
            String,
            '/assignments',
            self.assignment_callback,
            best_effort_qos
        )

        # Subscribe to task completion messages
        self.create_subscription(
            String,
            '/task_done',
            self.task_done_callback,
            best_effort_qos
        )

        # Publishers
        self.bid_pub = self.create_publisher(String, '/bids', 10)
        self.done_pub = self.create_publisher(String, '/task_done', 10)
        self.path_pub = self.create_publisher(Path, f'/planned_path_{drone_id}', 10)
        self.setpoint_pub = self.create_publisher(PoseStamped, f'/{drone_id}/mavros/setpoint_position/local', 10)

        self.get_logger().info(f"🚁 {drone_id} bidding node started and waiting for pose...")

        # Reference origin for local frame, must match FireTaskPublisher sim_center
        self.sim_center_lat = 63.4188137258128
        self.sim_center_lon = 10.401553034756608

    def pose_callback(self, msg):
        self.position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        if not self.initialized:
            self.get_logger().info(f"{self.drone_id} initialized at position {self.position}")
            self.initialized = True

    def gps_to_local(self, lat, lon):
        delta_lat = lat - self.sim_center_lat
        delta_lon = lon - self.sim_center_lon

        meters_per_deg_lat = 110540
        meters_per_deg_lon = 111320 * math.cos(math.radians(self.sim_center_lat))

        x_local = delta_lon * meters_per_deg_lon
        y_local = delta_lat * meters_per_deg_lat

        return [x_local, y_local, 0.0]

    def euclidean_distance(self, pos1, pos2):
        return math.sqrt(
            (pos1[0] - pos2[0])**2 +
            (pos1[1] - pos2[1])**2 +
            (pos1[2] - pos2[2])**2
        )

    def distance_to_bid_score(self, distance):
        # Bid score inversely proportional to distance (closer fires get higher bids)
        # For example: max score 100, min 1, with a max considered distance of 2000 m
        max_distance = 5000
        normalized = max(0.0, min(1.0, 1 - (distance / max_distance)))
        bid_score = int(normalized * 99) + 1
        return bid_score

    def fire_callback(self, msg):
        if self.current_task is not None or not self.initialized:
            return

        fire = json.loads(msg.data)
        fire_lat, fire_lon = fire['location']

        if self.position is None:
            self.get_logger().warn("Position not yet available, skipping bid.")
            return

        fire_local = self.gps_to_local(fire_lat, fire_lon)
        dist = self.euclidean_distance(self.position, fire_local)
        bid_score = self.distance_to_bid_score(dist)

        bid = {
            'task_id': fire['task_id'],
            'drone_id': self.drone_id,
            'bid_score': bid_score,
            'location': fire['location']  # Keep GPS coords here
        }

        self.bid_pub.publish(String(data=json.dumps(bid)))
        self.get_logger().info(f"Sent bid for task {fire['task_id']} based on distance {dist:.2f} m: {bid}")

    def assignment_callback(self, msg):
        assignment = json.loads(msg.data)
        if assignment['drone_id'] == self.drone_id and self.current_task is None:
            task_id = assignment['task_id']
            fire_loc = assignment['location']

            self.current_task = task_id
            self.get_logger().info(f"Assigned task {task_id}. Planning path...")

            self.publish_path(self.position, self.gps_to_local(fire_loc[0], fire_loc[1]))

    def task_done_callback(self, msg):
        self.get_logger().info(f"📥 Received task_done message: {msg.data}")
        try:
            data = json.loads(msg.data)
            if data['drone_id'] == self.drone_id and self.current_task == data['task_id']:
                self.get_logger().info(f"✅ Task {data['task_id']} complete. Resetting current task.")
                self.current_task = None
        except Exception as e:
            self.get_logger().error(f"❗ Error processing task completion: {e}")

    def publish_path(self, start, goal):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for point in [start, goal]:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.position.z = float(point[2])
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info(f"Published path from {start} to {goal}")

def main(args=None):
    rclpy.init(args=args)
    node = BiddingNode('drone_1')  # Change for each drone
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

