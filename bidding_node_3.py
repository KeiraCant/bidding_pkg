import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
import json
import random
from rclpy.qos import QoSProfile, ReliabilityPolicy
import threading
import math
import time


class BiddingNode(Node):
    def __init__(self, drone_id):
        super().__init__('bidding_node_' + drone_id)
        self.drone_id = drone_id
        self.current_task = None
        self.position = None  # Updated from MAVROS pose subscriber
        self.initialized = False 
        # QoS profile with BEST_EFFORT reliability for MAVROS pose and assignments
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Subscribe to MAVROS local position pose
        self.create_subscription(
            PoseStamped,
            f'/{drone_id}/mavros/local_position/pose',
            self.pose_callback,
            best_effort_qos
        )

        # Subscribe to fire tasks (default QoS depth=10)
        self.create_subscription(
            String,
            '/fire_tasks',
            self.fire_callback,
            best_effort_qos
        )

        # Subscribe to assignments with BEST_EFFORT QoS for compatibility
        self.create_subscription(
            String,
            '/assignments',
            self.assignment_callback,
            best_effort_qos
        )

        # Subscribe to task completion to reset state
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
        self.fire_marker_pub = self.create_publisher(Marker, f'/fire_marker_{drone_id}', 10)
        self.setpoint_pub = self.create_publisher(PoseStamped, f'/{drone_id}/mavros/setpoint_position/local', 10)

        self.get_logger().info(f"🚁 {drone_id} bidding node started and waiting for pose...")

    def pose_callback(self, msg):
        self.position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

        if not self.initialized:
            self.get_logger().info(f"{self.drone_id} initialized at {self.position}")
            self.initialized = True

    def fire_callback(self, msg):
        if self.current_task is not None or not self.initialized:
            return

        fire = json.loads(msg.data)
        bid_score = random.randint(1, 100)
        bid = {
            'task_id': fire['task_id'],
            'drone_id': self.drone_id,
            'bid_score': bid_score,
            'priority': fire.get('priority', 1),
            'location': fire['location']
        }
        self.bid_pub.publish(String(data=json.dumps(bid)))
        self.get_logger().info(f"Sent bid: {bid}")

        self.publish_fire_marker(fire['location'], fire['task_id'])

    def assignment_callback(self, msg):
        assignment = json.loads(msg.data)
        if assignment['drone_id'] == self.drone_id and self.current_task is None:
            task_id = assignment['task_id']
            fire_loc = assignment['location']

            self.current_task = task_id
            self.get_logger().info(f"Assigned {task_id}. Planning path...")

            self.publish_path(self.position, fire_loc)
            
    def task_done_callback(self, msg):
        self.get_logger().info(f"📥 Received task_done message: {msg.data}")
        try:
            data = json.loads(msg.data)
            self.get_logger().info(f"🔎 Parsed data: {data}, current task: {self.current_task}")
            if data['drone_id'] == self.drone_id:
                if self.current_task == data['task_id']:
                    self.get_logger().info(f"✅ Task {data['task_id']} complete. Resetting...")
                    self.current_task = None
        except Exception as e:
            self.get_logger().error(f"❗ Error processing task completion: {e}")



    def publish_path(self, start, goal):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for x, y, z in [start, goal]:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = float(z)
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info(f"Published path from {start} to {goal}")

    def publish_fire_marker(self, location, task_id):
        """Publish fire marker for visualization"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "fires"
        marker.id = int(task_id.split('_')[-1]) if '_' in task_id else hash(task_id) % 1000
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = float(location[0])
        marker.pose.position.y = float(location[1])
        marker.pose.position.z = float(location[2])
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        self.fire_marker_pub.publish(marker)
def main(args=None):
    rclpy.init(args=args)
    node = BiddingNode('drone_3')  # Change for each drone
    rclpy.spin(node)
    rclpy.shutdown()

