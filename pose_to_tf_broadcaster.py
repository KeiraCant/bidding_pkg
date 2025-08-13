import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
class PoseToTFBroadcaster(Node):
    def __init__(self):
        super().__init__('pose_to_tf_broadcaster')
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Publisher for standardized pose topic that your path planner expects
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/drone_1/pose',  # This matches what your path planner subscribes to
            qos
        )
        
        # Subscribe to the MAVROS pose
        self.subscription = self.create_subscription(
            PoseStamped,
            '/drone_1/mavros/local_position/pose',
            self.pose_callback,
            qos
        )
        
        self.get_logger().info("Enhanced Pose to TF Broadcaster started.")
        self.get_logger().info("Publishing pose to /drone_1/pose for path planner")
    
    def pose_callback(self, msg):
        # Broadcast TF transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'          # Parent frame
        t.child_frame_id = 'base_link'     # Child frame (the drone body frame)
        
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)
        
        # Also republish the pose message to the topic your path planner expects
        standardized_pose = PoseStamped()
        standardized_pose.header = msg.header
        standardized_pose.header.frame_id = 'map'  # Ensure consistent frame
        standardized_pose.pose = msg.pose
        self.get_logger().info(
        f"Republishing to /drone_1/pose: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}, z={msg.pose.position.z:.2f}")

        self.pose_publisher.publish(standardized_pose)

def main(args=None):
    rclpy.init(args=args)
    node = PoseToTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
