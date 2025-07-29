import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class DronePosePublisher(Node):
    def __init__(self, drone_id, start_x=0.0, start_y=0.0):
        super().__init__(f'drone_{drone_id}_pose_publisher')
        self.pub = self.create_publisher(PoseStamped, f'/drone_{drone_id}/pose', 10)
        self.timer = self.create_timer(0.5, self.publish_pose)
        self.x = start_x
        self.y = start_y

    def publish_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'  # Must match RViz Fixed Frame

        # Simulate drone movement by incrementing position
        self.x += 0.1
        self.y += 0.1

        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.position.z = self.z

        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.pub.publish(msg)
        self.get_logger().info(f'Drone {self.get_name()} position: ({self.x:.2f}, {self.y:.2f})')

def main(args=None):
    rclpy.init(args=args)
    # Create two drone publishers with different starting points
    drone1 = DronePosePublisher(1, start_x=0.0, start_y=0.0)
    drone2 = DronePosePublisher(2, start_x=5.0, start_y=5.0)

    # Spin both nodes in parallel
    try:
        rclpy.spin(drone1)
    except KeyboardInterrupt:
        pass

    drone1.destroy_node()
    drone2.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

