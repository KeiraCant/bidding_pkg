import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker
import json

class TaskVisualizer(Node):
    def __init__(self):
        super().__init__('task_visualizer')

        # Subscriber to /tasks topic (expects String messages with JSON data)
        self.subscription = self.create_subscription(
            String,
            '/tasks',
            self.task_callback,
            10
        )

        # Publisher for visualization markers
        self.marker_pub = self.create_publisher(Marker, '/task_marker', 10)

        self.get_logger().info("Task Visualizer node started. Waiting for tasks...")

    def task_callback(self, msg):
        try:
            data = json.loads(msg.data)
            task_id = data['task_id']
            location = data['location']  # expecting [x, y] or [x, y, z]

            # Create a Marker message
            marker = Marker()
            marker.header.frame_id = 'map'   # Set your fixed frame here
            marker.header.stamp = self.get_clock().now().to_msg()

            marker.ns = 'tasks'
            marker.id = hash(task_id) % 1000  # Unique ID for each task

            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            # Set position (assuming 2D, z=0)
            marker.pose.position.x = float(location[0])
            marker.pose.position.y = float(location[1])
            marker.pose.position.z = 0.0

            # No rotation
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            # Scale (size of sphere)
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0

            # Color (red, fully opaque)
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            # Publish the marker
            self.marker_pub.publish(marker)

            self.get_logger().info(f"Published visualization for task {task_id} at {location}")

        except Exception as e:
            self.get_logger().error(f"Failed to process task message: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TaskVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

