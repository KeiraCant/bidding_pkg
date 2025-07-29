import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class DronePathPublisher(Node):
    def __init__(self):
        super().__init__('drone_path_publisher')

        self.paths = {}  # Store Path message per drone
        self.drones = ['uav1', 'uav2', 'uav3']

        self.subscribers = []
        self.publishers = []

        for drone in self.drones:
            self.paths[drone] = Path()
            self.paths[drone].header.frame_id = 'map'  # Set your fixed frame

            # Subscribe to drone pose topic
            sub = self.create_subscription(
                PoseStamped,
                f'/{drone}/pose',
                lambda msg, d=drone: self.pose_callback(msg, d),
                10)
            self.subscribers.append(sub)

            # Publisher for path topic
            pub = self.create_publisher(Path, f'/{drone}/path', 10)
            self.publishers.append(pub)

    def pose_callback(self, msg, drone):
        path = self.paths[drone]
        path.header.stamp = msg.header.stamp
        path.poses.append(msg)

        # Keep path size manageable
        if len(path.poses) > 500:
            path.poses.pop(0)

        idx = self.drones.index(drone)
        self.publishers[idx].publish(path)

def main(args=None):
    rclpy.init(args=args)
    node = DronePathPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
