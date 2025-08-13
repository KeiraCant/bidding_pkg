
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from rclpy.qos import QoSProfile, ReliabilityPolicy
class FireDataListener(Node):
    def __init__(self):
        super().__init__('fire_data_listener')
        qos= QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        self.subscription = self.create_subscription(
            String,
            '/fire_data',
            self.listener_callback,
            qos)
        self.subscription

    def listener_callback(self, msg):
        try:
            data = json.loads(msg.data)
            pretty = json.dumps(data, indent=2)
            print(f"Received fire data:\n{pretty}\n")
        except Exception as e:
            print(f"Failed to parse JSON: {e}\nRaw message:\n{msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = FireDataListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

