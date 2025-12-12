#FireTaskPublisher node simulates fire events by periodically publishing fire task messages containing unique IDs and geographic coordinates to the 
# /fire_tasks topic. It iterates through a predefined list of fire locations and publishes one fire task every 30 seconds, allowing other nodes in the 
# system to subscribe and react to new fire incidents. The node logs its publishing activity for monitoring.
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime
import json


class FireTaskPublisher(Node):
    def __init__(self):
        super().__init__('fire_task_publisher')
        
        # Publisher for fire tasks (just ID and coordinates)
        self.publisher_ = self.create_publisher(String, '/fire_tasks', 10)
        
        self.task_id = 0
        self.fire_index = 0
        
        # Predefined fires with lat, lon only
        self.predefined_fires = [
            {'location': [63.4215217, 10.3997097]},
            #{'location': [63.4195320, 10.4019966]},
            #{'location': [63.4174508, 10.4037534]},
            #{'location': [63.4144163, 10.4015530]},
            #{'location': [63.4296754, 10.3975590]},
            #{'location': [63.4192631, 10.4032286]},
            #{'location': [63.4415407, 10.4152022]},
            #{'location': [63.4000059, 10.3790786]},
            #{'location': [63.4224137, 10.4320072]},
            #{'location': [63.4179092, 10.4002086]}
        ]
        
        # Timer to publish fires every 30 seconds
        self.fire_timer = self.create_timer(30.0, self.publish_fire_task)
        
        self.get_logger().info(" Fire Task Publisher started")
    def publish_log(self, text):
        """Publish log message to UI and console"""
        msg = String()
        msg.data = text
        self.log_pub.publish(msg)
        self.get_logger().info(text)
        
    def publish_fire_task(self):
        if self.fire_index >= len(self.predefined_fires):
            self.get_logger().info("✅ All predefined fires published")
            return
        
        self.task_id += 1
        fire_id = f'fire_{self.task_id}'
        fire_location = self.predefined_fires[self.fire_index]['location']
        self.fire_index += 1
        
        fire_task = {
            'task_id': fire_id,
            'location': fire_location
        }
        
        msg = String()
        msg.data = json.dumps(fire_task)
        self.publisher_.publish(msg)
        
        self.get_logger().info(f"🔥 Published fire task: {fire_task}")

def main(args=None):
    rclpy.init(args=args)
    node = FireTaskPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(" Fire Task Publisher shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

