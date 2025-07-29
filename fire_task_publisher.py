import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import random

class FireTaskPublisher(Node):
    def __init__(self):
        super().__init__('fire_task_publisher')
        self.publisher_ = self.create_publisher(String, '/fire_tasks', 30)
        self.task_id = 0

        # Only publish if allowed
        self.allow_publish = True
        self.current_fires = []  # Track fires sent but not yet completed
        self.timer = self.create_timer(30.0, self.try_publish_fire_task)

        # Listen for task completion
        self.subscription = self.create_subscription(
            String,
            '/task_done',
            self.on_task_done,
            10
        )

    def try_publish_fire_task(self):
        if self.allow_publish:
            self.publish_fire_task()

    def publish_fire_task(self):
        self.task_id += 1
        fire = {
            'task_id': f'fire_{self.task_id}',
            'location': [random.uniform(-50, 50), random.uniform(-50, 50), random.uniform(0,50)],
            'priority': random.randint(1, 5)
        }
        self.current_fires.append(fire['task_id'])
        msg = String(data=json.dumps(fire))
        self.publisher_.publish(msg)
        self.get_logger().info(f"🔥 Published fire task: {fire}")
        self.allow_publish = False  # Disable until task is done

    def on_task_done(self, msg):
        drone_report = msg.data  # expected: 'fire_3 done by drone_2'
        self.get_logger().info(f"✅ Task completed: {drone_report}")

        # Re-enable publishing
        self.allow_publish = True

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(FireTaskPublisher())
    rclpy.shutdown()

