import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from rclpy.qos import QoSProfile, ReliabilityPolicy

class AllocatorNode(Node):
    def __init__(self):
        super().__init__('allocator_node')

        self.active_tasks = {}  # task_id -> list of (drone_id, bid_score, priority, location)
        self.task_priority = {}  # task_id -> priority
        self.pending_tasks = []  # queue of task_ids
        self.busy_drones = set()  # set of drone_ids

        assignment_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )  

        self.assignment_pub = self.create_publisher(String, '/assignments', assignment_qos)
        self.create_subscription(String, '/bids', self.bid_callback, 10)
        self.create_subscription(String, '/task_done', self.task_done_callback, assignment_qos)

        self.get_logger().info("Allocator node with task queue ready.")

    def drone_busy(self, drone_id):
        return drone_id in self.busy_drones

    def mark_drone_busy(self, drone_id):
        self.busy_drones.add(drone_id)

    def mark_drone_free(self, drone_id):
        self.busy_drones.discard(drone_id)

    def bid_callback(self, msg):
        try:
            bid = json.loads(msg.data)
            task_id = bid['task_id']
            drone_id = bid['drone_id']
            bid_score = bid['bid_score']
            priority = bid.get('priority', 1)
            location = bid['location']

            if task_id not in self.active_tasks:
                self.active_tasks[task_id] = []
                self.task_priority[task_id] = priority

            self.active_tasks[task_id].append((drone_id, bid_score, priority, location))

            self.get_logger().info(
                f"\n📨 New bid received:\n"
                f"  🔧 Task ID: {task_id}\n"
                f"  🚁 Drone ID: {drone_id}\n"
                f"  📊 Score: {bid_score:.2f}\n"
                f"  ⬆️ Priority: {priority}\n"
                f"  📍 Location: {location}"
            )

            # Wait until all drones (assume 3) bid on this task
            if len(self.active_tasks[task_id]) >= 3:
                eligible_bids = [
                    b for b in self.active_tasks[task_id] if not self.drone_busy(b[0])
                ]

                if not eligible_bids:
                    self.get_logger().warn(
                        f"⚠️ All bidders are busy for task {task_id}. Task added to pending queue."
                    )
                    if task_id not in self.pending_tasks:
                        self.pending_tasks.append(task_id)
                    return

                self.assign_task(task_id, eligible_bids)

        except Exception as e:
            self.get_logger().error(f"❌ Error in bid_callback: {e}")

    def assign_task(self, task_id, eligible_bids):
        if task_id not in self.active_tasks:
            self.get_logger().warn(f"Task {task_id} already assigned or not active.")
            return

        best = min(eligible_bids, key=lambda x: x[1])
        drone_id = best[0]
        location = best[3]

        assignment = {
            'task_id': task_id,
            'drone_id': drone_id,
            'location': location
        }

        self.assignment_pub.publish(String(data=json.dumps(assignment)))
        self.get_logger().info(f"✅ Assigned {task_id} to {drone_id}")

        self.mark_drone_busy(drone_id)

        del self.active_tasks[task_id]
        if task_id in self.pending_tasks:
            self.pending_tasks.remove(task_id)

    def task_done_callback(self, msg):
        try:
            data = json.loads(msg.data)
            drone_id = data['drone_id']
            task_id = data['task_id']

            self.mark_drone_free(drone_id)
            self.get_logger().info(f"Drone {drone_id} completed task {task_id}, marked as free.")

            # Try to assign any pending tasks
            if self.pending_tasks:
                next_task_id = self.pending_tasks.pop(0)
                self.get_logger().info(f"Trying to assign pending task {next_task_id}")
                if next_task_id in self.active_tasks:
                    eligible_bids = [
                        b for b in self.active_tasks[next_task_id] if not self.drone_busy(b[0])
                    ]
                    if eligible_bids:
                        self.assign_task(next_task_id, eligible_bids)

        except Exception as e:
            self.get_logger().error(f"Error parsing /task_done message: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = AllocatorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

