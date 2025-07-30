import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy

class AllocatorNode(Node):
    def __init__(self):
        super().__init__('allocator_node')

        self.active_tasks = {}  # task_id -> list of (drone_id, bid_score, priority, location)
        self.task_priority = {}  # task_id -> priority
        self.task_timers = {}  # task_id -> timer start time
        self.pending_tasks = []  # queue of task_ids
        self.busy_drones = set()  # set of drone_ids
        self.bid_timeout = 10.0  # Wait 10 seconds for bids

        assignment_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )  

        self.assignment_pub = self.create_publisher(String, '/assignments', assignment_qos)
        self.create_subscription(String, '/bids', self.bid_callback, 10)
        self.create_subscription(String, '/task_done', self.task_done_callback, assignment_qos)

        # Timer to check for expired bid periods
        self.check_timer = self.create_timer(1.0, self.check_expired_tasks)

        self.get_logger().info("Allocator node with 10-second bid timer ready.")

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
                self.task_timers[task_id] = time.time()  # Start timer for this task
                self.get_logger().info(f"⏰ Started 10-second bid timer for task {task_id}")

            self.active_tasks[task_id].append((drone_id, bid_score, priority, location))

            self.get_logger().info(
                f"\n📨 New bid received:\n"
                f"  🔧 Task ID: {task_id}\n"
                f"  🚁 Drone ID: {drone_id}\n"
                f"  📊 Score: {bid_score:.2f}\n"
                f"  ⬆️ Priority: {priority}\n"
                f"  📍 Location: {location}\n"
                f"  📊 Total bids for this task: {len(self.active_tasks[task_id])}"
            )

            # Check if we have all 3 drones bidding
            if len(self.active_tasks[task_id]) >= 3:
                self.get_logger().info(f"✅ All 3 drones have bid on task {task_id}, assigning immediately")
                self.try_assign_task(task_id)

        except Exception as e:
            self.get_logger().error(f"❌ Error in bid_callback: {e}")

    def check_expired_tasks(self):
        """Check for tasks whose bid timer has expired"""
        current_time = time.time()
        expired_tasks = []
        
        for task_id, start_time in self.task_timers.items():
            if current_time - start_time >= self.bid_timeout:
                expired_tasks.append(task_id)
        
        for task_id in expired_tasks:
            if task_id in self.active_tasks and len(self.active_tasks[task_id]) > 0:
                self.get_logger().info(f"⏰ Bid timer expired for task {task_id} with {len(self.active_tasks[task_id])} bids")
                self.try_assign_task(task_id)
            else:
                # Clean up if no bids received
                if task_id in self.task_timers:
                    del self.task_timers[task_id]
                if task_id in self.active_tasks:
                    del self.active_tasks[task_id]
                if task_id in self.task_priority:
                    del self.task_priority[task_id]

    def try_assign_task(self, task_id):
        """Try to assign a task to an available drone"""
        if task_id not in self.active_tasks:
            return

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
        self.get_logger().info(f"✅ Assigned {task_id} to {drone_id} (bid score: {best[1]:.2f})")

        self.mark_drone_busy(drone_id)

        # Clean up task data
        del self.active_tasks[task_id]
        if task_id in self.task_timers:
            del self.task_timers[task_id]
        if task_id in self.task_priority:
            del self.task_priority[task_id]
        if task_id in self.pending_tasks:
            self.pending_tasks.remove(task_id)

    def task_done_callback(self, msg):
        try:
            data = json.loads(msg.data)
            drone_id = data['drone_id']
            task_id = data['task_id']

            self.mark_drone_free(drone_id)
            self.get_logger().info(f"🔓 Drone {drone_id} completed task {task_id}, marked as free.")

            # Try to assign any pending tasks
            if self.pending_tasks:
                next_task_id = self.pending_tasks[0]  # Don't pop yet
                self.get_logger().info(f"🔄 Trying to assign pending task {next_task_id}")
                
                if next_task_id in self.active_tasks:
                    eligible_bids = [
                        b for b in self.active_tasks[next_task_id] if not self.drone_busy(b[0])
                    ]
                    if eligible_bids:
                        self.pending_tasks.pop(0)  # Remove from pending only if we can assign
                        self.assign_task(next_task_id, eligible_bids)

        except Exception as e:
            self.get_logger().error(f"Error parsing /task_done message: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = AllocatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Allocator shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
