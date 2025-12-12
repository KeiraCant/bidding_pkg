#The allocator node acts as a centralised auction controller. Each UAV publishes a bid for a task that includes its drone ID, bid score, 
# and location. When the first bid arrives for a task, the allocator starts a 10-second timer and collects all incoming bids. 
# If all three drones submit a bid before the timer expires, it assigns the task immediately. 
# If the timer expires first, it assigns using whatever bids are available.
#Only drones that are not already busy are considered eligible. If every bidder for a task is currently occupied, 
# the task is placed into a pending queue. When a drone finishes a task and publishes a /task_done message, 
# the allocator marks it as free and checks whether any pending task now has an eligible bidder, assigning immediately if so.
#Assignments are published on /assignments and include the selected drone, the task location, and the fire priority value
# (which is stored but does not affect selection). After a task is assigned, all its associated state is cleared to prevent stale data 
# from affecting future allocations.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy

class AllocatorNode(Node):
    def __init__(self):
        super().__init__('allocator_node')

        # Task tracking
        self.active_tasks = {}      # task_id -> list of (drone_id, bid_score, location)
        self.task_timers = {}       # task_id -> bid start time
        self.pending_tasks = []     # queue of task_ids
        self.busy_drones = set()    # set of busy drone_ids
        self.bid_timeout = 10.0     # seconds to wait for bids
        self.task_priorities = {}   # task_id -> fire priority (info only)

        # QoS for publishers/subscribers
        assignment_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )  

        # Publishers and subscriptions
        self.assignment_pub = self.create_publisher(String, '/assignments', assignment_qos)
        self.create_subscription(String, '/bids', self.bid_callback, 10)
        self.create_subscription(String, '/task_done', self.task_done_callback, assignment_qos)
        self.create_subscription(String, '/fire_priority', self.fire_priority_callback, assignment_qos)

        # Timer to check for expired bids
        self.check_timer = self.create_timer(1.0, self.check_expired_tasks)

        self.get_logger().info("Allocator node ready. Fire priority included in assignments for info only.")

    # --- Drone state helpers ---
    def drone_busy(self, drone_id):
        return drone_id in self.busy_drones

    def mark_drone_busy(self, drone_id):
        self.busy_drones.add(drone_id)

    def mark_drone_free(self, drone_id):
        self.busy_drones.discard(drone_id)

    # --- Fire priority callback (info only) ---
    def fire_priority_callback(self, msg):
        try:
            data = json.loads(msg.data)
            task_id = data['task_id']
            priority = data['priority']
            self.task_priorities[task_id] = priority
            self.get_logger().info(f"Received priority {priority:.3f} for task {task_id}")
        except Exception as e:
            self.get_logger().error(f"❌ Error parsing /fire_priority: {e}")

    # --- Handle incoming bids ---
    def bid_callback(self, msg):
        try:
            bid = json.loads(msg.data)
            task_id = bid['task_id']
            drone_id = bid['drone_id']
            bid_score = bid['bid_score']
            location = bid['location']

            if task_id not in self.active_tasks:
                self.active_tasks[task_id] = []
                self.task_timers[task_id] = time.time()
                self.get_logger().info(f"Started 10-second bid timer for task {task_id}")

            self.active_tasks[task_id].append((drone_id, bid_score, location))
            self.get_logger().info(
                f"\n New bid received:\n"
                f"   Task ID: {task_id}\n"
                f"   Drone ID: {drone_id}\n"
                f"   Score: {bid_score:.2f}\n"
                f"   Location: {location}\n"
                f"   Total bids for this task: {len(self.active_tasks[task_id])}"
            )

            # If 3 bids received, assign immediately
            if len(self.active_tasks[task_id]) >= 3:
                self.get_logger().info(f"✅ All 3 drones have bid on task {task_id}, assigning immediately")
                self.try_assign_task(task_id)

        except Exception as e:
            self.get_logger().error(f"❌ Error in bid_callback: {e}")

    # --- Check for expired bid timers ---
    def check_expired_tasks(self):
        current_time = time.time()
        expired_tasks = [task_id for task_id, start in self.task_timers.items()
                         if current_time - start >= self.bid_timeout]

        for task_id in expired_tasks:
            if task_id in self.active_tasks and len(self.active_tasks[task_id]) > 0:
                self.get_logger().info(f" Bid timer expired for task {task_id} with {len(self.active_tasks[task_id])} bids")
                self.try_assign_task(task_id)
            else:
                # Clean up if no bids received
                self.task_timers.pop(task_id, None)
                self.active_tasks.pop(task_id, None)
                self.task_priorities.pop(task_id, None)

    # --- Try assigning a task if eligible drones exist ---
    def try_assign_task(self, task_id):
        if task_id not in self.active_tasks:
            return

        eligible_bids = [b for b in self.active_tasks[task_id] if not self.drone_busy(b[0])]

        if not eligible_bids:
            self.get_logger().warn(
                f"⚠️ All bidders are busy for task {task_id}. Task added to pending queue."
            )
            if task_id not in self.pending_tasks:
                self.pending_tasks.append(task_id)
            return

        self.assign_task(task_id, eligible_bids)

    # --- Assign task based solely on bid score ---
    def assign_task(self, task_id, eligible_bids):
        if task_id not in self.active_tasks:
            return

        # Select the drone with highest bid score
        best = max(eligible_bids, key=lambda x: x[1])
        drone_id, bid_score, location = best

        assignment = {
            'task_id': task_id,
            'drone_id': drone_id,
            'location': location,
            'priority': self.task_priorities.get(task_id, 0.0)  # info only
        }

        self.assignment_pub.publish(String(data=json.dumps(assignment)))
        self.get_logger().info(f" Assigned {task_id} to {drone_id} (bid: {bid_score:.2f}, priority: {assignment['priority']:.3f})")

        self.mark_drone_busy(drone_id)

        # Cleanup task
        self.active_tasks.pop(task_id, None)
        self.task_timers.pop(task_id, None)
        self.pending_tasks = [t for t in self.pending_tasks if t != task_id]
        self.task_priorities.pop(task_id, None)

    # --- Handle task completion ---
    def task_done_callback(self, msg):
        try:
            data = json.loads(msg.data)
            drone_id = data['drone_id']
            task_id = data['task_id']

            self.mark_drone_free(drone_id)
            self.get_logger().info(f" Drone {drone_id} completed task {task_id}, marked as free.")

            # Attempt pending assignments
            if self.pending_tasks:
                next_task_id = self.pending_tasks[0]
                if next_task_id in self.active_tasks:
                    eligible_bids = [b for b in self.active_tasks[next_task_id] if not self.drone_busy(b[0])]
                    if eligible_bids:
                        self.pending_tasks.pop(0)
                        self.assign_task(next_task_id, eligible_bids)

        except Exception as e:
            self.get_logger().error(f"❌ Error parsing /task_done message: {e}")

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

