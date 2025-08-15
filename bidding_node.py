import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import math
from sensor_msgs.msg import NavSatFix
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class BiddingNode(Node):
    """Node that calculates bid scores for fire tasks based on drone GPS position."""
    def __init__(self, drone_id):
        super().__init__('bidding_node_' + drone_id)
        self.drone_id = drone_id
        self.current_task = None
        self.current_gps = None  # Drone GPS: [lat, lon, alt]
        self.got_initial_gps = False

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to drone GPS
        self.create_subscription(
            NavSatFix,  
            f'/{drone_id}/mavros/global_position/global',
            self.gps_callback,
            qos
        )

        # Subscribe to fire tasks
        self.create_subscription(
            String,
            '/fire_tasks',
            self.fire_callback,
            qos
        )

        # Subscribe to assignments
        self.create_subscription(
            String,
            '/assignments',
            self.assignment_callback,
            qos
        )

        # Subscribe to task completion messages
        self.create_subscription(
            String,
            '/task_done',
            self.task_done_callback,
            qos
        )

        # Publisher for bids
        self.bid_pub = self.create_publisher(String, '/bids', 10)

        self.get_logger().info(f"🚁 {drone_id} bidding node started, waiting for GPS...")

    def gps_callback(self, msg):
        """Update current GPS position."""
        self.current_gps = [msg.latitude, msg.longitude, msg.altitude]

        # Log initial GPS only once
        if not self.got_initial_gps:
            self.get_logger().info(f"📡 Initial GPS: {self.current_gps}")
            self.got_initial_gps = True

    def haversine_distance(self, lat1, lon1, lat2, lon2):
        """Calculate distance (meters) between two GPS coordinates."""
        R = 6371000  # Earth radius in meters
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)

        a = math.sin(delta_phi/2.0)**2 + \
            math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda/2.0)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

        return R * c

    def distance_to_bid_score(self, distance):
        """Convert distance to bid score: closer fires = higher bid."""
        max_distance = 2000.0
        normalized = max(0.0, min(1.0, 1 - (distance / max_distance)))
        return int(normalized * 99) + 1

    def fire_callback(self, msg):
        """Send bid for the closest fire task if not already assigned."""
        if self.current_task is not None or self.current_gps is None:
            return

        fire = json.loads(msg.data)
        fire_lat, fire_lon = fire['location'][:2]

        dist = self.haversine_distance(self.current_gps[0], self.current_gps[1],
                                       fire_lat, fire_lon)
        bid_score = self.distance_to_bid_score(dist)

        bid = {
            'task_id': fire['task_id'],
            'drone_id': self.drone_id,
            'bid_score': bid_score,
            'location': fire['location']  # Keep GPS coords
        }

        self.bid_pub.publish(String(data=json.dumps(bid)))
        # Log GPS only when submitting a bid
        self.get_logger().info(f"📡 Submitting bid from GPS: {self.current_gps}")
        self.get_logger().info(f"Sent bid for task {fire['task_id']} at distance {dist:.1f} m: {bid}")

    def assignment_callback(self, msg):
        assignment = json.loads(msg.data)
        if assignment['drone_id'] == self.drone_id:
            self.current_task = assignment['task_id']
            self.get_logger().info(f"Assigned task {self.current_task}.")

    def task_done_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if data['drone_id'] == self.drone_id and self.current_task == data['task_id']:
                self.get_logger().info(f"✅ Task {data['task_id']} complete. Resetting current task.")
                self.current_task = None
        except Exception as e:
            self.get_logger().error(f"❗ Error processing task_done: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BiddingNode('drone_1')  # change drone_id for each drone
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
