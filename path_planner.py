import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
import json
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class MultiWaypointPlanner:
    """Plans multi-waypoint paths using lat/lon coordinates."""
    def __init__(self, detour_threshold=25.0, nearby_radius=35.0):
        self.detour_threshold = detour_threshold
        self.nearby_radius = nearby_radius

    def plan_multi_waypoint_path(self, start_pos, tasks, logger=None):
        if not tasks:
            return []

        path = [start_pos]  # start with current lat/lon/alt
        current_pos = start_pos
        unvisited = tasks.copy()  # list of (priority, task_id, (lat, lon, alt))

        while unvisited:
            # pick highest priority task
            unvisited.sort(key=lambda x: x[0])
            next_idx = 0
            next_task = unvisited.pop(next_idx)
            next_coords = next_task[2]
            path.append(next_coords)
            current_pos = next_coords

        return path

class FireDataPlanner(Node):
    def __init__(self, drone_id):
        super().__init__(f'fire_planner_drone_{drone_id}')
        self.drone_id = drone_id

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Store current GPS
        self.current_lat = None
        self.current_lon = None
        self.got_initial_gps = False

        # Subscribe to GPS
        self.gps_sub = self.create_subscription(
            NavSatFix,
            f'/drone_{drone_id}/mavros/global_position/global',
            self.gps_callback,
            qos
        )

        # Subscribe to fire data
        self.fire_data_sub = self.create_subscription(
            String, '/fire_data', self.fire_data_callback, qos
        )

        # Publish planned GPS waypoints
        self.path_pub = self.create_publisher(String, f'/fire_planner_path_drone_{drone_id}', qos)

        self.all_tasks = []
        self.path_published = False
        self.planner = MultiWaypointPlanner()

        self.get_logger().info(f"🔥 Fire Data Planner initialized for drone_{drone_id}")
        self.get_logger().info("Waiting for GPS position to establish current location...")

    def gps_callback(self, msg: NavSatFix):
        """Update current GPS position"""
        if msg.status.status >= 0:
            self.current_lat = msg.latitude
            self.current_lon = msg.longitude

            if not self.got_initial_gps:
                self.get_logger().info(f"📡 Initial GPS: {self.current_lat:.8f}, {self.current_lon:.8f}")
                self.got_initial_gps = True

            if self.all_tasks and not self.path_published:
                self.plan_and_publish_path()

    def fire_data_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            task_id = data.get('task_id')
            lat = data.get('lat')
            lon = data.get('lon')
            grid = data.get('landcover_grid_50m', [])

            if lat is None or lon is None:
                self.get_logger().warning(f"Incomplete fire_data: {data}")
                return

            # Build tasks from grid points
            new_tasks = []
            for i, pt in enumerate(grid):
                coords = (pt['lat'], pt['lon'], 50.0)
                priority = -pt.get('class_pct_coverage', 0.0)
                sub_task_id = f"{task_id}_pt_{i:02d}"
                new_tasks.append((priority, sub_task_id, coords))

            self.all_tasks.extend(new_tasks)
            self.all_tasks.sort(key=lambda x: x[0])
            self.path_published = False

            self.get_logger().info(f"🔥 Received fire data with {len(new_tasks)} new tasks")

            if self.current_lat is not None and self.current_lon is not None:
                self.plan_and_publish_path()
            else:
                self.get_logger().info("⏳ Waiting for current GPS position before planning...")

        except Exception as e:
            self.get_logger().error(f"❌ Error processing fire_data: {e}")

    def plan_and_publish_path(self):
        if self.current_lat is None or self.current_lon is None or not self.all_tasks:
            return

        start_pos = (self.current_lat, self.current_lon, 50.0)
        self.get_logger().info(f"🗺️ Planning path from GPS: ({start_pos[0]:.6f}, {start_pos[1]:.6f})")
        
        gps_path = self.planner.plan_multi_waypoint_path(start_pos, self.all_tasks, self.get_logger())

        if gps_path:
            self.get_logger().info(f"📍 Planned GPS path with {len(gps_path)} waypoints")
            self.publish_path(gps_path)
            self.path_published = True

    def publish_path(self, gps_waypoints):
        """Publish path as JSON list of [lat, lon, alt]"""
        json_data = json.dumps(gps_waypoints)
        self.path_pub.publish(String(data=json_data))
        self.get_logger().info(f"📡 GPS waypoints published to /fire_planner_path_{self.drone_id}")

def main(args=None):
    rclpy.init(args=args)
    import sys
    if len(sys.argv) < 2:
        print("Usage: python fire_planner.py <drone_id>")
        sys.exit(1)
    drone_id = sys.argv[1]
    node = FireDataPlanner(drone_id)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
