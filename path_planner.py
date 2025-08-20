import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import NavSatFix
import json
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import sys
import math

landcover_priority_map = {
    "Built-up": 100,           # Urban areas → very high priority
    "Tree cover": 80,          # Forest → high priority
    "Shrubland": 60,           # Medium priority
    "Grassland": 40,           # Medium-low priority
    "Cropland": 30,            # Low priority
    "Bare / sparse vegetation": 20,
    "Herbaceous wetland": 10,
    "Mangroves": 10,
    "Permanent water bodies": 0,  # Ignore
    "Snow and ice": 0,            # Ignore
    "Moss and lichen": 0,         # Ignore
    "Unknown": 1                  # Fallback for unknown types
}

class MultiWaypointPlanner:
    """Plans multi-waypoint paths prioritizing downstream tasks based on wind direction."""
    def __init__(self, detour_threshold, nearby_radius, downstream_angle_range, priority_boost):
        self.detour_threshold = detour_threshold
        self.nearby_radius = nearby_radius
        self.downstream_angle_range = downstream_angle_range  # e.g., 45.0 for ±45°
        self.priority_boost = priority_boost  # e.g., 1.5 for priority multiplier

    def distance_lat_lon(self, pos1, pos2):
        """Calculate distance between two lat/lon points (simplified)."""
        lat1, lon1 = pos1[:2]
        lat2, lon2 = pos2[:2]
        return math.sqrt((lat1 - lat2)**2 + (lon1 - lon2)**2)

    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        """Calculate bearing from (lat1, lon1) to (lat2, lon2) in degrees."""
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        dlon = lon2 - lon1
        x = math.sin(dlon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        bearing = math.degrees(math.atan2(x, y))
        return (bearing + 360) % 360

    def is_downstream(self, bearing, downwind_direction):
        """Check if bearing is within ±downstream_angle_range of downwind direction."""
        if downwind_direction is None:
            return False
        # Normalize angles to [0, 360)
        bearing = bearing % 360
        downwind_direction = downwind_direction % 360
        min_angle = (downwind_direction - self.downstream_angle_range) % 360
        max_angle = (downwind_direction + self.downstream_angle_range) % 360
        # Handle angle wraparound
        if min_angle <= max_angle:
            return min_angle <= bearing <= max_angle
        return bearing >= min_angle or bearing <= max_angle

    def plan_multi_waypoint_path(self, start_pos, tasks, node=None):
        if not tasks:
            return []

        # Tasks are already prioritized in fire_data_callback, so just use nearest-neighbor
        # Sort by priority (lower negative = higher priority)
        sorted_tasks = sorted(tasks, key=lambda x: x[0])
        path = [start_pos]
        current_pos = start_pos
        unvisited = [task[2] for task in sorted_tasks]  # Extract coordinates

        # Nearest-neighbor path planning
        while unvisited:
            nearest_dist = float('inf')
            nearest_idx = 0
            for i, wp in enumerate(unvisited):
                dist = self.distance_lat_lon(current_pos, wp)
                if dist < nearest_dist:
                    nearest_dist = dist
                    nearest_idx = i
            next_waypoint = unvisited.pop(nearest_idx)
            path.append(next_waypoint)
            current_pos = next_waypoint

        return path

class FireDataPlanner(Node):
    def __init__(self, drone_id):
        super().__init__(f'fire_planner_{drone_id}')
        self.drone_id = drone_id

        # Declare ROS2 parameters
        self.declare_parameter('downstream_angle_range', 45.0)  # ±45° for downstream tasks
        self.declare_parameter('priority_boost', 1.5)  # Priority multiplier for downstream tasks
        self.declare_parameter('waypoint_tolerance', 0.00003)  # ~3.3 meters
        self.declare_parameter('replan_threshold', 5)  # Replan after 5 waypoints
        self.declare_parameter('detour_threshold', 25.0)  # Detour threshold for planner
        self.declare_parameter('nearby_radius', 35.0)  # Nearby radius for planner
        self.declare_parameter('wind_direction_threshold', 1.0)  # Minimum change to trigger replan

        # Get parameter values
        downstream_angle_range = self.get_parameter('downstream_angle_range').value
        priority_boost = self.get_parameter('priority_boost').value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        self.replan_threshold = self.get_parameter('replan_threshold').value
        detour_threshold = self.get_parameter('detour_threshold').value
        nearby_radius = self.get_parameter('nearby_radius').value
        self.wind_direction_threshold = self.get_parameter('wind_direction_threshold').value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Store wind direction and main fire location
        self.wind_direction = None
        self.main_fire = None  # (lat, lon)

        # Store current GPS
        self.current_lat = None
        self.current_lon = None
        self.got_initial_gps = False

        # Track visited fire data waypoints
        self.visited_waypoints = set()  # Store visited waypoints as (lat, lon, alt) tuples

        # Replanning throttle
        self.waypoints_since_replan = 0

        # Subscribe to GPS
        self.gps_sub = self.create_subscription(
            NavSatFix,
            f'/{drone_id}/mavros/global_position/global',
            self.gps_callback,
            qos
        )

        # Subscribe to fire data
        self.fire_data_sub = self.create_subscription(
            String, '/fire_data', self.fire_data_callback, qos
        )

        # Subscribe to wind direction
        self.wind_direction_sub = self.create_subscription(
            Float32, '/wind_direction', self.wind_direction_callback, qos
        )

        # Subscribe to fire tasks (main fire location)
        self.fire_tasks_sub = self.create_subscription(
            String, '/fire_tasks', self.fire_tasks_callback, qos
        )

        # Subscribe to visited waypoints
        self.visited_waypoints_sub = self.create_subscription(
            String, f'/visited_waypoints_{drone_id}', self.visited_waypoints_callback, qos
        )

        # Publish planned GPS waypoints
        self.path_pub = self.create_publisher(String, f'/fire_planner_path_{drone_id}', qos)

        self.all_tasks = []
        self.last_published_path = []
        self.path_published = False
        self.planner = MultiWaypointPlanner(detour_threshold, nearby_radius, downstream_angle_range, priority_boost)

        self.get_logger().info(f"🔥 Fire Data Planner initialized for {drone_id}")
        self.get_logger().info(f"🌪️ Subscribed to /wind_direction for downstream prioritization (angle range: ±{downstream_angle_range}°)")
        self.get_logger().info(f"🔥 Subscribed to /fire_tasks for main fire location")
        self.get_logger().info(f"📡 Subscribed to /visited_waypoints_{drone_id} for waypoint tracking")
        self.get_logger().info(f"⚡ Using optimized replanning (every {self.replan_threshold} waypoints)")
        self.get_logger().info(f"📍 Waypoint tolerance: {self.waypoint_tolerance} (~{self.waypoint_tolerance*111000:.1f} meters)")
        self.get_logger().info(f"🌪️ Wind direction change threshold: {self.wind_direction_threshold}°")
        self.get_logger().info("Waiting for GPS position to establish current location...")

    def fire_tasks_callback(self, msg: String):
        """Handle main fire location from /fire_tasks."""
        try:
            data = json.loads(msg.data)
            self.get_logger().info(f"🔥 Received /fire_tasks message: {msg.data}")
            task_id = data.get('task_id')
            location = data.get('location')
            if task_id and location:
                # Handle both dict and list formats for location
                if isinstance(location, dict) and 'lat' in location and 'lon' in location:
                    new_main_fire = (location['lat'], location['lon'])
                elif isinstance(location, list) and len(location) >= 2:
                    new_main_fire = (location[0], location[1])
                else:
                    self.get_logger().warning(f"Invalid location format in fire_tasks: {location}")
                    return

                # Check if main fire location has changed significantly
                if self.main_fire:
                    dist = self.planner.distance_lat_lon(self.main_fire, new_main_fire)
                    if dist < self.waypoint_tolerance:
                        self.get_logger().info(f"🔥 Main fire location unchanged: ({new_main_fire[0]:.6f}, {new_main_fire[1]:.6f})")
                        return

                self.main_fire = new_main_fire
                self.get_logger().info(f"🔥 Updated main fire location for {task_id}: ({self.main_fire[0]:.6f}, {self.main_fire[1]:.6f})")
                
                # Recompute priorities for existing tasks
                self.recompute_task_priorities()
                
                # Trigger replan if we have tasks, GPS, and wind direction
                if self.all_tasks and self.current_lat is not None and self.current_lon is not None and self.wind_direction is not None:
                    self.path_published = False
                    self.plan_and_publish_path(replan_from_fire_location=True)
                    self.get_logger().info("🚨 Forced replan due to new main fire location")
                else:
                    reasons = []
                    if not self.all_tasks:
                        reasons.append("no tasks")
                    if self.current_lat is None or self.current_lon is None:
                        reasons.append("no GPS")
                    if self.wind_direction is None:
                        reasons.append("no wind direction")
                    self.get_logger().info(f"⏳ Waiting for {', '.join(reasons)} before replanning...")
        except Exception as e:
            self.get_logger().error(f"❌ Error processing fire_tasks: {e}")

    def recompute_task_priorities(self):
        """Recompute priorities for all existing tasks based on current wind and fire location."""
        if not self.all_tasks or self.main_fire is None:
            return
            
        downwind_direction = (self.wind_direction + 180) % 360 if self.wind_direction is not None else None
        priority_boost = self.get_parameter('priority_boost').value
        
        updated_tasks = []
        for priority, sub_task_id, coords in self.all_tasks:
            # Extract base priority info from the task ID or recalculate from landcover
            # For simplicity, we'll assume the base priority was stored correctly
            base_priority = priority / priority_boost if priority < 0 else priority
            
            # Apply downstream boost if applicable
            if self.main_fire and self.wind_direction is not None:
                bearing = self.planner.calculate_bearing(self.main_fire[0], self.main_fire[1], coords[0], coords[1])
                if self.planner.is_downstream(bearing, downwind_direction):
                    new_priority = base_priority * priority_boost
                    self.get_logger().info(f"🔄 Reprioritized downstream task {sub_task_id}: bearing {bearing:.1f}°, priority {base_priority:.1f} → {new_priority:.1f}")
                else:
                    new_priority = base_priority
            else:
                new_priority = base_priority
                
            updated_tasks.append((new_priority, sub_task_id, coords))
        
        self.all_tasks = updated_tasks
        self.all_tasks.sort(key=lambda x: x[0])  # Re-sort by priority

    def wind_direction_callback(self, msg: Float32):
        """Handle wind direction updates."""
        new_wind_direction = msg.data % 360  # Normalize to [0, 360)
        if self.wind_direction is not None and abs(new_wind_direction - self.wind_direction) < self.wind_direction_threshold:
            self.get_logger().info(f"🌪️ Wind direction unchanged: {new_wind_direction:.1f}°")
            return
        old_direction = self.wind_direction
        self.wind_direction = new_wind_direction
        downwind_direction = (self.wind_direction + 180) % 360
        downstream_angle_range = self.get_parameter('downstream_angle_range').value
        self.get_logger().info(f"🌪️ Received wind direction: {self.wind_direction:.1f}° (downwind: {downwind_direction:.1f}°, range: [{(downwind_direction - downstream_angle_range) % 360:.1f}°, {(downwind_direction + downstream_angle_range) % 360:.1f}°])")
        
        # Recompute priorities for existing tasks
        self.recompute_task_priorities()
        
        # Trigger replan if we have tasks, GPS, and main fire
        if self.all_tasks and self.current_lat is not None and self.current_lon is not None and self.main_fire is not None:
            self.path_published = False
            self.plan_and_publish_path(replan_from_wind=True)
            self.get_logger().info("🚨 Forced replan due to wind direction update")
        else:
            reasons = []
            if not self.all_tasks:
                reasons.append("no tasks")
            if self.current_lat is None or self.current_lon is None:
                reasons.append("no GPS")
            if self.main_fire is None:
                reasons.append("no main fire location")
            self.get_logger().info(f"⏳ Waiting for {', '.join(reasons)} before replanning...")

    def gps_callback(self, msg: NavSatFix):
        """Update current GPS position."""
        if msg.status.status >= 0:
            self.current_lat = msg.latitude
            self.current_lon = msg.longitude
            if not self.got_initial_gps:
                self.get_logger().info(f"📡 Initial GPS: {self.current_lat:.8f}, {self.current_lon:.8f}")
                self.got_initial_gps = True
            # Only plan if we have tasks, main fire, and wind direction
            if self.all_tasks and self.main_fire is not None and self.wind_direction is not None and not self.path_published:
                self.plan_and_publish_path()
            else:
                reasons = []
                if not self.all_tasks:
                    reasons.append("no tasks")
                if self.main_fire is None:
                    reasons.append("no main fire location")
                if self.wind_direction is None:
                    reasons.append("no wind direction")
                if self.path_published:
                    reasons.append("path already published")
                self.get_logger().info(f"⏳ Waiting for {', '.join(reasons)} before planning...")

    def is_waypoint_visited(self, waypoint):
        """Check if a waypoint has been visited within tolerance."""
        lat, lon, alt = waypoint
        for visited_lat, visited_lon, visited_alt in self.visited_waypoints:
            distance = math.sqrt((lat - visited_lat)**2 + (lon - visited_lon)**2)
            if distance < self.waypoint_tolerance:
                self.get_logger().info(f"📍 Waypoint ({lat:.6f}, {lon:.6f}, {alt}) matches visited ({visited_lat:.6f}, {visited_lon:.6f}, {visited_alt}) within tolerance {self.waypoint_tolerance} (~{self.waypoint_tolerance*111000:.1f} meters)")
                return True
        return False

    def visited_waypoints_callback(self, msg: String):
        """Handle visited waypoint notifications from drone controller."""
        try:
            data = json.loads(msg.data)
            if data.get('drone_id') != self.drone_id:
                return
            waypoint = tuple(data['waypoint'])  # Convert to tuple for set storage
            if waypoint in self.visited_waypoints:
                self.get_logger().info(f"⏭️ Skipping duplicate visited waypoint: ({waypoint[0]:.6f}, {waypoint[1]:.6f}, {waypoint[2]})")
                return
            self.visited_waypoints.add(waypoint)
            lat, lon, alt = waypoint
            self.get_logger().info(f"🎯 Marked fire waypoint as visited: ({lat:.6f}, {lon:.6f}, {alt})")
            self.get_logger().info(f"📊 Total visited fire waypoints: {len(self.visited_waypoints)}")

            # Remove visited tasks
            self.remove_visited_tasks()
            self.waypoints_since_replan += 1
            should_replan = (
                self.waypoints_since_replan >= self.replan_threshold or
                len(self.all_tasks) <= 10
            )
            if self.all_tasks and should_replan:
                self.waypoints_since_replan = 0
                self.plan_and_publish_path(replan_from_visited=True)
                self.get_logger().info(f"🔄 Triggered replan (threshold reached or few tasks remaining)")
            elif not self.all_tasks:
                self.get_logger().info("🎉 All fire data tasks completed!")
            else:
                self.get_logger().info(f"⏳ Deferring replan ({self.waypoints_since_replan}/{self.replan_threshold} waypoints since last replan)")
        except Exception as e:
            self.get_logger().error(f"❌ Error processing visited waypoint: {e}")

    def remove_visited_tasks(self):
        """Remove tasks that correspond to visited waypoints."""
        initial_count = len(self.all_tasks)
        self.all_tasks = [
            task for task in self.all_tasks
            if not self.is_waypoint_visited(task[2])
        ]
        removed_count = initial_count - len(self.all_tasks)
        if removed_count > 0:
            self.get_logger().info(f"🗑️ Removed {removed_count} visited tasks. Remaining: {len(self.all_tasks)}")

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

            # Build tasks from grid points with downstream prioritization
            existing_coords = {task[2] for task in self.all_tasks}
            new_tasks = []
            downwind_direction = (self.wind_direction + 180) % 360 if self.wind_direction is not None else None
            priority_boost = self.get_parameter('priority_boost').value  # e.g., 1.5
            
            for i, pt in enumerate(grid):
                coords = (pt['lat'], pt['lon'], 50.0)
                if self.is_waypoint_visited(coords):
                    self.get_logger().info(f"⏭️ Skipping already visited fire waypoint: ({coords[0]:.6f}, {coords[1]:.6f})")
                    continue
                if coords in existing_coords:
                    self.get_logger().info(f"⏭️ Skipping duplicate fire waypoint: ({coords[0]:.6f}, {coords[1]:.6f})")
                    continue
                
                # Base priority from landcover
                class_weight = landcover_priority_map.get(pt['class_name'], 1)
                pct_coverage = pt.get('class_pct_coverage', 0.0)
                priority = - (pct_coverage * class_weight)
                
                # Boost priority if downstream
                if self.main_fire and self.wind_direction is not None:
                    bearing = self.planner.calculate_bearing(self.main_fire[0], self.main_fire[1], coords[0], coords[1])
                    if self.planner.is_downstream(bearing, downwind_direction):
                        priority = priority * priority_boost  # Makes it more negative = higher priority
                        self.get_logger().info(f"📍 Downstream task {task_id}_pt_{i:02d}: ({coords[0]:.6f}, {coords[1]:.6f}), bearing {bearing:.1f}°, boosted priority {priority:.1f}")
                    else:
                        self.get_logger().info(f"📍 Task {task_id}_pt_{i:02d}: ({coords[0]:.6f}, {coords[1]:.6f}), bearing {bearing:.1f}° from main fire")
                else:
                    if self.main_fire:
                        bearing = self.planner.calculate_bearing(self.main_fire[0], self.main_fire[1], coords[0], coords[1])
                        self.get_logger().info(f"📍 Task {task_id}_pt_{i:02d}: ({coords[0]:.6f}, {coords[1]:.6f}), bearing {bearing:.1f}° from main fire")
                
                sub_task_id = f"{task_id}_pt_{i:02d}"
                new_tasks.append((priority, sub_task_id, coords))
                existing_coords.add(coords)

            if not new_tasks:
                self.get_logger().info("ℹ️ No new unvisited or unique tasks to add")
                return

            self.all_tasks.extend(new_tasks)
            self.remove_visited_tasks()
            self.all_tasks.sort(key=lambda x: x[0])  # Sort by priority
            self.get_logger().info(f"🔥 Received fire data with {len(new_tasks)} new unvisited tasks, total tasks: {len(self.all_tasks)}")

            # Only plan if we have GPS, main fire, and wind direction
            if self.current_lat is not None and self.current_lon is not None and self.main_fire is not None and self.wind_direction is not None:
                self.path_published = False
                self.plan_and_publish_path()
                self.get_logger().info("🚨 Forced replan due to new fire data")
            else:
                reasons = []
                if self.current_lat is None or self.current_lon is None:
                    reasons.append("no GPS")
                if self.main_fire is None:
                    reasons.append("no main fire location")
                if self.wind_direction is None:
                    reasons.append("no wind direction")
                self.get_logger().info(f"⏳ Waiting for {', '.join(reasons)} before planning...")

        except Exception as e:
            self.get_logger().error(f"❌ Error processing fire_data: {e}")

    def plan_and_publish_path(self, append=False, replan_from_visited=False, replan_from_wind=False, replan_from_fire_location=False):
        if self.current_lat is None or self.current_lon is None or not self.all_tasks or self.main_fire is None or self.wind_direction is None:
            reasons = []
            if self.current_lat is None or self.current_lon is None:
                reasons.append("no GPS")
            if not self.all_tasks:
                reasons.append("no tasks")
            if self.main_fire is None:
                reasons.append("no main fire location")
            if self.wind_direction is None:
                reasons.append("no wind direction")
            self.get_logger().info(f"⏳ Cannot plan path: {', '.join(reasons)}")
            return

        self.remove_visited_tasks()
        start_pos = (self.current_lat, self.current_lon, 50.0)
        gps_path = self.planner.plan_multi_waypoint_path(start_pos, self.all_tasks, self)

        if gps_path:
            self.publish_path(gps_path)
            self.last_published_path = gps_path
            self.path_published = True
            if replan_from_wind:
                self.get_logger().info(f"🌪️ Replanned path with {len(gps_path)} waypoints (wind direction update)")
            elif replan_from_visited:
                self.get_logger().info(f"🔄 Replanned path with {len(gps_path)} waypoints (after visiting waypoint)")
            elif replan_from_fire_location:
                self.get_logger().info(f"🔥 Replanned path with {len(gps_path)} waypoints (new main fire location)")
            else:
                self.get_logger().info(f"📍 Planned GPS path with {len(gps_path)} waypoints")

    def publish_path(self, gps_waypoints):
        """Publish path as JSON list of [lat, lon, alt]."""
        json_data = json.dumps(gps_waypoints)
        self.path_pub.publish(String(data=json_data))
        self.get_logger().info(f"📡 GPS waypoints published to /fire_planner_path_{self.drone_id}")

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 2:
        print("Usage: python fire_planner.py <drone_id>")
        sys.exit(1)
    drone_id = sys.argv[1]
    node = FireDataPlanner(drone_id)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🔥 Fire Data Planner shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()