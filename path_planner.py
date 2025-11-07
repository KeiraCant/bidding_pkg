import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import NavSatFix
import json
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import sys
import math
import os
import csv
from datetime import datetime
from mavros_msgs.srv import WaypointPull
from mavros_msgs.msg import WaypointReached, WaypointList
import numpy as np
import threading
import queue

landcover_priority_map = {
    "Built-up": 100,
    "Tree cover": 80,
    "Shrubland": 60,
    "Grassland": 40,
    "Cropland": 30,
    "Bare / sparse vegetation": 20,
    "Herbaceous wetland": 10,
    "Mangroves": 10,
    "Permanent water bodies": 0,
    "Snow and ice": 0,
    "Moss and lichen": 0,
    "Unknown": 1
}

class MultiWaypointPlanner:
    def __init__(self, detour_threshold, nearby_radius, downstream_angle_range, priority_boost):
        self.detour_threshold = detour_threshold
        self.nearby_radius = nearby_radius
        self.downstream_angle_range = downstream_angle_range
        self.priority_boost = priority_boost
        self.algorithm = "multi_waypoint"

    def set_algorithm(self, algorithm):
        """Set the path planning algorithm to use"""
        valid_algorithms = ["multi_waypoint", "sector_based", "dynamic_clustering", "priority_sweep"]
        if algorithm in valid_algorithms:
            self.algorithm = algorithm
            return True
        return False

    def distance_lat_lon(self, pos1, pos2):
        lat1, lon1 = pos1[:2]
        lat2, lon2 = pos2[:2]
        return math.sqrt((lat1 - lat2)**2 + (lon1 - lon2)**2)

    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        dlon = lon2 - lon1
        x = math.sin(dlon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        bearing = math.degrees(math.atan2(x, y))
        return (bearing + 360) % 360

    def is_downstream(self, bearing, downwind_direction):
        if downwind_direction is None:
            return False
        bearing = bearing % 360
        downwind_direction = downwind_direction % 360
        min_angle = (downwind_direction - self.downstream_angle_range) % 360
        max_angle = (downwind_direction + self.downstream_angle_range) % 360
        if min_angle <= max_angle:
            return min_angle <= bearing <= max_angle
        return bearing >= min_angle or bearing <= max_angle
    
    def plan_path(self, start_pos, tasks, node=None):
        """Route to the selected algorithm"""
        if self.algorithm == "multi_waypoint":
            return self.plan_multi_waypoint_path(start_pos, tasks, node)
        elif self.algorithm == "sector_based":
            return self.plan_sector_based_path(start_pos, tasks, node)
        elif self.algorithm == "dynamic_clustering":
            return self.plan_dynamic_clustering_path(start_pos, tasks, node)
        elif self.algorithm == "priority_sweep":
            return self.plan_priority_sweep_path(start_pos, tasks, node)
        else:
            return self.plan_multi_waypoint_path(start_pos, tasks, node)
        
    def plan_sector_based_path(self, start_pos, tasks, node=None):
        """Divide area into sectors and process by priority within each sector"""
        if not tasks:
            return [], []
        
        path = [start_pos]
        current_pos = start_pos
        task_order = []
        
        lats = [t[2][0] for t in tasks]
        lons = [t[2][1] for t in tasks]
        min_lat, max_lat = min(lats), max(lats)
        min_lon, max_lon = min(lons), max(lons)
        
        mid_lat = (min_lat + max_lat) / 2
        mid_lon = (min_lon + max_lon) / 2
        
        sectors = {
            'NE': [t for t in tasks if t[2][0] >= mid_lat and t[2][1] >= mid_lon],
            'NW': [t for t in tasks if t[2][0] >= mid_lat and t[2][1] < mid_lon],
            'SE': [t for t in tasks if t[2][0] < mid_lat and t[2][1] >= mid_lon],
            'SW': [t for t in tasks if t[2][0] < mid_lat and t[2][1] < mid_lon]
        }
        
        while any(sectors.values()):
            closest_sector = None
            min_sector_dist = float('inf')
            
            for sector_name, sector_tasks in sectors.items():
                if not sector_tasks:
                    continue
                
                sector_center = [
                    sum(t[2][0] for t in sector_tasks) / len(sector_tasks),
                    sum(t[2][1] for t in sector_tasks) / len(sector_tasks)
                ]
                sector_dist = self.distance_lat_lon(current_pos, sector_center)
                
                if sector_dist < min_sector_dist:
                    min_sector_dist = sector_dist
                    closest_sector = sector_name
            
            if closest_sector is None:
                break
                
            sector_tasks = sectors[closest_sector]
            highest_priority_task = max(sector_tasks, key=lambda t: abs(t[0]))
            sectors[closest_sector].remove(highest_priority_task)
            
            path.append(highest_priority_task[2])
            task_order.append(highest_priority_task)
            current_pos = highest_priority_task[2]
            
            #if node:
             #   node.get_logger().info(
              #      f"Selected task from {closest_sector} sector: {highest_priority_task[1]}, "
               #     f"priority {highest_priority_task[0]:.2f}"
                #)
    
        return path, task_order
  
    def plan_dynamic_clustering_path(self, start_pos, tasks, node=None):
        """Dynamic clustering with priority-weighted centroids"""
        if not tasks:
            return [], []
        
        path = [start_pos]
        current_pos = start_pos
        task_order = []
        remaining = tasks[:]
        
        cluster_size = max(3, min(8, len(tasks) // 4))
        
        while remaining:
            if len(remaining) <= cluster_size:
                remaining.sort(key=lambda t: abs(t[0]), reverse=True)
                for task in remaining:
                    path.append(task[2])
                    task_order.append(task)
                    current_pos = task[2]
                remaining.clear()
                continue
            
            distances = [(i, self.distance_lat_lon(current_pos, t[2])) for i, t in enumerate(remaining)]
            distances.sort(key=lambda x: x[1])
            
            cluster_candidates = distances[:cluster_size * 2]
            cluster = []
            
            for idx, dist in cluster_candidates:
                if len(cluster) >= cluster_size:
                    break
                task = remaining[idx]
                if not cluster or abs(task[0]) > np.percentile([abs(t[0]) for t in remaining], 60):
                    cluster.append((idx, task))
            
            while len(cluster) < min(cluster_size, len(remaining)):
                for idx, dist in distances:
                    if idx not in [c[0] for c in cluster]:
                        cluster.append((idx, remaining[idx]))
                        break
            
            cluster.sort(key=lambda x: abs(x[1][0]), reverse=True)
            
            for idx, task in cluster:
                if task in remaining:
                    remaining.remove(task)
                    path.append(task[2])
                    task_order.append(task)
                    current_pos = task[2]
                    
                   # if node:
                    #    node.get_logger().info(
                     #       f"Cluster task: {task[1]}, priority {task[0]:.2f}, "
                      #      f"distance {self.distance_lat_lon(path[-2], task[2])*111000:.2f}m"
                       # )
        
        return path, task_order

    def plan_priority_sweep_path(self, start_pos, tasks, node=None):
        """Priority-first sweep with spatial optimization within tiers"""
        if not tasks:
            return [], []

        path = [start_pos]
        current_pos = start_pos
        task_order = []
        
        priorities = [abs(task[0]) for task in tasks]
        if not priorities:
            return path, task_order
            
        priority_threshold_high = np.percentile(priorities, 75)
        priority_threshold_medium = np.percentile(priorities, 40)
        
        high_priority = [t for t in tasks if abs(t[0]) >= priority_threshold_high]
        medium_priority = [t for t in tasks if priority_threshold_medium <= abs(t[0]) < priority_threshold_high]
        low_priority = [t for t in tasks if abs(t[0]) < priority_threshold_medium]
        
        for tier_name, tier in [("HIGH", high_priority), ("MEDIUM", medium_priority), ("LOW", low_priority)]:
            if not tier:
                continue
                
            remaining = tier[:]
            if node:
                node.get_logger().info(f"Processing {tier_name} priority tier: {len(remaining)} tasks")
                
            while remaining:
                nearest_task = min(remaining, key=lambda t: self.distance_lat_lon(current_pos, t[2]))
                remaining.remove(nearest_task)
                path.append(nearest_task[2])
                task_order.append(nearest_task)
                current_pos = nearest_task[2]
                
                #if node:
                 #   node.get_logger().info(f"  Selected {tier_name} task {nearest_task[1]}: priority {nearest_task[0]:.2f}")
        
        return path, task_order

    def plan_multi_waypoint_path(self, start_pos, tasks, node=None):
        if not tasks:
            return [], []

        path = [start_pos]
        current_pos = start_pos
        task_order = []

        unvisited = tasks[:]
        while unvisited:
            best_score = float('inf')
            best_idx = 0
            best_dist = float('inf')
            max_priority = max(abs(task[0]) for task in unvisited) or 1.0

            for i, task_data in enumerate(unvisited):
                priority = task_data[0]
                wp = task_data[2]
                dist = self.distance_lat_lon(current_pos, wp)
                normalized_priority = abs(priority) / max_priority
                normalized_dist = dist / 0.0005
                score = (1.0 - normalized_priority) * 0.7 + normalized_dist * 0.3

                if score < best_score:
                    best_score = score
                    best_idx = i
                    best_dist = dist

            highest_priority_task = unvisited[0]
            highest_priority_dist = self.distance_lat_lon(current_pos, highest_priority_task[2])
            if highest_priority_task != unvisited[best_idx] and highest_priority_dist <= self.detour_threshold:
                next_task = unvisited.pop(0)
            else:
                next_task = unvisited.pop(best_idx)

            next_waypoint = next_task[2]
            path.append(next_waypoint)
            current_pos = next_waypoint
            task_order.append(next_task)

            #if node:
             #   node.get_logger().info(
               #     f"Selected task {next_task[1]}: priority {next_task[0]:.2f}, "
              #      f"distance {best_dist*111000:.2f}m, coords ({next_waypoint[0]:.6f}, {next_waypoint[1]:.6f})"
                #)

        return path, task_order

def select_algorithm():
    """Display menu and get user's initial algorithm choice"""
    print("\n" + "="*70)
    print("  FIRE RESPONSE PATH PLANNING - ALGORITHM SELECTION")
    print("="*70)
    print("\nNote: You will be prompted to change the algorithm if downstream tasks are detected.")
    print("\nAvailable path planning algorithms:\n")
    
    print("1. MULTI-WAYPOINT PLANNING (Default)")
    print("   Balanced scoring approach that weighs both task priority (70%)")
    print("   and travel distance (30%). Best general-purpose algorithm.\n")
    
    print("2. SECTOR-BASED PLANNING")
    print("   Divides the area into quadrants and processes the highest priority task.")
    print("   Good for large dispersed areas.\n")
    
    print("3. DYNAMIC CLUSTERING")
    print("   Groups nearby tasks into clusters and processes by priority.")
    print("   Efficient for dense task regions.\n")
    
    print("4. PRIORITY SWEEP")
    print("   Processes tasks by priority tiers (HIGH/MEDIUM/LOW).")
    print("   Recommended for critical downstream tasks.\n")
    
    print("="*70)
    
    while True:
        try:
            choice = input("\nSelect initial algorithm (1-4, or press Enter for default): ").strip()
            
            if choice == "" or choice == "1":
                return "multi_waypoint"
            elif choice == "2":
                return "sector_based"
            elif choice == "3":
                return "dynamic_clustering"
            elif choice == "4":
                return "priority_sweep"
            else:
                print("Invalid choice. Please enter 1-4 or press Enter for default.")
        except KeyboardInterrupt:
            print("\n\nExiting...")
            sys.exit(0)

class FireDataPlanner(Node):
    def __init__(self, drone_id, algorithm='multi_waypoint'):
        super().__init__(f'fire_planner_{drone_id}')
        self.drone_id = drone_id

        # Parameters
        self.declare_parameter('downstream_angle_range', 45.0)
        self.declare_parameter('priority_boost', 1.5)
        self.declare_parameter('waypoint_tolerance', 0.00003)
        self.declare_parameter('detour_threshold', 25.0)
        self.declare_parameter('nearby_radius', 35.0)
        self.declare_parameter('wind_direction_threshold', 1.0)
        self.declare_parameter('replan_throttle_seconds', 120.0)

        downstream_angle_range = self.get_parameter('downstream_angle_range').value
        priority_boost = self.get_parameter('priority_boost').value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        detour_threshold = self.get_parameter('detour_threshold').value
        nearby_radius = self.get_parameter('nearby_radius').value
        self.wind_direction_threshold = self.get_parameter('wind_direction_threshold').value
        self.replan_throttle_seconds = self.get_parameter('replan_throttle_seconds').value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # State
        self.wind_direction = None
        self.main_fire_locations = {}
        self.current_lat = None
        self.current_lon = None
        self.got_initial_gps = False
        self.visited_waypoints = set()
        self.plan_counter = 0
        self.tasks_by_fire = {}
        self.active_fire_queue = []
        self.current_fire_index = 0
        self.last_replan_time = 0.0
        self.precip_by_fire = {}
        self.prompted_for_downstream = False
        self.downstream_task_counts = {}

        # CSV files
        self.downstream_csv_path = os.path.expanduser(f'~/downstream_fire_tasks_{drone_id}.csv')
        with open(self.downstream_csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['drone_id', 'task_id', 'sub_task_id', 'lat', 'lon', 'alt', 'bearing_from_main_fire', 'wind_direction', 'downwind_direction', 'base_priority', 'boosted_priority', 'landcover_class', 'class_coverage_pct'])

        self.task_order_csv_path = os.path.expanduser(f'~/task_execution_order_{drone_id}.csv')
        with open(self.task_order_csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'plan_number', 'execution_order', 'drone_id', 'task_id', 'sub_task_id', 'lat', 'lon', 'alt', 'priority', 'distance_from_previous', 'cumulative_distance', 'replan_reason'])
        
        self.simple_task_csv_path = os.path.expanduser(f'~/task_coords_priority_{drone_id}.csv')
        with open(self.simple_task_csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['lat', 'lon', 'alt', 'priority'])

        # Subscriptions
        self.gps_sub = self.create_subscription(NavSatFix, f'/{drone_id}/mavros/global_position/global', self.gps_callback, qos)
        self.fire_data_sub = self.create_subscription(String, '/fire_data', self.fire_data_callback, qos)
        self.wind_direction_sub = self.create_subscription(Float32, '/wind_direction', self.wind_direction_callback, qos)
        self.fire_tasks_sub = self.create_subscription(String, '/fire_tasks', self.fire_tasks_callback, qos)
        self.visited_waypoints_sub = self.create_subscription(String, f'/visited_waypoints_{drone_id}', self.visited_waypoints_callback, qos)
        self.path_pub = self.create_publisher(String, f'/fire_planner_path_{drone_id}', qos)
        # Logging to UI
        self.log_pub = self.create_publisher(String, '/fire_planner_log', qos)

        self.last_published_path = []
        self.path_published = False
        self.planner = MultiWaypointPlanner(detour_threshold, nearby_radius, downstream_angle_range, priority_boost)
        self.planner.set_algorithm(algorithm)
        self.publish_log(f"Using algorithm: {algorithm.upper().replace('_', ' ')}")
        self.get_logger().info(f"Multi-Fire Data Planner initialized for {drone_id}")

        self.fire_precip_sub = self.create_subscription(String, '/fire_precip', self.fire_precip_callback, qos)
        
        # MAVROS mission tracking
        self.total_wp = None
        self.pull_client = self.create_client(WaypointPull, f'/{drone_id}/mavros/mission/pull')
        self.sub_reached = self.create_subscription(
            WaypointReached,
            f'/{drone_id}/mavros/mission/reached',
            self.reached_callback,
            qos
        )
        self.timer = self.create_timer(2.0, self.update_mission_info)
        self.mavros_total_waypoints = 0
        self.mavros_last_reached = -1

        # Input queue for non-blocking user input
        self.input_queue = queue.Queue()
        self.input_thread = None
        
    def publish_log(self, text):
        """Publish log message to UI and console, tagged with drone ID"""
        tagged_text = f"[{self.drone_id}] {text}"
        msg = String()
        msg.data = tagged_text
        self.log_pub.publish(msg)
        self.get_logger().info(tagged_text)

    
    def count_downstream_tasks(self):
        """Count downstream tasks for each fire"""
        self.downstream_task_counts.clear()
        downwind_direction = (self.wind_direction + 180) % 360 if self.wind_direction is not None else None
        
        if not downwind_direction:
            return False

        has_downstream = False
        for task_id, tasks in self.tasks_by_fire.items():
            main_fire = self.main_fire_locations.get(task_id)
            if not main_fire:
                continue
                
            downstream_count = 0
            for task_data in tasks:
                coords = task_data[2]
                bearing = self.planner.calculate_bearing(main_fire[0], main_fire[1], coords[0], coords[1])
                if self.planner.is_downstream(bearing, downwind_direction):
                    downstream_count += 1
            
            self.downstream_task_counts[task_id] = downstream_count
            if downstream_count > 0:
                has_downstream = True
        
        return has_downstream

    def prompt_algorithm_change(self):
        """Prompt user to change algorithm based on downstream tasks"""
        if self.prompted_for_downstream or not self.count_downstream_tasks():
            return

        self.prompted_for_downstream = True
        total_fires = len(self.tasks_by_fire)
        total_downstream = sum(self.downstream_task_counts.values())
        self.publish_log(f"[DOWNSTREAM] Total downstream tasks: {total_downstream}")
        print("\n" + "="*70)
        print("  DOWNSTREAM FIRE TASKS DETECTED")
        print("="*70)
        print(f"\nCurrent Fire Status:")
        print(f"  Total active fires: {total_fires}")
        print(f"  Total downstream tasks: {total_downstream}")
        for task_id, count in self.downstream_task_counts.items():
            print(f"    Fire {task_id}: {count} downstream tasks")
        print(f"  Wind direction: {self.wind_direction:.1f}°")
        print(f"\nCurrent algorithm: {self.planner.algorithm.upper().replace('_', ' ')}")
        print("\nWould you like to change the path planning algorithm?")
        print("Available algorithms:\n")
        
        print("1. MULTI-WAYPOINT PLANNING")
        print("   Balanced scoring (70% priority, 30% distance). Best general-purpose algorithm.")
        print("   Recommended for scenarios with many downstream tasks.\n")
        
        print("2. SECTOR-BASED PLANNING")
        print("   Processes highest priority task in nearest quadrant. Good for dispersed areas.\n")
        
        print("3. DYNAMIC CLUSTERING")
        print("   Groups tasks into clusters and processes by priority. Efficient for dense regions.\n")
        
        print("4. PRIORITY SWEEP")
        print("   Processes tasks by priority tiers (HIGH/MEDIUM/LOW). Recommended for critical downstream tasks.\n")
        
        print("="*70)
        
        def get_user_input():
            try:
                choice = input("\nSelect algorithm (1-4, or press Enter to keep current): ").strip()
                self.input_queue.put(choice)
            except KeyboardInterrupt:
                self.input_queue.put("exit")
        
        self.input_thread = threading.Thread(target=get_user_input)
        self.input_thread.daemon = True
        self.input_thread.start()

    def process_algorithm_change(self):
        """Process user input for algorithm change from input queue"""
        try:
            choice = self.input_queue.get_nowait()
            if choice == "exit":
                self.get_logger().info("User requested shutdown")
                rclpy.shutdown()
                return

            algorithm_map = {
                "1": "multi_waypoint",
                "2": "sector_based",
                "3": "dynamic_clustering",
                "4": "priority_sweep",
                "": self.planner.algorithm
            }
            
            new_algorithm = algorithm_map.get(choice, self.planner.algorithm)
            if new_algorithm != self.planner.algorithm:
                self.planner.set_algorithm(new_algorithm)
                self.get_logger().info(f"Algorithm changed to: {new_algorithm.upper().replace('_', ' ')}")
                self.last_replan_time = self.get_clock().now().nanoseconds / 1e9
                self.plan_and_publish_path(replan_reason="algorithm_change")
        
        except queue.Empty:
            pass

    def create_progress_bar(self, current, total, bar_length=30):
        """Create a text-based progress bar"""
        if total == 0:
            return "[" + " " * bar_length + "] 0/0 (0%)"
        
        percent = current / total
        filled_length = int(bar_length * percent)
        bar = "=" * filled_length + ">" + " " * (bar_length - filled_length - 1)
        
        if filled_length == bar_length:
            bar = "=" * bar_length
        
        return f"[{bar}] {current}/{total} ({percent*100:.0f}%)"

    def update_mission_info(self):
        """Check mission status and process algorithm change input"""
        if self.pull_client.service_is_ready():
            future = self.pull_client.call_async(WaypointPull.Request())
            future.add_done_callback(self.mission_info_callback)
        
        self.process_algorithm_change()

    def save_downstream_task_to_csv(self, task_id, sub_task_id, coords, bearing, base_priority, boosted_priority, landcover_class, class_coverage_pct):
        """Save downstream task to CSV immediately when detected"""
        try:
            downwind_direction = (self.wind_direction + 180) % 360 if self.wind_direction is not None else None
            with open(self.downstream_csv_path, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    self.drone_id, task_id, sub_task_id, coords[0], coords[1], coords[2],
                    bearing, self.wind_direction, downwind_direction, base_priority, boosted_priority,
                    landcover_class, class_coverage_pct
                ])
        except Exception as e:
            self.get_logger().error(f"❌ Error saving downstream task to CSV: {e}")

    def save_task_order_to_csv(self, task_id, task_order, gps_path, replan_reason="initial_plan"):
        try:
            self.plan_counter += 1
            timestamp = datetime.now().isoformat()
            cumulative_distance = 0.0
            with open(self.task_order_csv_path, 'a', newline='') as f:
                writer = csv.writer(f)
                for i, task_data in enumerate(task_order):
                    priority = task_data[0]
                    sub_task_id = task_data[1]
                    coords = task_data[2]
                    if i == 0:
                        prev_pos = (self.current_lat, self.current_lon) if self.current_lat is not None else gps_path[0]
                        distance_from_previous = self.planner.distance_lat_lon(prev_pos, coords) * 111000
                    else:
                        prev_coords = task_order[i-1][2]
                        distance_from_previous = self.planner.distance_lat_lon(prev_coords, coords) * 111000
                    cumulative_distance += distance_from_previous
                    writer.writerow([
                        timestamp, self.plan_counter, i + 1, self.drone_id, task_id, sub_task_id,
                        coords[0], coords[1], coords[2], priority, round(distance_from_previous, 2),
                        round(cumulative_distance, 2), replan_reason
                    ])
            self.get_logger().info(f"Saved task execution order (Plan #{self.plan_counter}) for {task_id}: {len(task_order)} tasks, total distance: {cumulative_distance:.2f}m")
        except Exception as e:
            self.get_logger().error(f"❌ Error saving task order to CSV: {e}")

    def get_current_task_id(self):
        """Get the current fire being processed"""
        if not self.active_fire_queue or self.current_fire_index >= len(self.active_fire_queue):
            return None
        return self.active_fire_queue[self.current_fire_index]

    def advance_to_next_fire(self):
        """Move to the next fire in the queue"""
        self.current_fire_index += 1
        if self.current_fire_index < len(self.active_fire_queue):
            next_fire = self.active_fire_queue[self.current_fire_index]
            self.publish_log(f"Advanced to next fire: {next_fire}")
            return next_fire
        else:
            self.publish_log("All fires completed!")
            return None

    def fire_tasks_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            task_id = data.get('task_id')
            location = data.get('location')
            if task_id and location:
                if isinstance(location, dict) and 'lat' in location and 'lon' in location:
                    new_main_fire = (location['lat'], location['lon'])
                elif isinstance(location, list) and len(location) >= 2:
                    new_main_fire = (location[0], location[1])
                else:
                    self.get_logger().warning(f"Invalid location format: {location}")
                    return

                old_location = self.main_fire_locations.get(task_id)
                if old_location and self.planner.distance_lat_lon(old_location, new_main_fire) < self.waypoint_tolerance:
                    return

                self.main_fire_locations[task_id] = new_main_fire
                self.get_logger().info(f"Updated main fire location for {task_id}: ({new_main_fire[0]:.6f}, {new_main_fire[1]:.6f})")

                if task_id not in self.active_fire_queue:
                    self.active_fire_queue.append(task_id)
                    self.sort_active_fire_queue()

                self.recompute_all_task_priorities()
                
                current_task = self.get_current_task_id()
                if current_task and self.tasks_by_fire.get(current_task) and self.should_replan():
                    self.plan_and_publish_path(replan_reason="fire_location_update")

        except Exception as e:
            self.get_logger().error(f"❌ Error processing fire_tasks: {e}")

    def recompute_all_task_priorities(self):
        """Recompute priorities for ALL fires when wind changes"""
        if not self.tasks_by_fire or not self.main_fire_locations:
            return
            
        downwind_direction = (self.wind_direction + 180) % 360 if self.wind_direction is not None else None
        priority_boost = self.get_parameter('priority_boost').value
        
        for task_id, tasks in self.tasks_by_fire.items():
            main_fire = self.main_fire_locations.get(task_id)
            if not main_fire:
                continue
                
            updated_tasks = []
            for task_data in tasks:
                if len(task_data) == 4:
                    priority, sub_task_id, coords, landcover_data = task_data
                    landcover_class = landcover_data['class_name']
                    class_coverage_pct = landcover_data['class_pct_coverage']
                else:
                    priority, sub_task_id, coords = task_data
                    landcover_class = "Unknown"
                    class_coverage_pct = 0.0
                    landcover_data = {'class_name': landcover_class, 'class_pct_coverage': class_coverage_pct}
                
                base_priority = priority / priority_boost if priority < 0 else priority
                
                if self.wind_direction is not None:
                    bearing = self.planner.calculate_bearing(main_fire[0], main_fire[1], coords[0], coords[1])
                    if self.planner.is_downstream(bearing, downwind_direction):
                        new_priority = base_priority * priority_boost
                        #self.get_logger().info(f"Reprioritized downstream task {sub_task_id} in {task_id}: bearing {bearing:.1f}°, priority {base_priority:.1f} → {new_priority:.1f}")
                        self.save_downstream_task_to_csv(
                            task_id, sub_task_id, coords, bearing, base_priority, new_priority,
                            landcover_class, class_coverage_pct
                        )
                    else:
                        new_priority = base_priority
                else:
                    new_priority = base_priority
                
                updated_tasks.append((new_priority, sub_task_id, coords, landcover_data))
            
            updated_tasks.sort(key=lambda x: x[0])
            self.tasks_by_fire[task_id] = updated_tasks

    def should_replan(self):
        """Throttle replanning to avoid constant path updates"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.last_replan_time < self.replan_throttle_seconds:
            return False
        self.last_replan_time = current_time
        return True

    def wind_direction_callback(self, msg: Float32):
        new_wind_direction = msg.data % 360
        
        if self.wind_direction is not None and abs(new_wind_direction - self.wind_direction) < self.wind_direction_threshold:
            return
        
        self.wind_direction = new_wind_direction
        self.get_logger().info(f"Received wind direction: {self.wind_direction:.1f}°")
        
        self.recompute_all_task_priorities()
        self.prompt_algorithm_change()
        
        current_task = self.get_current_task_id()
        if current_task and self.tasks_by_fire.get(current_task) and self.should_replan():
            self.plan_and_publish_path(replan_reason="wind_direction_update")

    def gps_callback(self, msg: NavSatFix):
        if msg.status.status >= 0:
            self.current_lat = msg.latitude
            self.current_lon = msg.longitude
            if not self.got_initial_gps:
                self.get_logger().info(f"Initial GPS: {self.current_lat:.8f}, {self.current_lon:.8f}")
                self.got_initial_gps = True
                
            current_task = self.get_current_task_id()
            if (current_task and self.tasks_by_fire.get(current_task) and 
                self.main_fire_locations.get(current_task) and 
                self.wind_direction is not None and not self.path_published):
                self.plan_and_publish_path(replan_reason="initial_plan")

    def is_waypoint_visited(self, waypoint):
        lat, lon, alt = waypoint
        for visited_lat, visited_lon, visited_alt in self.visited_waypoints:
            distance = math.sqrt((lat - visited_lat)**2 + (lon - visited_lon)**2)
            if distance < self.waypoint_tolerance:
                self.get_logger().info(f"📍 Waypoint ({lat:.6f}, {lon:.6f}, {alt}) matches visited ({visited_lat:.6f}, {visited_lon:.6f}, {visited_alt})")
                return True
        return False

    def visited_waypoints_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            if data.get('drone_id') != self.drone_id:
                return
            waypoint = tuple(data['waypoint'])
            sub_task_id = data.get('sub_task_id')
            
            if waypoint in self.visited_waypoints:
                return
                
            self.visited_waypoints.add(waypoint)
            lat, lon, alt = waypoint
            self.get_logger().info(f"Marked fire waypoint {sub_task_id or ''} as visited: ({lat:.6f}, {lon:.6f}, {alt})")
            
            self.remove_visited_tasks()
            
            current_task = self.get_current_task_id()
            if current_task and not self.tasks_by_fire.get(current_task):
                self.get_logger().info(f"All tasks for {current_task} completed!")
                next_fire = self.advance_to_next_fire()
                if next_fire and self.should_replan():
                    self.path_published = False
                    self.plan_and_publish_path(replan_reason="new_fire_started")
                    
        except Exception as e:
            self.get_logger().error(f"❌ Error processing visited waypoint: {e}")

    def remove_visited_tasks(self):
        initial_counts = {task_id: len(tasks) for task_id, tasks in self.tasks_by_fire.items()}
        for task_id in list(self.tasks_by_fire.keys()):
            self.tasks_by_fire[task_id] = [
                task for task in self.tasks_by_fire[task_id]
                if not self.is_waypoint_visited(task[2])
            ]
            removed_count = initial_counts[task_id] - len(self.tasks_by_fire[task_id])
            if removed_count > 0:
                self.get_logger().info(f"Removed {removed_count} visited tasks for {task_id}. Remaining: {len(self.tasks_by_fire[task_id])}")
            if not self.tasks_by_fire[task_id]:
                del self.tasks_by_fire[task_id]

    def fire_data_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            task_id = data.get('task_id')
            lat = data.get('lat')
            lon = data.get('lon')
            grid = data.get('landcover_grid_50m', [])
            if lat is None or lon is None:
                return

            if task_id not in self.tasks_by_fire:
                self.tasks_by_fire[task_id] = []
                
            existing_coords = {task[2] for task in self.tasks_by_fire[task_id]}
            new_tasks = []
            downwind_direction = (self.wind_direction + 180) % 360 if self.wind_direction is not None else None
            priority_boost = self.get_parameter('priority_boost').value
            main_fire = self.main_fire_locations.get(task_id)

            for i, pt in enumerate(grid):
                coords = (pt['lat'], pt['lon'], 50.0)
                if self.is_waypoint_visited(coords) or coords in existing_coords:
                    continue
                
                class_weight = landcover_priority_map.get(pt['class_name'], 1)
                pct_coverage = pt.get('class_pct_coverage', 0.0)
                base_priority = - (pct_coverage * class_weight)
                priority = base_priority
                sub_task_id = f"{task_id}_pt_{i:02d}"
                
                landcover_data = {
                    'class_name': pt['class_name'],
                    'class_pct_coverage': pct_coverage
                }
                
                if main_fire and self.wind_direction is not None:
                    bearing = self.planner.calculate_bearing(main_fire[0], main_fire[1], coords[0], coords[1])
                    if self.planner.is_downstream(bearing, downwind_direction):
                        priority = base_priority * priority_boost
                        self.save_downstream_task_to_csv(
                            task_id, sub_task_id, coords, bearing, base_priority, priority,
                            pt['class_name'], pct_coverage
                        )
                        self.get_logger().info(f"⬇Detected downstream task {sub_task_id}: bearing {bearing:.1f}°, priority {base_priority:.1f} → {priority:.1f}")
                
                new_tasks.append((priority, sub_task_id, coords, landcover_data))
                existing_coords.add(coords)
                self.save_simple_task_to_csv(coords, priority)
                
            if new_tasks:
                self.tasks_by_fire[task_id].extend(new_tasks)
                self.tasks_by_fire[task_id].sort(key=lambda x: x[0])
                self.get_logger().info(f"Added {len(new_tasks)} new tasks for {task_id}, total: {len(self.tasks_by_fire[task_id])}")
                
                if self.wind_direction is not None:
                    self.prompt_algorithm_change()
                
                current_task = self.get_current_task_id()
                if task_id == current_task and self.should_replan():
                    self.plan_and_publish_path(replan_reason="new_fire_data")
                    
        except Exception as e:
            self.get_logger().error(f"❌ Error processing fire_data: {e}")

    def plan_and_publish_path(self, replan_reason="unspecified"):
        current_task = self.get_current_task_id()
        if (not current_task or self.current_lat is None or self.current_lon is None or 
            not self.tasks_by_fire.get(current_task) or 
            not self.main_fire_locations.get(current_task) or 
            self.wind_direction is None):
            return

        self.remove_visited_tasks()
        start_pos = (self.current_lat, self.current_lon, 50.0)
        gps_path, task_order = self.planner.plan_path(start_pos, self.tasks_by_fire[current_task], self)
        
        if gps_path and task_order:
            self.save_task_order_to_csv(current_task, task_order, gps_path, replan_reason)
            self.publish_path(gps_path)
            self.last_published_path = gps_path
            self.path_published = True
            self.get_logger().info(f"📍 Planned path for {current_task} with {len(gps_path)} waypoints ({replan_reason})")

    def publish_path(self, gps_waypoints):
        json_data = json.dumps(gps_waypoints)
        self.path_pub.publish(String(data=json_data))
        self.get_logger().info(f"GPS waypoints published to /fire_planner_path_{self.drone_id}")
        for i, wp in enumerate(gps_waypoints[:5]):
            self.get_logger().info(f"  WP {i}: lat={wp[0]}, lon={wp[1]}, alt={wp[2]}")
        if len(gps_waypoints) > 10:
            self.get_logger().info("  ...")
            for i, wp in enumerate(gps_waypoints[-5:], start=len(gps_waypoints)-5):
                self.get_logger().info(f"  WP {i}: lat={wp[0]}, lon={wp[1]}, alt={wp[2]}")

    def sort_active_fire_queue(self):
        """Sort active fire tasks based on precipitation"""
        self.active_fire_queue.sort(key=lambda task_id: self.precip_by_fire.get(task_id, 0.0))
        self.publish_log(f"Sorted active fire queue by precipitation: {self.active_fire_queue}")

    def fire_precip_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            task_id = data.get('task_id')
            precip_mm = data.get('precip_24h_mm', 0.0)
            if task_id:
                self.precip_by_fire[task_id] = precip_mm
                self.get_logger().info(f"Updated 24h precip for {task_id}: {precip_mm:.2f} mm")
                self.sort_active_fire_queue()
        except Exception as e:
            self.get_logger().error(f"❌ Error processing precipitation data: {e}")

    def mission_info_callback(self, future):
        try:
            result = future.result()
            if result.success:
                self.total_wp = result.wp_received
                
        except Exception as e:
            self.get_logger().error(f"Mission info request failed: {e}")

    def reached_callback(self, msg: WaypointReached):
        if self.total_wp is None or not self.last_published_path:
            return
        
        wp_seq = msg.wp_seq
        progress = self.create_progress_bar(wp_seq, self.total_wp - 1)
        self.publish_log(f"[MISSION] Mission Progress: {progress}")
        
        if wp_seq < len(self.last_published_path):
            lat, lon, alt = self.last_published_path[wp_seq]
            self.visited_waypoints.add((lat, lon, alt))
        else:
            self.get_logger().warning(f"Reached waypoint {wp_seq} is out of bounds of last_published_path")

        if wp_seq >= self.total_wp - 1:
            self.publish_log(f"[MISSION] Mission complete for fire {self.get_current_task_id()}")
            next_fire = self.advance_to_next_fire()
            if next_fire and self.should_replan():
                self.path_published = False
                self.plan_and_publish_path(replan_reason="next_fire_started")

    def save_simple_task_to_csv(self, coords, priority):
        """Save just coordinates and priority to CSV"""
        try:
            with open(self.simple_task_csv_path, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([coords[0], coords[1], coords[2], priority])
        except Exception as e:
            self.get_logger().error(f"Error saving simple task to CSV: {e}")

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 2:
        print("Usage: python fire_planner.py <drone_id>")
        sys.exit(1)
    
    drone_id = sys.argv[1]
    
    algorithm = select_algorithm()
    print(f"\nStarting planner with {algorithm.upper().replace('_', ' ')} algorithm...\n")
    
    node = FireDataPlanner(drone_id, algorithm)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Multi-Fire Data Planner shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()