import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
import json
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import sys
import os
import csv
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
    """Plans multi-waypoint paths using lat/lon coordinates with better spatial optimization."""
    def __init__(self, detour_threshold=25.0, nearby_radius=35.0):
        self.detour_threshold = detour_threshold
        self.nearby_radius = nearby_radius

    def distance_lat_lon(self, pos1, pos2):
        """Calculate distance between two lat/lon points (simplified)"""
        lat1, lon1 = pos1[:2]
        lat2, lon2 = pos2[:2]
        return math.sqrt((lat1 - lat2)**2 + (lon1 - lon2)**2)

    def plan_multi_waypoint_path(self, start_pos, tasks, logger=None):
        if not tasks:
            return []

        # Sort by priority first
        sorted_tasks = sorted(tasks, key=lambda x: x[0])
        
        # Use a simple nearest-neighbor approach within priority groups
        path = [start_pos]
        current_pos = start_pos
        unvisited = [task[2] for task in sorted_tasks]  # Just the coordinates
        
        # Visit waypoints in a more spatially efficient order
        while unvisited:
            # Find nearest unvisited waypoint
            nearest_dist = float('inf')
            nearest_idx = 0
            
            for i, wp in enumerate(unvisited):
                dist = self.distance_lat_lon(current_pos, wp)
                if dist < nearest_dist:
                    nearest_dist = dist
                    nearest_idx = i
            
            # Add nearest waypoint to path
            next_waypoint = unvisited.pop(nearest_idx)
            path.append(next_waypoint)
            current_pos = next_waypoint

        return path

class FireDataPlanner(Node):
    def __init__(self, drone_id):
        super().__init__(f'fire_planner_{drone_id}')
        self.drone_id = drone_id

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        # Store wind direction
        self.wind_direction = None
        # Store current GPS
        self.current_lat = None
        self.current_lon = None
        self.got_initial_gps = False

        # Track visited fire data waypoints
        self.visited_waypoints = set()  # Store visited waypoints as (lat, lon, alt) tuples
        self.waypoint_tolerance = 0.00001  # ~1 meter tolerance for matching waypoints

        # Replanning throttle - only replan every N waypoints or when new data arrives
        self.replan_threshold = 5  # Replan after every 5 waypoints visited
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
        #subscribe to wind direction 
        self.wind_direction_sub = self.create_subscription(
            Float32, '/wind_direction', self.wind_direction_callback, qos
        )
        # Subscribe to visited waypoints from drone controller
        self.visited_waypoints_sub = self.create_subscription(
            String, f'/visited_waypoints_{drone_id}', self.visited_waypoints_callback, qos
        )

        # Publish planned GPS waypoints
        self.path_pub = self.create_publisher(String, f'/fire_planner_path_{drone_id}', qos)

        self.all_tasks = []
        self.last_published_path = []
        self.path_published = False
        self.planner = MultiWaypointPlanner()

        self.get_logger().info(f"🔥 Fire Data Planner initialized for {drone_id}")
        self.get_logger().info(f"📡 Subscribed to /visited_waypoints_{drone_id} for waypoint tracking")
        self.get_logger().info(f"⚡ Using optimized replanning (every {self.replan_threshold} waypoints)")
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

    def is_waypoint_visited(self, waypoint):
        """Check if a waypoint has been visited within tolerance"""
        lat, lon, alt = waypoint
        for visited_lat, visited_lon, visited_alt in self.visited_waypoints:
            distance = math.sqrt((lat - visited_lat)**2 + (lon - visited_lon)**2)
            if distance < self.waypoint_tolerance:
                return True
        return False

    def visited_waypoints_callback(self, msg: String):
        """Handle visited waypoint notifications from drone controller"""
        try:
            data = json.loads(msg.data)
            if data.get('drone_id') != self.drone_id:
                return

            waypoint = tuple(data['waypoint'])  # Convert to tuple for set storage
            self.visited_waypoints.add(waypoint)
            
            lat, lon, alt = waypoint
            self.get_logger().info(f"🎯 Marked fire waypoint as visited: ({lat:.6f}, {lon:.6f}, {alt})")
            self.get_logger().info(f"📊 Total visited fire waypoints: {len(self.visited_waypoints)}")

            # Remove visited tasks
            self.remove_visited_tasks()
            
            # Increment counter for replanning throttle
            self.waypoints_since_replan += 1
            
            # Only replan every N waypoints or if we have few remaining tasks
            should_replan = (
                self.waypoints_since_replan >= self.replan_threshold or 
                len(self.all_tasks) <= 10  # Always replan when few tasks remain
            )
            
            if self.all_tasks and should_replan:
                self.waypoints_since_replan = 0  # Reset counter
                self.plan_and_publish_path(replan_from_visited=True)
                self.get_logger().info(f"🔄 Triggered replan (threshold reached or few tasks remaining)")
            elif not self.all_tasks:
                self.get_logger().info("🎉 All fire data tasks completed!")
            else:
                self.get_logger().info(f"⏳ Deferring replan ({self.waypoints_since_replan}/{self.replan_threshold} waypoints since last replan)")

        except Exception as e:
            self.get_logger().error(f"❌ Error processing visited waypoint: {e}")

    def remove_visited_tasks(self):
        """Remove tasks that correspond to visited waypoints"""
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

            # Build tasks from grid points, checking for duplicates
            existing_coords = {task[2] for task in self.all_tasks}  # Track existing task coordinates
            new_tasks = []
            for i, pt in enumerate(grid):
                coords = (pt['lat'], pt['lon'], 50.0)
                if self.is_waypoint_visited(coords):
                    self.get_logger().info(f"⏭️ Skipping already visited fire waypoint: ({coords[0]:.6f}, {coords[1]:.6f})")
                    continue
                if coords in existing_coords:
                    self.get_logger().info(f"⏭️ Skipping duplicate fire waypoint: ({coords[0]:.6f}, {coords[1]:.6f})")
                    continue
                class_weight = landcover_priority_map.get(pt['class_name'], 1)
                pct_coverage = pt.get('class_pct_coverage', 0.0)
                priority = - (pct_coverage * class_weight)
                sub_task_id = f"{task_id}_pt_{i:02d}"
                new_tasks.append((priority, sub_task_id, coords))
                existing_coords.add(coords)

            if not new_tasks:
                self.get_logger().info("ℹ️ No new unvisited or unique tasks to add")
                return

            # Append new tasks and remove visited tasks
            self.all_tasks.extend(new_tasks)
            self.remove_visited_tasks()  # Ensure visited tasks are removed
            self.all_tasks.sort(key=lambda x: x[0])  # Sort by priority
            self.path_published = False

            self.get_logger().info(f"🔥 Received fire data with {len(new_tasks)} new unvisited tasks, total tasks: {len(self.all_tasks)}")

            # Write tasks to CSV
            output_file = os.path.expanduser('~/fire_tasks_dump.csv')
            with open(output_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['task_id', 'sub_task_id', 'lat', 'lon', 'alt', 'priority', 'visited'])
                for prio, sub_id, coords in self.all_tasks:
                    is_visited = self.is_waypoint_visited(coords)
                    writer.writerow([task_id, sub_id, coords[0], coords[1], coords[2], prio, is_visited])
            self.get_logger().info(f"📝 All tasks written to {output_file}")

            # Always replan when new fire data arrives (high priority)
            if self.current_lat is not None and self.current_lon is not None:
                self.waypoints_since_replan = 0  # Reset counter due to new data
                self.plan_and_publish_path()  # Plan from current position
                self.get_logger().info("🚨 Forced replan due to new fire data")
            else:
                self.get_logger().info("⏳ Waiting for current GPS position before planning...")

        except Exception as e:
            self.get_logger().error(f"❌ Error processing fire_data: {e}")

    def plan_and_publish_path(self, append=False, replan_from_visited=False):
        if self.current_lat is None or self.current_lon is None or not self.all_tasks:
            return

        # Remove visited tasks to ensure only unvisited tasks are planned
        self.remove_visited_tasks()

        # Start from current GPS position
        start_pos = (self.current_lat, self.current_lon, 50.0)

        gps_path = self.planner.plan_multi_waypoint_path(start_pos, self.all_tasks, self.get_logger())

        if gps_path:
            self.publish_path(gps_path)
            self.last_published_path = gps_path
            self.path_published = True
            
            if replan_from_visited:
                self.get_logger().info(f"🔄 Replanned path with {len(gps_path)} waypoints (after visiting waypoint)")
            else:
                self.get_logger().info(f"📍 Planned GPS path with {len(gps_path)} waypoints")

    def publish_path(self, gps_waypoints):
        """Publish path as JSON list of [lat, lon, alt]"""
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
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()