import math
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav_msgs.msg import Path


# Constants for your known base reference coordinates
BASE_LAT = 63.4188137258128
BASE_LON = 10.401553034756608

def latlon_to_xy(lat, lon):
    """Convert lat/lon to local XY in meters relative to base."""
    R = 6378137  # Earth radius in meters
    dx = (lon - BASE_LON) * (math.pi / 180) * R * math.cos(math.radians(BASE_LAT))
    dy = (lat - BASE_LAT) * (math.pi / 180) * R
    return dx, dy


class MultiWaypointPlanner:
    """Simple path planner that visits all waypoints in priority order."""
    
    def __init__(self, detour_threshold=20.0, nearby_radius=30.0):
        self.detour_threshold = detour_threshold
        self.nearby_radius = nearby_radius

    def distance(self, p1, p2):
        """Calculate Euclidean distance between two points."""
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def plan_multi_waypoint_path(self, start_pos, priority_tasks, logger=None):
        """Plan a path visiting all waypoints in priority order with smart detours."""
        if not priority_tasks:
            return []
            
        if logger:
            logger.info(f"Planning multi-waypoint path for {len(priority_tasks)} tasks")

        # Extract coordinates and sort by priority (highest urban % first)
        unvisited = [(coords, priority, task_id) for priority, task_id, coords in priority_tasks]
        path = [start_pos[:2]]  # Start with current position (x,y only)
        current_pos = start_pos[:2]
        total_distance = 0.0

        while unvisited:
            # Find next waypoint with detours
            next_idx, detour_indices = self._find_next_waypoint_with_detours(
                current_pos, unvisited, logger
            )
            
            # Add detour points first (in reverse order to maintain indices)
            for detour_idx in sorted(detour_indices, reverse=True):
                detour_coords, detour_priority, detour_task_id = unvisited.pop(detour_idx)
                path.append(detour_coords[:2])
                
                distance_to_detour = self.distance(current_pos, detour_coords[:2])
                total_distance += distance_to_detour
                current_pos = detour_coords[:2]
                
                if logger:
                    logger.debug(f"Detour to {detour_task_id} (priority={-detour_priority:.1f}%), distance={distance_to_detour:.1f}m")

            # Adjust next_idx after removing detour points
            for detour_idx in sorted(detour_indices):
                if detour_idx <= next_idx:
                    next_idx -= 1

            # Add main next waypoint
            if next_idx < len(unvisited):
                next_coords, next_priority, next_task_id = unvisited.pop(next_idx)
                path.append(next_coords[:2])
                
                distance_to_next = self.distance(current_pos, next_coords[:2])
                total_distance += distance_to_next
                current_pos = next_coords[:2]
                
                if logger:
                    logger.debug(f"Main waypoint: {next_task_id} (priority={-next_priority:.1f}%), distance={distance_to_next:.1f}m")

        if logger:
            logger.info(f"Multi-waypoint path complete: {len(path)} waypoints, total distance: {total_distance:.1f}m")

        return path

    def _find_next_waypoint_with_detours(self, current_pos, unvisited, logger=None):
        """Find next waypoint with detour opportunities."""
        if not unvisited:
            return 0, []

        # Pick highest priority point
        priority_sorted = sorted(enumerate(unvisited), key=lambda x: x[1][1])
        highest_priority_idx = priority_sorted[0][0]
        highest_priority_coords = unvisited[highest_priority_idx][0][:2]
        
        # Find detour candidates
        detour_candidates = []
        for i, (coords, priority, task_id) in enumerate(unvisited):
            if i == highest_priority_idx:
                continue
                
            coords_2d = coords[:2]
            direct_distance = self.distance(current_pos, highest_priority_coords)
            detour_distance = (self.distance(current_pos, coords_2d) + 
                             self.distance(coords_2d, highest_priority_coords))
            extra_distance = detour_distance - direct_distance
            
            near_current = self.distance(current_pos, coords_2d) <= self.nearby_radius
            near_target = self.distance(highest_priority_coords, coords_2d) <= self.nearby_radius
            
            if extra_distance <= self.detour_threshold or near_current or near_target:
                detour_candidates.append((i, extra_distance, priority, coords_2d))
                
        # Sort by extra distance and limit detours
        detour_candidates.sort(key=lambda x: x[1])
        max_detours = 3
        selected_detours = detour_candidates[:max_detours]
        detour_indices = [idx for idx, _, _, _ in selected_detours]
        
        return highest_priority_idx, detour_indices


class FireDataPlanner(Node):
    def __init__(self, drone_id: str):
        super().__init__(f'fire_planner_drone_{drone_id}')
        self.drone_id = drone_id

        # Use consistent QoS settings
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to drone pose
        self.pose_subscription = self.create_subscription(
            PoseStamped, f'/drone_{drone_id}/mavros/local_position/pose', self.pose_callback, qos)

        # Subscribe to fire data
        self.fire_data_subscription = self.create_subscription(
            String, '/fire_data', self.fire_data_callback, qos)

        # Publish planned path
        # Publish planned path
        self.path_pub = self.create_publisher(Path, f'/fire_planner_path_drone_{drone_id}', qos)

        # State variables
        self.current_pose = None
        self.all_tasks = []
        self.path_published = False

        # Planner instance
        self.planner = MultiWaypointPlanner(
            detour_threshold=25.0,
            nearby_radius=35.0
        )

        # Timer to republish path periodically (helps with QoS issues)
        self.publish_timer = self.create_timer(3.0, self.republish_path)

        self.get_logger().info(f"🔥 Fire Data Planner initialized for drone_{drone_id}")
        self.get_logger().info("Waiting for fire data and drone pose...")

    def pose_callback(self, msg: PoseStamped):
        """Handle pose updates from drone."""
        was_none = self.current_pose is None
        self.current_pose = msg
        
        if was_none:
            self.get_logger().info(f"✅ Received initial pose: ({msg.pose.position.x:.1f}, {msg.pose.position.y:.1f}, {msg.pose.position.z:.1f})")
        
        # Try to plan path if we have both pose and tasks
        if self.all_tasks and not self.path_published:
            self.plan_and_publish_path()

    def fire_data_callback(self, msg: String):
        """Handle fire data and build task list."""
        try:
            data = json.loads(msg.data)
            task_id = data.get('task_id')
            lat = data.get('lat')
            lon = data.get('lon')
            landcover_grid = data.get('landcover_grid_50m', [])

            if not task_id or lat is None or lon is None:
                self.get_logger().warning(f"Incomplete fire_data: {data}")
                return

            self.get_logger().info(f"🔥 Processing fire data: {task_id} with {len(landcover_grid)} grid points")

            # Convert grid points to tasks with urban priority
            points_with_urban_pct = []
            for pt in landcover_grid:
                x, y = latlon_to_xy(pt['lat'], pt['lon'])
                urban_pct = 0.0
                if pt['class_code'] == 50:  # Built-up areas
                    urban_pct = pt.get('class_pct_coverage', 0.0)
                points_with_urban_pct.append((urban_pct, (x, y, 50.0)))

            # Sort by urban percentage (highest first)
            points_with_urban_pct.sort(key=lambda x: x[0], reverse=True)

            # Create task list: (priority, task_id, coords)
            # Use negative urban_pct for priority (lower = higher priority)
            new_tasks = []
            for i, (urban_pct, coords) in enumerate(points_with_urban_pct):
                priority = -urban_pct  # Negative so lower = higher priority
                sub_task_id = f"{task_id}_point_{i:02d}"
                new_tasks.append((priority, sub_task_id, coords))

            # Add to task list and sort by priority
            self.all_tasks.extend(new_tasks)
            self.all_tasks.sort(key=lambda x: x[0])  # Sort by priority (most urgent first)

            urban_count = sum(1 for urban_pct, _ in points_with_urban_pct if urban_pct > 0)
            self.get_logger().info(f"📊 Added {len(new_tasks)} tasks ({urban_count} urban areas)")
            self.get_logger().info(f"📋 Total tasks: {len(self.all_tasks)}")
            
            # Show top priorities
            self.get_logger().info("🏆 Top 5 priority areas:")
            for i, (priority, task_id, coords) in enumerate(self.all_tasks[:5]):
                self.get_logger().info(f"   {i+1}. {task_id}: {-priority:.1f}% urban at ({coords[0]:.1f}, {coords[1]:.1f})")

            # Reset path published flag to trigger new planning
            self.path_published = False
            
            # Try to plan path if we have pose
            if self.current_pose:
                self.plan_and_publish_path()

        except Exception as e:
            self.get_logger().error(f"❌ Error processing fire_data: {e}")

    def plan_and_publish_path(self):
        """Plan complete path and publish it."""
        if not self.current_pose or not self.all_tasks or self.path_published:
            return

        start_pos = (
            self.current_pose.pose.position.x,
            self.current_pose.pose.position.y,
            self.current_pose.pose.position.z
        )

        self.get_logger().info(f"🗺️  Planning complete path from ({start_pos[0]:.1f}, {start_pos[1]:.1f})")
        self.get_logger().info(f"🎯 Will visit {len(self.all_tasks)} fire response locations")

        # Plan the complete multi-waypoint path
        complete_path = self.planner.plan_multi_waypoint_path(
            start_pos, self.all_tasks, self.get_logger()
        )

        if complete_path:
            # Publish the path
            self.publish_path(complete_path)
            self.path_published = True
            
            self.get_logger().info(f"✅ Published complete path with {len(complete_path)} waypoints")
            self.log_path_summary(complete_path)
        else:
            self.get_logger().error("❌ Failed to generate path")

    def publish_path(self, waypoint_list):
        """Publish path as ROS Path message."""
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for i, waypoint in enumerate(waypoint_list):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            
            pose.pose.position.x = float(waypoint[0])
            pose.pose.position.y = float(waypoint[1])
            pose.pose.position.z = 50.0  # Fixed altitude
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
        self.get_logger().info(f"📡 Path published: {len(path_msg.poses)} waypoints sent to drone controller")

    def republish_path(self):
        """Periodically republish path to help with reliability."""
        if self.path_published and self.all_tasks:
            # Only republish if we have a planned path
            if self.current_pose:
                start_pos = (
                    self.current_pose.pose.position.x,
                    self.current_pose.pose.position.y,
                    self.current_pose.pose.position.z
                )
                complete_path = self.planner.plan_multi_waypoint_path(
                    start_pos, self.all_tasks, None  # Don't log details on republish
                )
                if complete_path:
                    self.publish_path(complete_path)
                    self.get_logger().debug(f"🔄 Republished path with {len(complete_path)} waypoints")

    def log_path_summary(self, path):
        """Log a summary of the planned path."""
        if len(path) < 2:
            return
            
        self.get_logger().info("📍 Path Summary:")
        total_distance = 0.0
        
        # Show first few waypoints
        for i in range(min(3, len(path))):
            wp = path[i]
            if i == 0:
                self.get_logger().info(f"   START: ({wp[0]:.1f}, {wp[1]:.1f})")
            else:
                prev_wp = path[i-1]
                dist = math.sqrt((wp[0] - prev_wp[0])**2 + (wp[1] - prev_wp[1])**2)
                total_distance += dist
                self.get_logger().info(f"   WP {i}: ({wp[0]:.1f}, {wp[1]:.1f}) [{dist:.1f}m]")
        
        # Calculate total distance
        for i in range(1, len(path)):
            if i >= 3:  # Already calculated first few
                prev_wp = path[i-1]
                wp = path[i]
                dist = math.sqrt((wp[0] - prev_wp[0])**2 + (wp[1] - prev_wp[1])**2)
                total_distance += dist
        
        if len(path) > 3:
            self.get_logger().info(f"   ... ({len(path)-3} more waypoints)")
            # Show last waypoint
            last_wp = path[-1]
            self.get_logger().info(f"   END: ({last_wp[0]:.1f}, {last_wp[1]:.1f})")
        
        avg_distance = total_distance / (len(path) - 1) if len(path) > 1 else 0
        self.get_logger().info(f"📏 Total distance: {total_distance:.1f}m, Average segment: {avg_distance:.1f}m")


def main(args=None):
    rclpy.init(args=args)
    import sys
    
    if len(sys.argv) < 2:
        print("Usage: python3 fire_planner.py <drone_id>")
        print("Example: python3 fire_planner.py 1")
        sys.exit(1)
    
    drone_id = sys.argv[1]
    planner = FireDataPlanner(drone_id)

    try:
        planner.get_logger().info(f"🚀 Fire Data Planner started for drone_{drone_id}")
        planner.get_logger().info("Ready to receive fire data and plan complete paths!")
        rclpy.spin(planner)
    except KeyboardInterrupt:
        planner.get_logger().info("🛑 Shutting down...")
    finally:
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
