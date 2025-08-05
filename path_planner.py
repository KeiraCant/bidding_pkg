
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
import math
import random
import numpy as np
from typing import Dict, Tuple, List
import heapq
from abc import ABC, abstractmethod
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class PathPlanner(ABC):
    """Base class for all path planners"""
    def __init__(self, drone_id: str, logger):
        self.drone_id = drone_id
        self.logger = logger
        self.safety_distance = 3.0
        self.obstacles = set()  # Set of obstacle grid coordinates
        
    @abstractmethod
    def plan(self, start, goal) -> List[Tuple[float, float, float]]:
        pass
    
    def is_safe(self, start, goal, drone_poses: dict) -> bool:
        """Check if direct path is safe from other drones"""
        for drone_id, pose_msg in drone_poses.items():
            if drone_id == self.drone_id:
                continue
                
            drone_pos = pose_msg.pose.position
            if self._point_to_line_distance(
                (start.x, start.y, start.z),
                (goal[0], goal[1], goal[2]),
                (drone_pos.x, drone_pos.y, drone_pos.z)
            ) < self.safety_distance:
                return False
        return True
    
    def _point_to_line_distance(self, line_start, line_end, point):
        """Calculate minimum distance from point to line segment"""
        line_vec = np.array(line_end) - np.array(line_start)
        point_vec = np.array(point) - np.array(line_start)
        line_len = np.linalg.norm(line_vec)
        
        if line_len < 1e-6:
            return np.linalg.norm(point_vec)
        
        line_unitvec = line_vec / line_len
        proj_length = np.dot(point_vec, line_unitvec)
        proj_length = max(min(proj_length, line_len), 0)
        proj = line_unitvec * proj_length
        return np.linalg.norm(point_vec - proj)

class AStarPlanner(PathPlanner):
    """A* path planner with 3D grid-based search"""
    
    def __init__(self, drone_id: str, logger, grid_size: float = 2.0, search_radius: float = 50.0):
        super().__init__(drone_id, logger)
        self.grid_size = grid_size
        self.search_radius = search_radius
        
    def plan(self, start, goal) -> List[Tuple[float, float, float]]:
        """Plan path using A* algorithm"""
        start_pos = (start.x, start.y, start.z)
        goal_pos = tuple(goal)
        
        # Convert to grid coordinates
        start_grid = self._world_to_grid(start_pos)
        goal_grid = self._world_to_grid(goal_pos)
        
        # A* algorithm
        open_set = []
        heapq.heappush(open_set, (0, start_grid))
        came_from = {}
        g_score = {start_grid: 0}
        f_score = {start_grid: self._heuristic(start_grid, goal_grid)}
        
        directions = [
            (1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0),
            (0, 0, 1), (0, 0, -1),  # 6-connected
            (1, 1, 0), (1, -1, 0), (-1, 1, 0), (-1, -1, 0),  # 8-connected in XY
            (1, 0, 1), (-1, 0, 1), (1, 0, -1), (-1, 0, -1),  # Diagonal in XZ
            (0, 1, 1), (0, -1, 1), (0, 1, -1), (0, -1, -1),  # Diagonal in YZ
            (1, 1, 1), (1, 1, -1), (1, -1, 1), (-1, 1, 1),   # 3D diagonals
            (-1, -1, 1), (-1, 1, -1), (1, -1, -1), (-1, -1, -1)
        ]
        
        while open_set:
            current = heapq.heappop(open_set)[1]
            
            if current == goal_grid:
                # Reconstruct path
                path = []
                while current in came_from:
                    world_pos = self._grid_to_world(current)
                    path.append(world_pos)
                    current = came_from[current]
                path.append(start_pos)
                path.reverse()
                
                # Smooth path
                return self._smooth_path(path)
            
            for dx, dy, dz in directions:
                neighbor = (current[0] + dx, current[1] + dy, current[2] + dz)
                
                if self._is_obstacle(neighbor):
                    continue
                
                # Cost is Euclidean distance
                movement_cost = math.sqrt(dx*dx + dy*dy + dz*dz) * self.grid_size
                tentative_g_score = g_score[current] + movement_cost
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self._heuristic(neighbor, goal_grid)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        # No path found, return direct path
        self.logger.warning(f"[drone_{self.drone_id}] A* couldn't find path, using direct route")
        return [start_pos, goal_pos]
    
    def _world_to_grid(self, pos: Tuple[float, float, float]) -> Tuple[int, int, int]:
        """Convert world coordinates to grid coordinates"""
        return (
            int(pos[0] / self.grid_size),
            int(pos[1] / self.grid_size),
            int(pos[2] / self.grid_size)
        )
    
    def _grid_to_world(self, grid_pos: Tuple[int, int, int]) -> Tuple[float, float, float]:
        """Convert grid coordinates to world coordinates"""
        return (
            grid_pos[0] * self.grid_size,
            grid_pos[1] * self.grid_size,
            grid_pos[2] * self.grid_size
        )
    
    def _heuristic(self, pos1: Tuple[int, int, int], pos2: Tuple[int, int, int]) -> float:
        """Euclidean distance heuristic"""
        dx = pos1[0] - pos2[0]
        dy = pos1[1] - pos2[1]
        dz = pos1[2] - pos2[2]
        return math.sqrt(dx*dx + dy*dy + dz*dz) * self.grid_size
    
    def _is_obstacle(self, grid_pos: Tuple[int, int, int]) -> bool:
        """Check if grid position contains an obstacle"""
        return grid_pos in self.obstacles
    
    def _smooth_path(self, path: List[Tuple[float, float, float]]) -> List[Tuple[float, float, float]]:
        """Smooth the path by removing unnecessary waypoints"""
        if len(path) <= 2:
            return path
        
        smoothed = [path[0]]
        i = 0
        
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1:
                if self._is_line_clear(path[i], path[j]):
                    smoothed.append(path[j])
                    i = j
                    break
                j -= 1
            else:
                smoothed.append(path[i + 1])
                i += 1
        
        return smoothed
    
    def _is_line_clear(self, start: Tuple[float, float, float], end: Tuple[float, float, float]) -> bool:
        """Check if line segment is clear of obstacles"""
        steps = int(math.sqrt(sum((e-s)**2 for s, e in zip(start, end))) / self.grid_size) + 1
        for i in range(steps + 1):
            t = i / max(steps, 1)
            point = tuple(s + t * (e - s) for s, e in zip(start, end))
            grid_point = self._world_to_grid(point)
            if self._is_obstacle(grid_point):
                return False
        return True

class RRTStarPlanner(PathPlanner):
    """RRT* (Rapidly-exploring Random Tree Star) path planner"""
    
    def __init__(self, drone_id: str, logger, max_iter: int = 1000, step_size: float = 3.0, 
                 search_radius: float = 50.0, rewire_radius: float = 5.0):
        super().__init__(drone_id, logger)
        self.max_iter = max_iter
        self.step_size = step_size
        self.search_radius = search_radius
        self.rewire_radius = rewire_radius
        
    def plan(self, start, goal) -> List[Tuple[float, float, float]]:
        """Plan path using RRT* algorithm"""
        start_pos = (start.x, start.y, start.z)
        goal_pos = tuple(goal)
        
        # Initialize tree
        nodes = {0: {'pos': start_pos, 'parent': None, 'cost': 0.0}}
        node_id = 1
        goal_node_id = None
        
        for iteration in range(self.max_iter):
            # Sample random point (with goal bias)
            if random.random() < 0.1:  # 10% goal bias
                sample_pos = goal_pos
            else:
                sample_pos = self._sample_random_point(start_pos, goal_pos)
            
            # Find nearest node
            nearest_id = self._find_nearest_node(nodes, sample_pos)
            nearest_pos = nodes[nearest_id]['pos']
            
            # Steer towards sample
            new_pos = self._steer(nearest_pos, sample_pos)
            
            # Check if path is collision-free
            if not self._is_path_clear(nearest_pos, new_pos):
                continue
            
            # Find nodes within rewire radius
            near_nodes = self._find_near_nodes(nodes, new_pos)
            
            # Choose parent (minimize cost)
            best_parent_id = nearest_id
            min_cost = nodes[nearest_id]['cost'] + self._distance(nearest_pos, new_pos)
            
            for near_id in near_nodes:
                near_pos = nodes[near_id]['pos']
                if self._is_path_clear(near_pos, new_pos):
                    cost = nodes[near_id]['cost'] + self._distance(near_pos, new_pos)
                    if cost < min_cost:
                        best_parent_id = near_id
                        min_cost = cost
            
            # Add new node
            nodes[node_id] = {
                'pos': new_pos,
                'parent': best_parent_id,
                'cost': min_cost
            }
            
            # Rewire tree
            for near_id in near_nodes:
                if near_id == best_parent_id:
                    continue
                near_pos = nodes[near_id]['pos']
                new_cost = min_cost + self._distance(new_pos, near_pos)
                if new_cost < nodes[near_id]['cost'] and self._is_path_clear(new_pos, near_pos):
                    nodes[near_id]['parent'] = node_id
                    nodes[near_id]['cost'] = new_cost
                    self._update_children_cost(nodes, near_id)
            
            # Check if we reached the goal
            if self._distance(new_pos, goal_pos) < self.step_size:
                if self._is_path_clear(new_pos, goal_pos):
                    goal_cost = min_cost + self._distance(new_pos, goal_pos)
                    if goal_node_id is None or goal_cost < nodes[goal_node_id]['cost']:
                        goal_node_id = node_id + 1
                        nodes[goal_node_id] = {
                            'pos': goal_pos,
                            'parent': node_id,
                            'cost': goal_cost
                        }
            
            node_id += 1
        
        # Extract path
        if goal_node_id is not None:
            path = []
            current_id = goal_node_id
            while current_id is not None:
                path.append(nodes[current_id]['pos'])
                current_id = nodes[current_id]['parent']
            path.reverse()
            self.logger.info(f"[drone_{self.drone_id}] RRT* found path with {len(path)} waypoints, cost: {nodes[goal_node_id]['cost']:.2f}")
            return path
        else:
            self.logger.warning(f"[drone_{self.drone_id}] RRT* couldn't find path, using direct route")
            return [start_pos, goal_pos]
    
    def _sample_random_point(self, start_pos: Tuple[float, float, float], 
                           goal_pos: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """Sample a random point within search area"""
        center_x = (start_pos[0] + goal_pos[0]) / 2
        center_y = (start_pos[1] + goal_pos[1]) / 2
        center_z = (start_pos[2] + goal_pos[2]) / 2
        
        return (
            center_x + random.uniform(-self.search_radius, self.search_radius),
            center_y + random.uniform(-self.search_radius, self.search_radius),
            center_z + random.uniform(-5, 15)  # Keep reasonable altitude
        )
    
    def _find_nearest_node(self, nodes: Dict, pos: Tuple[float, float, float]) -> int:
        """Find the nearest node to given position"""
        min_dist = float('inf')
        nearest_id = 0
        
        for node_id, node in nodes.items():
            dist = self._distance(node['pos'], pos)
            if dist < min_dist:
                min_dist = dist
                nearest_id = node_id
        
        return nearest_id
    
    def _find_near_nodes(self, nodes: Dict, pos: Tuple[float, float, float]) -> List[int]:
        """Find nodes within rewire radius"""
        near_nodes = []
        for node_id, node in nodes.items():
            if self._distance(node['pos'], pos) <= self.rewire_radius:
                near_nodes.append(node_id)
        return near_nodes
    
    def _steer(self, from_pos: Tuple[float, float, float], 
              to_pos: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """Steer from one position towards another with step size limit"""
        dist = self._distance(from_pos, to_pos)
        if dist <= self.step_size:
            return to_pos
        
        ratio = self.step_size / dist
        return (
            from_pos[0] + ratio * (to_pos[0] - from_pos[0]),
            from_pos[1] + ratio * (to_pos[1] - from_pos[1]),
            from_pos[2] + ratio * (to_pos[2] - from_pos[2])
        )
    
    def _distance(self, pos1: Tuple[float, float, float], pos2: Tuple[float, float, float]) -> float:
        """Calculate Euclidean distance between two positions"""
        return math.sqrt(sum((p1 - p2)**2 for p1, p2 in zip(pos1, pos2)))
    
    def _is_path_clear(self, start_pos: Tuple[float, float, float], 
                      end_pos: Tuple[float, float, float]) -> bool:
        """Check if path between two points is clear"""
        steps = int(self._distance(start_pos, end_pos) / 0.5) + 1
        for i in range(steps + 1):
            t = i / max(steps, 1)
            point = tuple(s + t * (e - s) for s, e in zip(start_pos, end_pos))
            grid_point = (
                int(point[0] / 2.0),
                int(point[1] / 2.0),
                int(point[2] / 2.0)
            )
            if grid_point in self.obstacles:
                return False
        return True
    
    def _update_children_cost(self, nodes: Dict, parent_id: int):
        """Recursively update cost of children nodes"""
        for node_id, node in nodes.items():
            if node['parent'] == parent_id:
                old_cost = node['cost']
                new_cost = nodes[parent_id]['cost'] + self._distance(nodes[parent_id]['pos'], node['pos'])
                node['cost'] = new_cost
                if new_cost != old_cost:  # Only recurse if cost changed
                    self._update_children_cost(nodes, node_id)

class DijkstraPlanner(PathPlanner):
    """Dijkstra's algorithm path planner with 3D grid"""
    
    def __init__(self, drone_id: str, logger, grid_size: float = 2.0, search_radius: float = 50.0):
        super().__init__(drone_id, logger)
        self.grid_size = grid_size
        self.search_radius = search_radius
        
    def plan(self, start, goal) -> List[Tuple[float, float, float]]:
        """Plan path using Dijkstra's algorithm"""
        start_pos = (start.x, start.y, start.z)
        goal_pos = tuple(goal)
        
        # Convert to grid coordinates
        start_grid = self._world_to_grid(start_pos)
        goal_grid = self._world_to_grid(goal_pos)
        
        # Dijkstra's algorithm
        distances = {start_grid: 0}
        previous = {}
        unvisited = set()
        
        # Priority queue: (distance, node)
        pq = [(0, start_grid)]
        
        # 26-connected neighborhood (3D)
        directions = [
            (dx, dy, dz) 
            for dx in [-1, 0, 1] 
            for dy in [-1, 0, 1] 
            for dz in [-1, 0, 1] 
            if not (dx == 0 and dy == 0 and dz == 0)
        ]
        
        while pq:
            current_dist, current = heapq.heappop(pq)
            
            if current == goal_grid:
                # Reconstruct path
                path = []
                while current in previous:
                    world_pos = self._grid_to_world(current)
                    path.append(world_pos)
                    current = previous[current]
                path.append(start_pos)
                path.reverse()
                
                self.logger.info(f"[drone_{self.drone_id}] Dijkstra found path with {len(path)} waypoints, cost: {current_dist:.2f}")
                return self._smooth_path(path)
            
            if current_dist > distances.get(current, float('inf')):
                continue
            
            for dx, dy, dz in directions:
                neighbor = (current[0] + dx, current[1] + dy, current[2] + dz)
                
                if self._is_obstacle(neighbor):
                    continue
                
                # Check if neighbor is within search bounds
                neighbor_world = self._grid_to_world(neighbor)
                if self._distance_3d(start_pos, neighbor_world) > self.search_radius:
                    continue
                
                # Calculate edge weight (Euclidean distance)
                edge_weight = math.sqrt(dx*dx + dy*dy + dz*dz) * self.grid_size
                new_distance = current_dist + edge_weight
                
                if new_distance < distances.get(neighbor, float('inf')):
                    distances[neighbor] = new_distance
                    previous[neighbor] = current
                    heapq.heappush(pq, (new_distance, neighbor))
        
        # No path found
        self.logger.warning(f"[drone_{self.drone_id}] Dijkstra couldn't find path, using direct route")
        return [start_pos, goal_pos]
    
    def _world_to_grid(self, pos: Tuple[float, float, float]) -> Tuple[int, int, int]:
        """Convert world coordinates to grid coordinates"""
        return (
            int(pos[0] / self.grid_size),
            int(pos[1] / self.grid_size),
            int(pos[2] / self.grid_size)
        )
    
    def _grid_to_world(self, grid_pos: Tuple[int, int, int]) -> Tuple[float, float, float]:
        """Convert grid coordinates to world coordinates"""
        return (
            grid_pos[0] * self.grid_size,
            grid_pos[1] * self.grid_size,
            grid_pos[2] * self.grid_size
        )
    
    def _distance_3d(self, pos1: Tuple[float, float, float], pos2: Tuple[float, float, float]) -> float:
        """Calculate 3D Euclidean distance"""
        return math.sqrt(sum((p1 - p2)**2 for p1, p2 in zip(pos1, pos2)))
    
    def _is_obstacle(self, grid_pos: Tuple[int, int, int]) -> bool:
        """Check if grid position contains an obstacle"""
        return grid_pos in self.obstacles
    
    def _smooth_path(self, path: List[Tuple[float, float, float]]) -> List[Tuple[float, float, float]]:
        """Smooth the path by removing unnecessary waypoints"""
        if len(path) <= 2:
            return path
        
        smoothed = [path[0]]
        i = 0
        
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1:
                if self._is_line_clear(path[i], path[j]):
                    smoothed.append(path[j])
                    i = j
                    break
                j -= 1
            else:
                smoothed.append(path[i + 1])
                i += 1
        
        return smoothed
    
    def _is_line_clear(self, start: Tuple[float, float, float], end: Tuple[float, float, float]) -> bool:
        """Check if line segment is clear of obstacles"""
        steps = int(self._distance_3d(start, end) / self.grid_size) + 1
        for i in range(steps + 1):
            t = i / max(steps, 1)
            point = tuple(s + t * (e - s) for s, e in zip(start, end))
            grid_point = self._world_to_grid(point)
            if self._is_obstacle(grid_point):
                return False
        return True

class DirectPlanner(PathPlanner):
    """Simple direct path planner"""
    def plan(self, start, goal) -> List[Tuple[float, float, float]]:
        self.logger.info(f"[drone_{self.drone_id}] DirectPlanner planning from {(start.x, start.y, start.z)} to {tuple(goal)}")
        return [(start.x, start.y, start.z), tuple(goal)]

class WaypointPlanner(PathPlanner):
    """Simple waypoint planner with intermediate points"""
    def plan(self, start, goal) -> List[Tuple[float, float, float]]:
        start_pos = (start.x, start.y, start.z)
        goal_pos = tuple(goal)
        
        # Add intermediate waypoint
        mid_x = (start_pos[0] + goal_pos[0]) / 2
        mid_y = (start_pos[1] + goal_pos[1]) / 2
        mid_z = max(start_pos[2], goal_pos[2]) + 5  # Higher altitude
        
        self.logger.info(f"[drone_{self.drone_id}] WaypointPlanner planning with intermediate point {(mid_x, mid_y, mid_z)}")
        return [start_pos, (mid_x, mid_y, mid_z), goal_pos]

class AvoidancePlanner(PathPlanner):
    """Simple avoidance planner that goes around obstacles"""
    def plan(self, start, goal) -> List[Tuple[float, float, float]]:
        start_pos = (start.x, start.y, start.z)
        goal_pos = tuple(goal)
        
        # Simple avoidance: go up, then to goal, then down
        safe_altitude = 15.0
        waypoints = [
            start_pos,
            (start_pos[0], start_pos[1], safe_altitude),
            (goal_pos[0], goal_pos[1], safe_altitude),
            goal_pos
        ]
        
        self.logger.info(f"[drone_{self.drone_id}] AvoidancePlanner planning with safe altitude {safe_altitude}")
        return waypoints

class PlannerSelector(Node):
    def __init__(self, drone_id: str):
        super().__init__(f'planner_selector_drone_{drone_id}')
        self.drone_id = drone_id
        
        # QoS profile for subscriptions
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to LIDAR data
        self.lidar_sub = self.create_subscription(
            LaserScan, f'/drone_{drone_id}/scan', self.lidar_callback, qos
        )
        self.lidar_ranges = []
        
        # Subscribe to drone pose
        self.current_pose = None
        self.pose_sub = self.create_subscription(
            PoseStamped, f'/drone_{drone_id}/pose', self.pose_callback, qos
        )
        
        self.get_logger().info(f"PlannerSelector initialized for drone_{drone_id}")

    def pose_callback(self, msg: PoseStamped):
        """Store current drone pose"""
        self.current_pose = msg.pose
        self.get_logger().debug(f"[drone_{self.drone_id}] Received pose: x={msg.pose.position.x}, y={msg.pose.position.y}, z={msg.pose.position.z}")

    def lidar_callback(self, msg: LaserScan):
        """Store LIDAR data"""
        self.lidar_ranges = msg.ranges
        self.get_logger().debug(f"[drone_{self.drone_id}] Received LIDAR data with {len(self.lidar_ranges)} ranges")

    def _lidar_to_obstacles(self) -> set:
        """Convert LIDAR ranges to obstacle grid positions, only for drone_1"""
        if self.drone_id != '1':
            self.get_logger().info(f"[drone_{self.drone_id}] No obstacle processing (drone_id != 1)")
            return set()
        
        obstacles = set()
        if not self.lidar_ranges or not self.current_pose:
            self.get_logger().warning(f"[drone_{self.drone_id}] No LIDAR data or pose available")
            return obstacles
        
        for i, r in enumerate(self.lidar_ranges):
            if math.isinf(r) or r < 0.1 or r > 20.0:  # Ignore invalid ranges
                continue
            angle = -3.14159 + i * (2 * 3.14159 / 360)  # Convert index to angle
            x = self.current_pose.position.x + r * math.cos(angle)
            y = self.current_pose.position.y + r * math.sin(angle)
            z = self.current_pose.position.z
            grid_pos = (int(x / 2.0), int(y / 2.0), int(z / 2.0))  # 2m grid cells to match AStar/Dijkstra
            obstacles.add(grid_pos)
        
        self.get_logger().info(f"[drone_{self.drone_id}] Detected {len(obstacles)} obstacles from LIDAR")
        return obstacles

    def _estimate_obstacle_density(self, drone_poses: Dict, start: Tuple[float, float, float], 
                                 goal: Tuple[float, float, float]) -> float:
        """Estimate obstacle density"""
        volume = 50.0 * 50.0 * 20.0
        obstacles = self._lidar_to_obstacles()
        num_obstacles = len(obstacles)
        self.get_logger().info(f"[drone_{self.drone_id}] Total obstacles: {num_obstacles}, density: {num_obstacles / volume}")
        return num_obstacles / volume

    def select_planner(self, fire_data: dict, start: Tuple[float, float, float], 
                      goal: Tuple[float, float, float], drone_poses: Dict, 
                      time_constraint: float) -> PathPlanner:
        """Select planner based on fire priority and obstacle density"""
        density = self._estimate_obstacle_density(drone_poses, start, goal)
        priority = fire_data.get('priority_level', 5)
        obstacles = self._lidar_to_obstacles()
        
        # Update obstacles for all planners
        planner_instances = {
            'astar': AStarPlanner(self.drone_id, self.get_logger()),
            'rrtstar': RRTStarPlanner(self.drone_id, self.get_logger()),
            'dijkstra': DijkstraPlanner(self.drone_id, self.get_logger()),
            'direct': DirectPlanner(self.drone_id, self.get_logger()),
            'waypoint': WaypointPlanner(self.drone_id, self.get_logger()),
            'avoidance': AvoidancePlanner(self.drone_id, self.get_logger())
        }
        for planner in planner_instances.values():
            planner.obstacles = obstacles
        
        if priority <= 2:  # High-priority fire
            self.get_logger().info(f"[drone_{self.drone_id}] Selecting AStarPlanner due to high priority ({priority})")
            return planner_instances['astar']
        elif density > 0.1:  # High obstacle density
            self.get_logger().info(f"[drone_{self.drone_id}] Selecting RRTStarPlanner due to high density ({density})")
            return planner_instances['rrtstar']
        elif time_constraint < 30.0:  # Tight time constraint
            self.get_logger().info(f"[drone_{self.drone_id}] Selecting DirectPlanner due to time constraint ({time_constraint})")
            return planner_instances['direct']
        elif density > 0.05:  # Moderate obstacle density
            self.get_logger().info(f"[drone_{self.drone_id}] Selecting DijkstraPlanner due to moderate density ({density})")
            return planner_instances['dijkstra']
        elif priority <= 3:  # Medium-priority fire
            self.get_logger().info(f"[drone_{self.drone_id}] Selecting WaypointPlanner due to medium priority ({priority})")
            return planner_instances['waypoint']
        else:
            self.get_logger().info(f"[drone_{self.drone_id}] Selecting AvoidancePlanner")
            return planner_instances['avoidance']

def main(args=None):
    rclpy.init(args=args)
    import sys
    drone_id = sys.argv[1] if len(sys.argv) > 1 else '1'
    planner_selector = PlannerSelector(drone_id)
    rclpy.spin(planner_selector)
    planner_selector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

