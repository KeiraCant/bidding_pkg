# path_planner.py - Simple pluggable path planning

import math

class PathPlanner:
    def __init__(self, drone_id, logger=None):
        self.drone_id = drone_id
        self.logger = logger
        self.safety_radius = 5.0
    
    def is_safe(self, current_pos, goal, other_drones):
        """Override this method in your custom planners"""
        # Default: check if goal is safe
        for drone_id, pose_msg in other_drones.items():
            pos = pose_msg.pose.position
            dist = math.sqrt(
                (goal[0] - pos.x)**2 +
                (goal[1] - pos.y)**2 +
                (goal[2] - pos.z)**2
            )
            if dist < self.safety_radius:
                return False
        return True
    
    def plan(self, current_pos, goal):
        """Override this method in your custom planners"""
        # Default: direct path
        return [
            [current_pos.x, current_pos.y, current_pos.z],
            goal
        ]


# Example planners - just copy and modify these!

class DirectPlanner(PathPlanner):
    """Goes straight to target"""
    def plan(self, current_pos, goal):
        return [goal]  # Just go direct


class WaypointPlanner(PathPlanner):
    """Adds waypoints every 5 meters"""
    def plan(self, current_pos, goal):
        start = [current_pos.x, current_pos.y, current_pos.z]
        
        # Calculate distance
        dx = goal[0] - start[0]
        dy = goal[1] - start[1] 
        dz = goal[2] - start[2]
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        
        if distance < 5.0:
            return [goal]
        
        # Add waypoints every 5 meters
        steps = int(distance / 5.0)
        waypoints = []
        
        for i in range(1, steps + 1):
            progress = i / steps
            waypoint = [
                start[0] + dx * progress,
                start[1] + dy * progress, 
                start[2] + dz * progress
            ]
            waypoints.append(waypoint)
        
        waypoints.append(goal)
        return waypoints


class AvoidancePlanner(PathPlanner):
    """Takes detour if blocked"""
    def is_safe(self, current_pos, goal, other_drones):
        return True  # Always plan around obstacles
    
    def plan(self, current_pos, goal):
        start = [current_pos.x, current_pos.y, current_pos.z]
        
        # Simple detour: go up and around
        detour = [
            (start[0] + goal[0]) / 2,  # Midpoint X
            (start[1] + goal[1]) / 2 + 10,  # Midpoint Y + 10m detour
            max(start[2], goal[2]) + 5  # Higher altitude
        ]
        
        return [detour, goal]
