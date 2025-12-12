


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import time

class PathPlannerDebugger(Node):
    def __init__(self):
        super().__init__('path_planner_debugger')
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to both topics that the path planner uses
        self.pc_sub = self.create_subscription(
            PointCloud2, 
            '/lidar/points',
            self.pc_callback, 
            qos
        )
        
        self.pose_sub = self.create_subscription(
            PoseStamped, 
            '/drone_1/pose',
            self.pose_callback, 
            qos
        )
        
        # Test planning periodically
        self.test_timer = self.create_timer(5.0, self.test_path_planning)
        
        # Import the fixed planner
        from bidding_pkg.path_planner import AStarPlanner, DirectPlanner, PointCloudProcessor
        
        self.PointCloudProcessor = PointCloudProcessor
        self.test_astar = AStarPlanner('test', self.get_logger())
        self.test_direct = DirectPlanner('test', self.get_logger())
        
        self.current_pose = None
        self.latest_pc = None
        self.test_count = 0
        
        self.get_logger().info("=== PATH PLANNER DEBUG STARTED ===")

    def pc_callback(self, msg):
        self.latest_pc = msg

    def pose_callback(self, msg):
        self.current_pose = msg.pose

    def test_path_planning(self):
        """Test actual path planning with obstacles"""
        self.test_count += 1
        self.get_logger().info(f"\n=== DEBUG TEST #{self.test_count} ===")
        
        if not self.current_pose or not self.latest_pc:
            self.get_logger().error("❌ Missing pose or point cloud data")
            return
        
        try:
            # Simulate obstacle detection (same as your main code)
            obstacles = self._detect_obstacles()
            
            # Update both planners
            self.test_astar.update_obstacles(obstacles)
            self.test_direct.update_obstacles(obstacles)
            
            # Test planning from current position to a goal
            start_pos = (
                self.current_pose.position.x,
                self.current_pose.position.y, 
                self.current_pose.position.z
            )
            
            # Goal 10 meters ahead
            goal_pos = (
                start_pos[0] + 10,
                start_pos[1] + 5,
                start_pos[2]
            )
            
            self.get_logger().info(f"🎯 Planning from {start_pos} to {goal_pos}")
            self.get_logger().info(f"🚧 Using {len(obstacles)} obstacles")
            
            # Test A* planner
            start_time = time.time()
            astar_path = self.test_astar.plan(
                type('obj', (object,), {'x': start_pos[0], 'y': start_pos[1], 'z': start_pos[2]})(),
                goal_pos
            )
            astar_time = time.time() - start_time
            
            # Test Direct planner
            start_time = time.time()
            direct_path = self.test_direct.plan(
                type('obj', (object,), {'x': start_pos[0], 'y': start_pos[1], 'z': start_pos[2]})(),
                goal_pos
            )
            direct_time = time.time() - start_time
            
            # Compare results
            self.get_logger().info(f"📈 RESULTS:")
            self.get_logger().info(f"  A* path: {len(astar_path)} waypoints, {astar_time:.3f}s")
            self.get_logger().info(f"  Direct path: {len(direct_path)} waypoints, {direct_time:.3f}s")
            
            # Check if paths are different (indicating obstacle avoidance)
            if len(astar_path) > 2 and len(obstacles) > 0:
                self.get_logger().info("✅ A* is avoiding obstacles!")
            elif len(obstacles) > 0:
                self.get_logger().warning("⚠️ A* might not be using obstacles properly")
            else:
                self.get_logger().info("ℹ️ No obstacles to avoid")
                
            if len(direct_path) > 2 and len(obstacles) > 0:
                self.get_logger().info("✅ Direct planner is avoiding obstacles!")
                
            # Log sample waypoints
            self.get_logger().info("Sample A* waypoints:")
            for i, wp in enumerate(astar_path[:3]):
                self.get_logger().info(f"  {i}: {wp}")
                
        except Exception as e:
            self.get_logger().error(f"❌ Path planning test failed: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())

    def _detect_obstacles(self):
        """Same obstacle detection as main code"""
        obstacles = set()
        
        if not self.latest_pc or not self.current_pose:
            return obstacles
    
        try:
            points = list(self.PointCloudProcessor.read_points(
                self.latest_pc, 
                field_names=["x", "y", "z"], 
                skip_nans=True
            ))
        
            drone_x = self.current_pose.position.x
            drone_y = self.current_pose.position.y
            drone_z = self.current_pose.position.z
        
            height_tolerance = 2.0
        
            for point in points:
                x, y, z = point[:3]
            
                horizontal_distance = (x*x + y*y)**0.5
                if horizontal_distance < 0.1 or horizontal_distance > 20.0:
                    continue
            
                world_x = drone_x + x
                world_y = drone_y + y
                world_z = drone_z + z
            
                if abs(world_z - drone_z) <= height_tolerance:
                    grid_pos = (
                        int(round(world_x / 2.0)), 
                        int(round(world_y / 2.0)), 
                        int(round(drone_z / 2.0))
                    )
                    obstacles.add(grid_pos)
                elif world_z > drone_z and (world_z - drone_z) <= 5.0:
                    grid_pos = (
                        int(round(world_x / 2.0)), 
                        int(round(world_y / 2.0)), 
                        int(round(drone_z / 2.0))
                    )
                    obstacles.add(grid_pos)
        
            return obstacles
        
        except Exception as e:
            self.get_logger().error(f"Error in obstacle detection: {e}")
            return set()

def main():
    rclpy.init()
    
    debugger = PathPlannerDebugger()
    
    try:
        rclpy.spin(debugger)
    except KeyboardInterrupt:
        pass
    finally:
        debugger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
