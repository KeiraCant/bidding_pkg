
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import struct
import math

class PointCloudDecoder(Node):
    def __init__(self):
        super().__init__('pointcloud_decoder')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/lidar/points',
            self.pointcloud_callback,
            10)
        self.get_logger().info("🔍 Waiting for point cloud data...")
        self.sample_count = 0

    def pointcloud_callback(self, msg):
        self.sample_count += 1
        if self.sample_count > 3:  # Only analyze first few messages
            return
            
        self.get_logger().info(f"=== POINT CLOUD ANALYSIS #{self.sample_count} ===")
        self.get_logger().info(f"📊 Dimensions: {msg.width}W × {msg.height}H = {msg.width * max(1, msg.height)} points")
        self.get_logger().info(f"📏 Point step: {msg.point_step} bytes")
        self.get_logger().info(f"📦 Data size: {len(msg.data)} bytes")
        self.get_logger().info(f"🔗 Frame ID: {msg.header.frame_id}")
        
        # Find field info
        field_map = {}
        for field in msg.fields:
            field_map[field.name] = (field.offset, field.datatype)
            
        self.get_logger().info(f"🏗️ Fields: {list(field_map.keys())}")
        
        if 'x' in field_map and 'y' in field_map and 'z' in field_map:
            # Extract sample points
            self.get_logger().info(f"=== SAMPLE COORDINATES ===")
            
            x_offset, _ = field_map['x']
            y_offset, _ = field_map['y'] 
            z_offset, _ = field_map['z']
            
            z_values = []
            valid_points = 0
            
            # Check first 20 points and some scattered ones
            sample_indices = list(range(20)) + [100, 200, 500, 1000, 2000, 5000, 10000]
            sample_indices = [i for i in sample_indices if i < msg.width * max(1, msg.height)]
            
            for i in sample_indices:
                try:
                    point_start = i * msg.point_step
                    point_data = msg.data[point_start:point_start + msg.point_step]
                    
                    x = struct.unpack_from('f', point_data, x_offset)[0]
                    y = struct.unpack_from('f', point_data, y_offset)[0]
                    z = struct.unpack_from('f', point_data, z_offset)[0]
                    
                    # Skip invalid points
                    if math.isnan(x) or math.isnan(y) or math.isnan(z):
                        continue
                        
                    z_values.append(z)
                    valid_points += 1
                    
                    # Show first 10 valid points
                    if valid_points <= 10:
                        distance = math.sqrt(x*x + y*y + z*z)
                        self.get_logger().info(f"  Point {i:4d}: x={x:7.3f}, y={y:7.3f}, z={z:7.3f}, dist={distance:6.2f}m")
                        
                except (struct.error, IndexError) as e:
                    continue
            
            # Analyze Z statistics
            if z_values:
                min_z = min(z_values)
                max_z = max(z_values)
                avg_z = sum(z_values) / len(z_values)
                
                self.get_logger().info(f"=== Z-AXIS ANALYSIS ===")
                self.get_logger().info(f"📈 Valid points analyzed: {valid_points}")
                self.get_logger().info(f"📏 Z-range: {min_z:.3f} to {max_z:.3f} meters")
                self.get_logger().info(f"📊 Z-average: {avg_z:.3f} meters")
                self.get_logger().info(f"📐 Z-spread: {max_z - min_z:.3f} meters")
                
                # Check if z-axis has variation
                if abs(max_z - min_z) < 0.001:
                    self.get_logger().warn("⚠️  Z-AXIS PROBLEM: All z-values are nearly identical!")
                    self.get_logger().warn("   This suggests the LiDAR is not detecting height variation")
                elif abs(avg_z) > 100:
                    self.get_logger().warn("⚠️  Z-AXIS PROBLEM: Z-values seem too large (coordinate frame issue?)")
                else:
                    self.get_logger().info("✅ Z-AXIS OK: Good height variation detected")
                    
                # Count points at different height bands
                ground_points = sum(1 for z in z_values if abs(z) < 0.1)
                low_points = sum(1 for z in z_values if 0.1 <= abs(z) < 1.0)
                high_points = sum(1 for z in z_values if abs(z) >= 1.0)
                
                self.get_logger().info(f"🏠 Height distribution:")
                self.get_logger().info(f"   Ground level (|z| < 0.1m): {ground_points} points")
                self.get_logger().info(f"   Low height (0.1-1.0m): {low_points} points") 
                self.get_logger().info(f"   High height (>1.0m): {high_points} points")
                
            else:
                self.get_logger().error("❌ No valid points found!")
        else:
            self.get_logger().error("❌ Missing x, y, or z fields!")
            
        if self.sample_count >= 3:
            self.get_logger().info("🏁 Analysis complete. Shutting down...")
            self.destroy_node()
            rclpy.shutdown()

def main():
    rclpy.init()
    decoder = PointCloudDecoder()
    try:
        rclpy.spin(decoder)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            decoder.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()



