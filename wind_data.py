import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, Float32
import json
import math
import ee
from datetime import datetime, timedelta

# Initialize Earth Engine with your project
ee.Initialize(project='wildfirethesis')

class WindDataNode(Node):
    def __init__(self):
        super().__init__('wind_direction_node')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Subscribe to fire tasks
        self.create_subscription(String, '/fire_tasks', self.fire_task_callback, qos)

        # Publisher for wind direction (in degrees)
        self.wind_direction_pub = self.create_publisher(Float32, '/wind_direction', qos)

        self.get_logger().info("🌪️ Wind Direction Node started, waiting for /fire_tasks...")

    def get_latest_gfs_data(self):
        """Get the most recent GFS forecast data available"""
        try:
            # Get current date and go back a few days to ensure data availability
            end_date = datetime.utcnow()
            start_date = end_date - timedelta(days=3)
            
            # Format dates for Earth Engine
            start_str = start_date.strftime('%Y-%m-%d')
            end_str = end_date.strftime('%Y-%m-%d')
            
            # Load GFS collection and get the most recent forecast
            gfs_collection = ee.ImageCollection('NOAA/GFS0P25')
            recent_forecasts = gfs_collection.filterDate(start_str, end_str).sort('system:time_start', False)
            latest_forecast = recent_forecasts.first()
            
            if latest_forecast is None:
                self.get_logger().error("No GFS forecast data found in recent days")
                return None
                
            forecast_time = ee.Date(latest_forecast.get('system:time_start'))
            forecast_time_str = forecast_time.format('YYYY-MM-dd HH:mm:ss').getInfo()
            self.get_logger().info(f"🌪️ Using GFS forecast from: {forecast_time_str}")
            
            return latest_forecast
            
        except Exception as e:
            self.get_logger().error(f"❌ Error getting GFS data: {e}")
            return None

    def query_wind_data_grid(self, fire_lat, fire_lon):
        """Query wind data around the fire location to calculate wind direction"""
        try:
            # Get the latest GFS forecast
            gfs_image = self.get_latest_gfs_data()
            if gfs_image is None:
                return None

            # Create a region around the fire location
            half_box_degrees = 0.15  # ~16.7km radius
            region = ee.Geometry.Rectangle([
                fire_lon - half_box_degrees,
                fire_lat - half_box_degrees,
                fire_lon + half_box_degrees,
                fire_lat + half_box_degrees
            ])

            # Select wind components
            wind_bands = gfs_image.select([
                'u_component_of_wind_10m_above_ground',
                'v_component_of_wind_10m_above_ground'
            ])

            # Sample wind data
            sampled = wind_bands.sample(
                region=region,
                scale=28000,  # GFS resolution (~28km)
                geometries=True,
                numPixels=25,  # 5x5 grid
                seed=42
            )
            
            features = sampled.getInfo()['features']
            if not features:
                self.get_logger().warning(f"⚠️ No wind data found around ({fire_lat:.6f}, {fire_lon:.6f})")
                return None

            wind_directions = []
            for f in features:
                try:
                    props = f['properties']
                    u_wind = props.get('u_component_of_wind_10m_above_ground', 0)
                    v_wind = props.get('v_component_of_wind_10m_above_ground', 0)
                    
                    if u_wind is None or v_wind is None:
                        continue
                    
                    # Calculate wind direction (meteorological convention: direction wind is coming FROM)
                    wind_direction_rad = math.atan2(-u_wind, -v_wind)
                    wind_direction_deg = (wind_direction_rad * 180 / math.pi) % 360
                    wind_directions.append(wind_direction_deg)
                    
                except Exception as e:
                    self.get_logger().warning(f"⚠️ Error processing wind sample: {e}")
                    continue

            if not wind_directions:
                self.get_logger().warning(f"❌ No valid wind direction data for ({fire_lat:.6f}, {fire_lon:.6f})")
                return None

            # Calculate average wind direction
            avg_direction = self.calculate_average_wind_direction(wind_directions)
            self.get_logger().info(f"🌪️ Average wind direction: {avg_direction:.1f}°")
            return avg_direction

        except Exception as e:
            self.get_logger().error(f"❌ Earth Engine wind query failed: {e}")
            return None

    def calculate_average_wind_direction(self, wind_directions):
        """Calculate vector average of wind directions"""
        try:
            sum_x = sum(math.cos(math.radians(deg)) for deg in wind_directions)
            sum_y = sum(math.sin(math.radians(deg)) for deg in wind_directions)
            avg_direction_rad = math.atan2(sum_y, sum_x)
            avg_direction_deg = (avg_direction_rad * 180 / math.pi) % 360
            return avg_direction_deg
        except Exception as e:
            self.get_logger().error(f"❌ Error calculating average wind direction: {e}")
            return 0.0

    def fire_task_callback(self, msg):
        try:
            fire_task = json.loads(msg.data)
            task_id = fire_task.get('task_id')
            location = fire_task.get('location')

            if isinstance(location, list) and len(location) >= 2:
                fire_lat, fire_lon = location[0], location[1]
            elif isinstance(location, dict):
                fire_lat = location.get('lat')
                fire_lon = location.get('lon')
            else:
                self.get_logger().warning(f"❌ Invalid location format: {location}")
                return

            self.get_logger().info(f"🌪️ Processing wind direction for {task_id} at ({fire_lat:.6f}, {fire_lon:.6f})")

            avg_wind_direction = self.query_wind_data_grid(fire_lat, fire_lon)
            if avg_wind_direction is not None:
                # Publish wind direction
                self.wind_direction_pub.publish(Float32(data=avg_wind_direction))
                self.get_logger().info(f"✅ Published wind direction for {task_id}: {avg_wind_direction:.1f}°")

        except Exception as e:
            self.get_logger().error(f"❌ Error processing fire task: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = WindDataNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()