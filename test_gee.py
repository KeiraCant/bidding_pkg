import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
import json
import math
import ee

# Initialize Earth Engine with your project
ee.Initialize(project='wildfirethesis')

# Your known base location (origin for relative offsets)
BASE_LAT = 63.4188137258128
BASE_LON = 10.401553034756608

def offset_to_latlon(base_lat, base_lon, dx_m, dy_m):
    """Convert offset in meters (dx east, dy north) to lat/lon."""
    R = 6378137  # Earth radius in meters
    new_lat = base_lat + (dy_m / R) * (180 / math.pi)
    new_lon = base_lon + (dx_m / (R * math.cos(math.pi * base_lat / 180))) * (180 / math.pi)
    return new_lat, new_lon

class FireDataNode(Node):
    def __init__(self):
        super().__init__('fire_data_from_tasks')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Subscribe to fire tasks
        self.create_subscription(String, '/fire_tasks', self.fire_task_callback, qos)

        # Publisher for fire data
        self.fire_data_pub = self.create_publisher(String, '/fire_data', qos)

        self.get_logger().info("🔥 Fire Data Node started, waiting for /fire_tasks...")

        # ESA WorldCover landcover legend
        self.landcover_map = {
            10: "Tree cover",
            20: "Shrubland",
            30: "Grassland",
            40: "Cropland",
            50: "Built-up",
            60: "Bare / sparse vegetation",
            70: "Snow and ice",
            80: "Permanent water bodies",
            90: "Herbaceous wetland",
            95: "Mangroves",
            100: "Moss and lichen"
        }

        # MODIS vegetation type legend
        self.veg_type_map = {
            0: 'Water',
            1: 'Evergreen Needleleaf Forests',
            2: 'Evergreen Broadleaf Forests',
            3: 'Deciduous Needleleaf Forests',
            4: 'Deciduous Broadleaf Forests',
            5: 'Mixed Forests',
            6: 'Closed Shrublands',
            7: 'Open Shrublands',
            8: 'Woody Savannas',
            9: 'Savannas',
            10: 'Grasslands',
            11: 'Permanent Wetlands',
            12: 'Croplands',
            13: 'Urban and Built-Up',
            14: 'Cropland/Natural Vegetation Mosaics',
            15: 'Permanent Snow and Ice',
            16: 'Barren or Sparsely Vegetated',
            17: 'Unclassified',
            18: 'Fill Value'
        }

        # Load MODIS vegetation image properly (updated dataset ID)
        modis_collection = ee.ImageCollection('MODIS/061/MCD12Q1')
        modis_img = (
            modis_collection
            .filterDate('2021-01-01', '2021-12-31')
            .first()
        )

        if modis_img is None:
            self.get_logger().error("No MODIS image found for 2021 date range.")
            self.modis_veg_img = None
        else:
            self.modis_veg_img = modis_img.select('LC_Type1')

    def query_ee_grid_50m(self, lat, lon):
        try:
            half_size = 25  # meters (half of 50m)
            lat1, lon1 = offset_to_latlon(lat, lon, -half_size, -half_size)
            lat2, lon2 = offset_to_latlon(lat, lon, half_size, half_size)
            region = ee.Geometry.Rectangle([lon1, lat1, lon2, lat2])

            img = ee.Image('ESA/WorldCover/v200/2021').select('Map')

            sampled = img.sample(
                region=region,
                scale=10,
                geometries=True
            )

            features = sampled.getInfo()['features']
            class_counts = {}
            for f in features:
                val = f['properties']['Map']
                class_counts[val] = class_counts.get(val, 0) + 1

            total_pixels = sum(class_counts.values()) or 1
            grid_data = []

            for f in features:
                coords = f['geometry']['coordinates']
                val = f['properties']['Map']
                class_name = self.landcover_map.get(val, f"Unknown({val})")
                pct_coverage = (class_counts[val] / total_pixels) * 100.0
                cell_data = {
                    'lon': coords[0],
                    'lat': coords[1],
                    'class_code': val,
                    'class_name': class_name,
                    'class_pct_coverage': pct_coverage 
                }

                self.get_logger().info(
                    f"Cell data: lon={coords[0]:.6f}, lat={coords[1]:.6f}, "
                    f"class_code={val}, class_name={class_name}, "
                    f"pct_coverage={pct_coverage:.2f}%"
                )

                grid_data.append(cell_data)

            return grid_data
        except Exception as e:
            self.get_logger().error(f"EE 50m grid query failed: {e}")
            return []

    def fire_task_callback(self, msg):
        try:
            fire_task = json.loads(msg.data)
            location = fire_task.get('location')

            # Convert relative location to lat/lon if needed
            if isinstance(location, list) and len(location) >= 2:
                dx_m, dy_m = location[0], location[1]
                lat, lon = offset_to_latlon(BASE_LAT, BASE_LON, dx_m, dy_m)
            elif isinstance(location, dict):
                lat = location.get('lat')
                lon = location.get('lon')
            else:
                self.get_logger().warning(f"Invalid location format: {location}")
                return

            self.get_logger().info(f"📍 Fire task {fire_task.get('task_id')} at {lat:.6f}, {lon:.6f}")

            grid_data = self.query_ee_grid_50m(lat, lon)

            result = {
                'task_id': fire_task.get('task_id'),
                'lat': lat,
                'lon': lon,
                'landcover_grid_50m': grid_data
            }

            result_msg = String()
            result_msg.data = json.dumps(result)
            self.fire_data_pub.publish(result_msg)

            self.get_logger().info(f"✅ Published 50m grid fire data for task {fire_task.get('task_id')}")

        except Exception as e:
            self.get_logger().error(f"Error processing fire task: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = FireDataNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

