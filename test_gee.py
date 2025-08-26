import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
import json
import math
import ee
import os
import csv

# Initialize Earth Engine with your project
ee.Initialize(project='wildfirethesis')

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

        # Publisher for fire priority
        self.fire_priority_pub = self.create_publisher(String, '/fire_priority', qos)

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

        # CSV file for raw fire data - CREATE NEW FILE EACH TIME NODE STARTS
        self.raw_csv_path = os.path.expanduser('~/fire_data_raw.csv')
        
        # Always create a fresh CSV file with headers (overwrites existing file)
        with open(self.raw_csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['task_id','lat','lon','class_code','class_name','class_pct_coverage'])
        
        self.get_logger().info(f"📄 Created fresh CSV file: {self.raw_csv_path}")

    def query_ee_grid_50m(self, fire_lat, fire_lon):
        try:
            half_box_degrees = 0.00045
            region = ee.Geometry.Rectangle([
                fire_lon - half_box_degrees,
                fire_lat - half_box_degrees,
                fire_lon + half_box_degrees,
                fire_lat + half_box_degrees
            ])

            img = ee.Image('ESA/WorldCover/v200/2021').select('Map')
            sampled = img.sample(region=region, scale=10, geometries=True)
            features = sampled.getInfo()['features']

            if not features:
                self.get_logger().warning(f"⚠️ No landcover data found at ({fire_lat:.6f}, {fire_lon:.6f})")
                return []

            class_counts = {}
            for f in features:
                val = f['properties']['Map']
                class_counts[val] = class_counts.get(val, 0) + 1

            total_pixels = sum(class_counts.values())
            grid_data = []
            for f in features:
                coords = f['geometry']['coordinates']
                landcover_class = f['properties']['Map']
                class_name = self.landcover_map.get(landcover_class, f"Unknown({landcover_class})")
                pct_coverage = (class_counts[landcover_class] / total_pixels) * 100.0
                grid_data.append({
                    'lat': coords[1],
                    'lon': coords[0],
                    'class_code': landcover_class,
                    'class_name': class_name,
                    'class_pct_coverage': pct_coverage
                })
            self.get_logger().info(f"📊 Sampled {len(grid_data)} points in 50x50m around fire location")
            return grid_data

        except Exception as e:
            self.get_logger().error(f"❌ Earth Engine query failed: {e}")
            return []

    def calculate_priority(self, grid_data):
        try:
            urban_pixels = sum(1 for cell in grid_data if cell['class_code'] == 50)
            total_pixels = len(grid_data)
            priority = urban_pixels / total_pixels if total_pixels > 0 else 0.0
            return priority
        except Exception as e:
            self.get_logger().error(f"❌ Error calculating priority: {e}")
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

            self.get_logger().info(f"🔥 Processing {task_id} at ({fire_lat:.6f}, {fire_lon:.6f})")

            grid_data = self.query_ee_grid_50m(fire_lat, fire_lon)
            if not grid_data:
                self.get_logger().warning(f"⚠️ No landcover data for {task_id}")
                return

            result = {
                'task_id': task_id,
                'lat': fire_lat,
                'lon': fire_lon,
                'landcover_grid_50m': grid_data
            }

            self.fire_data_pub.publish(String(data=json.dumps(result)))
            self.get_logger().info(f"✅ Published landcover data for {task_id}: {len(grid_data)} sample points")

            # Publish priority as before
            priority = self.calculate_priority(grid_data)
            priority_msg = {'task_id': task_id, 'priority': priority}
            self.fire_priority_pub.publish(String(data=json.dumps(priority_msg)))
            self.get_logger().info(f"📊 Priority for {task_id}: {priority:.3f}")

            # Write raw data to CSV (append mode since we created fresh file at startup)
            try:
                with open(self.raw_csv_path, 'a', newline='') as f:
                    writer = csv.writer(f)
                    for cell in grid_data:
                        writer.writerow([
                            task_id,
                            cell['lat'],
                            cell['lon'],
                            cell['class_code'],
                            cell['class_name'],
                            cell['class_pct_coverage']
                        ])
                self.get_logger().info(f"📝 Raw fire data for {task_id} written to {self.raw_csv_path}")
            except Exception as e:
                self.get_logger().error(f"❌ Error writing raw CSV: {e}")

        except Exception as e:
            self.get_logger().error(f"❌ Error processing fire task: {e}")

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
