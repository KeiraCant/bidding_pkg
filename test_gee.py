import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
import json
import math
import ee
import os
import csv
import rasterio
import numpy as np
from rasterio.windows import from_bounds
from scipy.stats import mode

try:
    ee.Initialize(project='wildfirethesis')
    EE_AVAILABLE = True
except Exception as e:
    print(f"Warning: Earth Engine initialization failed: {e}")
    EE_AVAILABLE = False

class FireDataNode(Node):
    def __init__(self):
        super().__init__('fire_data_from_tasks')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        self.create_subscription(String, '/fire_tasks', self.fire_task_callback, qos)
        self.fire_data_pub = self.create_publisher(String, '/fire_data', qos)
        self.fire_priority_pub = self.create_publisher(String, '/fire_priority', qos)
        self.get_logger().info(" Fire Data Node started, waiting for /fire_tasks...")

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

        self.local_tiff_path = os.path.expanduser(
            '/home/keira/ros2_ws/src/bidding_pkg/bidding_pkg/ESA_WorldCover_10m_2021_v200_N63E009_Map.tif'
        )
        self.local_tiff_available = False
        if os.path.exists(self.local_tiff_path):
            try:
                with rasterio.open(self.local_tiff_path) as src:
                    self.local_bounds = src.bounds
                    self.local_transform = src.transform
                    self.local_crs = src.crs
                    self.get_logger().info(f" Local TIFF loaded: {self.local_tiff_path}")
                    self.get_logger().info(f"Bounds: {self.local_bounds}")
                    self.local_tiff_available = True
            except Exception as e:
                self.get_logger().error(f"❌ Error loading local TIFF: {e}")
        else:
            self.get_logger().warning(f"⚠️ Local TIFF not found at: {self.local_tiff_path}")

        self.raw_csv_path = os.path.expanduser('~/fire_data_raw.csv')
        with open(self.raw_csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['task_id','lat','lon','class_code','class_name','class_pct_coverage','source'])
        self.get_logger().info(f" Created fresh CSV file: {self.raw_csv_path}")

    def is_point_in_local_bounds(self, lat, lon):
        if not self.local_tiff_available:
            return False
        return (self.local_bounds.left <= lon <= self.local_bounds.right and 
                self.local_bounds.bottom <= lat <= self.local_bounds.top)

    def query_local_tiff_grid_50m(self, fire_lat, fire_lon):
        """
        Samples a 1km x 1km area (±500m) around fire location.
        Aggregates 10m pixels into dominant landcover classes per 200m x 200m grid cell.
        """
        try:
            # Convert 500m to degrees latitude and longitude
            meters_to_degrees_lat = 500.0 / 111000.0
            meters_to_degrees_lon = 500.0 / (111000.0 * math.cos(math.radians(fire_lat)))

            min_lon = fire_lon - meters_to_degrees_lon
            max_lon = fire_lon + meters_to_degrees_lon
            min_lat = fire_lat - meters_to_degrees_lat
            max_lat = fire_lat + meters_to_degrees_lat

            with rasterio.open(self.local_tiff_path) as src:
                window = from_bounds(min_lon, min_lat, max_lon, max_lat, src.transform)
                data = src.read(1, window=window)

                if data.size == 0:
                    self.get_logger().warning(f"⚠️ No data in local TIFF for location ({fire_lat:.6f}, {fire_lon:.6f})")
                    return []

                window_transform = rasterio.windows.transform(window, src.transform)

                rows, cols = data.shape
                block_size = 20  # 20 pixels * 10m = 200m grid cells
                class_counts = {}
                grid_data = []

                for row_block in range(0, rows, block_size):
                    for col_block in range(0, cols, block_size):
                        block = data[row_block:min(row_block+block_size, rows),
                                     col_block:min(col_block+block_size, cols)]

                        block_valid = block[(block != 0) & (block != 255)]
                        if block_valid.size == 0:
                            continue

                        mode_class = int(mode(block_valid, axis=None).mode[0])

                        center_row = row_block + block.shape[0] // 2
                        center_col = col_block + block.shape[1] // 2
                        lon_pixel, lat_pixel = rasterio.transform.xy(window_transform, center_row, center_col)

                        class_counts[mode_class] = class_counts.get(mode_class, 0) + 1
                        class_name = self.landcover_map.get(mode_class, f"Unknown({mode_class})")

                        grid_data.append({
                            'lat': lat_pixel,
                            'lon': lon_pixel,
                            'class_code': mode_class,
                            'class_name': class_name,
                            'class_pct_coverage': 0.0
                        })

                total_blocks = sum(class_counts.values())
                if total_blocks > 0:
                    for cell in grid_data:
                        cell['class_pct_coverage'] = (class_counts[cell['class_code']] / total_blocks) * 100.0

                self.get_logger().info(f" LOCAL: Aggregated {len(grid_data)} blocks (200x200m) around fire location")
                return grid_data

        except Exception as e:
            self.get_logger().error(f"❌ Local TIFF 200m aggregation query failed: {e}")
            return []

    def query_ee_grid_50m(self, fire_lat, fire_lon):
        """
        Query Earth Engine for 1km x 1km area, aggregated into 200m grid cells.
        """
        if not EE_AVAILABLE:
            self.get_logger().error("❌ Earth Engine not available for fallback")
            return []

        try:
            meters_to_degrees_lat = 400.0 / 111000.0
            meters_to_degrees_lon = 400.0 / (111000.0 * math.cos(math.radians(fire_lat)))

            region = ee.Geometry.Rectangle([
            fire_lon - meters_to_degrees_lon,
            fire_lat - meters_to_degrees_lat,
            fire_lon + meters_to_degrees_lon,
            fire_lat + meters_to_degrees_lat
            ])


            img = ee.Image('ESA/WorldCover/v200/2021').select('Map')

            aggregated_img = img.reduceResolution(
                reducer=ee.Reducer.mode(),
                maxPixels=500
            ).reproject(
                crs=img.projection().crs(),
                scale=200
            )

            sampled = aggregated_img.sample(region=region, scale=200, geometries=True)
            features = sampled.getInfo().get('features', [])
            if not features:
                self.get_logger().warning(f"⚠️ No landcover data found at ({fire_lat:.6f}, {fire_lon:.6f})")
                return []

            class_counts = {}
            grid_data = []
            for feature in features:
                val = feature['properties']['Map']
                class_counts[val] = class_counts.get(val, 0) + 1

            total_pixels = sum(class_counts.values())

            for feature in features:
                coords = feature['geometry']['coordinates']
                landcover_class = feature['properties']['Map']
                class_name = self.landcover_map.get(landcover_class, f"Unknown({landcover_class})")
                pct_coverage = (class_counts[landcover_class] / total_pixels) * 100.0
                grid_data.append({
                    'lat': coords[1],
                    'lon': coords[0],
                    'class_code': landcover_class,
                    'class_name': class_name,
                    'class_pct_coverage': pct_coverage,
                    'source': "earth_engine"
                })

            self.get_logger().info(f" EARTH ENGINE: Sampled {len(grid_data)} aggregated 200x200m points around fire location")
            return grid_data

        except Exception as e:
            self.get_logger().error(f"❌ Earth Engine query failed: {e}")
            return []

    def query_landcover_data(self, fire_lat, fire_lon):
        grid_data = []
        data_source = "unknown"
        if EE_AVAILABLE:
            self.get_logger().info(f"🌐 Using Earth Engine for ({fire_lat:.6f}, {fire_lon:.6f})")
            grid_data = self.query_ee_grid_50m(fire_lat, fire_lon)
            data_source = "earth_engine"

        if not grid_data and self.local_tiff_available and self.is_point_in_local_bounds(fire_lat, fire_lon):
            self.get_logger().info(f" Falling back to LOCAL TIFF 200m grid for ({fire_lat:.6f}, {fire_lon:.6f})")
            grid_data = self.query_local_tiff_grid_50m(fire_lat, fire_lon)
            data_source = "local_tiff"

        for cell in grid_data:
            cell['source'] = data_source

        return grid_data

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

            self.get_logger().info(f" Processing {task_id} at ({fire_lat:.6f}, {fire_lon:.6f})")
            grid_data = self.query_landcover_data(fire_lat, fire_lon)
            if not grid_data:
                self.get_logger().warning(f"⚠️ No landcover data for {task_id}")
                return

            result = {
                'task_id': task_id,
                'lat': fire_lat,
                'lon': fire_lon,
                'landcover_grid_50m': grid_data,
                'data_source': grid_data[0]['source'] if grid_data else 'unknown'
            }

            self.fire_data_pub.publish(String(data=json.dumps(result)))
            self.get_logger().info(f" Published landcover data for {task_id}: {len(grid_data)} sample points from {grid_data[0]['source'] if grid_data else 'unknown'}")

            priority = self.calculate_priority(grid_data)
            priority_msg = {'task_id': task_id, 'priority': priority}
            self.fire_priority_pub.publish(String(data=json.dumps(priority_msg)))
            self.get_logger().info(f" Priority for {task_id}: {priority:.3f}")

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
                            cell['class_pct_coverage'],
                            cell['source']
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
