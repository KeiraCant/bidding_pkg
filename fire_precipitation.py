import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
import json
import requests
from datetime import datetime, timedelta
import math
import pandas as pd

class FirePrecipNode(Node):
    def __init__(self):
        super().__init__('fire_precip_node')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        
        # Subscribe to fire tasks
        self.create_subscription(String, '/fire_tasks', self.fire_task_callback, qos)
        
        # Publisher for fire precipitation data
        self.fire_precip_pub = self.create_publisher(String, '/fire_precip', qos)
        self.log_pub = self.create_publisher(String, '/fire_planner_log', qos)

        # FROST API endpoints
        self.frost_sources_endpoint = 'https://frost.met.no/sources/v0.jsonld'
        self.frost_obs_endpoint = 'https://frost.met.no/observations/v0.jsonld'
        self.client_id = '274b14f2-3547-4403-935a-7fc46d15b0a4'  # Replace with your actual Frost API key

        self.get_logger().info("🌧️ Fire Precip Node started with FROST API, waiting for /fire_tasks...")
    def publish_log(self, text):
        """Publish log message to UI and console"""
        msg = String()
        msg.data = text
        self.log_pub.publish(msg)
        self.get_logger().info(text)
        
    def haversine_distance(self, lat1, lon1, lat2, lon2):
        R = 6371  # Earth radius in km
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        return 2 * R * math.asin(math.sqrt(a))

    def find_nearest_stations(self, fire_lat, fire_lon, max_distance_km=50):
        try:
            response = requests.get(self.frost_sources_endpoint, params={"types": "SensorSystem"}, auth=(self.client_id, ''))
            response.raise_for_status()
            stations = []
            for source in response.json().get('data', []):
                coords = source.get('geometry', {}).get('coordinates')
                if coords:
                    station_lon, station_lat = coords[0], coords[1]
                    distance = self.haversine_distance(fire_lat, fire_lon, station_lat, station_lon)
                    if distance <= max_distance_km:
                        stations.append({
                            'id': source['id'],
                            'name': source.get('name', 'Unknown'),
                            'lat': station_lat,
                            'lon': station_lon,
                            'distance': distance
                        })
            stations.sort(key=lambda x: x['distance'])
            self.get_logger().info(f"🔍 Found {len(stations)} stations within {max_distance_km} km")
            return stations[:5]
        except Exception as e:
            self.get_logger().error(f"❌ Error finding stations: {e}")
            return []

    def query_station_precip_1day(self, station_id, date):
        """Query sum(precipitation_amount P1D) for a single day directly from /observations"""
        date_str = date.strftime('%Y-%m-%d')
        params = {
            'sources': station_id,
            'elements': 'sum(precipitation_amount P1D)',
            'referencetime': f'{date_str}/{date_str}',
            'fields': 'sourceId,referenceTime,elementId,value,unit'
        }
        try:
            r = requests.get(self.frost_obs_endpoint, params=params, auth=(self.client_id, ''))
            if r.status_code != 200:
                self.get_logger().warning(f"⚠️ Observation query failed for {station_id}: {r.status_code}")
                return 0.0
            data = r.json().get('data', [])
            rows = []
            for d in data:
                df = pd.DataFrame(d['observations'])
                rows.append(df)
            if rows:
                df = pd.concat(rows, ignore_index=True)
                df_precip = df[df['elementId'] == 'sum(precipitation_amount P1D)']
                return df_precip['value'].sum()
            else:
                return 0.0
        except Exception as e:
            self.get_logger().error(f"❌ Error querying precipitation for {station_id}: {e}")
            return 0.0

    def query_station_precip_7days(self, station_id):
        """Query sum(precipitation_amount P1D) for the last 7 days for a station"""
        total_precip = 0.0
        today = datetime.utcnow().date()
        
        for days_ago in range(7):
            date = today - timedelta(days=days_ago)
            total_precip += self.query_station_precip_1day(station_id, date)
        
        return total_precip

    def query_precip_7_days(self, fire_lat, fire_lon):
        """Main 7-day precipitation query"""
        stations = self.find_nearest_stations(fire_lat, fire_lon, max_distance_km=100)
        if not stations:
            return 0.0, "no_stations"

        for station in stations:
            self.publish_log(f"[Rain]  Querying station {station['name']} ({station['distance']:.1f} km away)")
            precip = self.query_station_precip_7days(station['id'])
            if precip > 0:
                return precip, f"FROST_{station['id']}"

        return 0.0, "no_data"

    def fire_task_callback(self, msg):
        try:
            fire_task = json.loads(msg.data)
            task_id = fire_task.get('task_id')
            location = fire_task.get('location')
            if isinstance(location, list) and len(location) >= 2:
                fire_lat, fire_lon = location[0], location[1]
            else:
                self.get_logger().warning(f"❌ Invalid location format: {location}")
                return

            self.get_logger().info(f"🌍 Checking 7-day precip for {task_id} at ({fire_lat:.6f}, {fire_lon:.6f})")
            precip_mm, data_source = self.query_precip_7_days(fire_lat, fire_lon)

            result = {
                'task_id': task_id,
                'lat': fire_lat,
                'lon': fire_lon,
                'precip_7day_mm': precip_mm,
                'timestamp': datetime.utcnow().isoformat(),
                'data_source': data_source,
                'period_days': 7
            }
            self.fire_precip_pub.publish(String(data=json.dumps(result)))
            self.publish_log(f"[Rain] Published 7-day precip for {task_id}: {precip_mm:.2f} mm")
        except Exception as e:
            self.get_logger().error(f"❌ Error processing fire task: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = FirePrecipNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
