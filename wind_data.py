import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, Float32
import json
import math
import requests
from datetime import datetime, timedelta
import time

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
        self.log_pub = self.create_publisher(String, '/fire_planner_log', qos)

        # Frost API configuration
        self.frost_client_id = "274b14f2-3547-4403-935a-7fc46d15b0a4"  # Replace with your actual client ID
        self.frost_base_url = "https://frost.met.no"
        
        # Cache for nearby stations to avoid repeated lookups
        self.station_cache = {}

        self.get_logger().info("🌪️ Wind Direction Node started with Frost API, waiting for /fire_tasks...")
    
    
    def publish_log(self, text):
        """Publish log message to UI and console"""
        msg = String()
        msg.data = text
        self.log_pub.publish(msg)
        self.get_logger().info(text)

    def find_nearby_stations(self, lat, lon, max_distance_km=50):
        """Find weather stations near the given coordinates"""
        cache_key = f"{lat:.2f},{lon:.2f}"
        if cache_key in self.station_cache:
            return self.station_cache[cache_key]
        
        try:
            # Use sources endpoint to find stations (following official example pattern)
            endpoint = f"{self.frost_base_url}/sources/v0.jsonld"
            parameters = {
                'country': 'NO',
                'types': 'SensorSystem'
            }
            
            # Make request using the exact pattern from the official example
            r = requests.get(endpoint, parameters, auth=(self.frost_client_id, ''))
            
            # Check if the request worked (following official example)
            if r.status_code == 200:
                json_data = r.json()
                self.get_logger().info(f"✅ Data retrieved from frost.met.no!")
            else:
                json_data = r.json()
                self.get_logger().error(f"❌ Error! Returned status code {r.status_code}")
                if 'error' in json_data:
                    self.get_logger().error(f"Message: {json_data['error'].get('message', 'Unknown')}")
                    self.get_logger().error(f"Reason: {json_data['error'].get('reason', 'Unknown')}")
                return []
            
            data = json_data.get('data', [])
            stations = []
            
            for source in data:
                source_id = source.get('id', '')
                name = source.get('name', 'Unknown')
                
                # Extract coordinates if available
                geometry = source.get('geometry')
                if geometry and geometry.get('coordinates'):
                    coords = geometry['coordinates']
                    station_lon, station_lat = coords[0], coords[1]
                    
                    # Calculate approximate distance
                    distance = self.calculate_distance(lat, lon, station_lat, station_lon)
                    
                    if distance <= max_distance_km:
                        stations.append({
                            'id': source_id,
                            'name': name,
                            'lat': station_lat,
                            'lon': station_lon,
                            'distance': distance
                        })
            
            # Sort by distance and limit to reasonable number
            stations.sort(key=lambda x: x['distance'])
            stations = stations[:10]  # Take only 10 closest
            
            # Cache result
            self.station_cache[cache_key] = stations
            
            self.get_logger().info(f"🗺️ Found {len(stations)} stations within {max_distance_km}km of ({lat:.4f}, {lon:.4f})")
            for station in stations[:3]:  # Log first 3
                self.get_logger().info(f"   📍 {station['name']} ({station['id']}) - {station['distance']:.1f}km")
            
            return stations
            
        except Exception as e:
            self.get_logger().error(f"❌ Error finding nearby stations: {e}")
            return []

    def get_wind_data_from_station(self, station_id, hours_back=24):
        """Get recent wind data from a specific station"""
        try:
            # Get time range (last N hours)
            end_time = datetime.utcnow()
            start_time = end_time - timedelta(hours=hours_back)
            
            # Format times for Frost API (following official example)
            start_str = start_time.strftime('%Y-%m-%d')
            end_str = end_time.strftime('%Y-%m-%d')
            
            # Use observations endpoint (following official example pattern)
            endpoint = f"{self.frost_base_url}/observations/v0.jsonld"
            parameters = {
                'sources': station_id,
                'referencetime': f'{start_str}/{end_str}',
                'elements': 'wind_speed,wind_from_direction'
            }
            
            # Issue an HTTP GET request (exact pattern from official example)
            r = requests.get(endpoint, parameters, auth=(self.frost_client_id, ''))
            
            # Check if the request worked (following official example)
            if r.status_code == 200:
                json_data = r.json()
                data = json_data.get('data', [])
                self.get_logger().info(f"✅ Got {len(data)} observations from {station_id}")
            else:
                json_data = r.json()
                self.get_logger().error(f"❌ Error for station {station_id}! Status code {r.status_code}")
                if 'error' in json_data:
                    self.get_logger().error(f"Message: {json_data['error'].get('message', 'Unknown')}")
                return None
            
            if not data:
                return None
            
            # Extract most recent wind data (following data structure from official example)
            latest_wind_direction = None
            latest_wind_speed = None
            latest_time = None
            
            for obs in data:
                obs_time = obs.get('referenceTime')
                for observation in obs.get('observations', []):
                    element_id = observation.get('elementId')
                    value = observation.get('value')
                    
                    if element_id == 'wind_from_direction' and value is not None:
                        if latest_time is None or obs_time > latest_time:
                            latest_wind_direction = float(value)
                            latest_time = obs_time
                    elif element_id == 'wind_speed' and value is not None:
                        latest_wind_speed = float(value)
            
            if latest_wind_direction is not None:
                self.publish_log(f"[Wind] Station {station_id}: Wind direction {latest_wind_direction}° at {latest_time}")
                if latest_wind_speed is not None:
                    self.publish_log(f"[Wind] Wind speed: {latest_wind_speed} m/s")
                return latest_wind_direction
            
            return None
            
        except Exception as e:
            self.get_logger().error(f"❌ Error getting wind data from station {station_id}: {e}")
            return None

    def get_wind_direction_for_location(self, lat, lon):
        """Get wind direction for a specific location using nearby stations"""
        try:
            # Find nearby stations
            stations = self.find_nearby_stations(lat, lon)
            if not stations:
                self.get_logger().warning(f"⚠️ No weather stations found near ({lat:.6f}, {lon:.6f})")
                return None
            
            # Try to get data from stations, starting with the closest
            for station in stations:
                wind_direction = self.get_wind_data_from_station(station['id'])
                if wind_direction is not None:
                    self.publish_log(f"[Wind] Got wind data from {station['name']} ({station['distance']:.1f}km away)")
                    return wind_direction
                else:
                    self.get_logger().info(f"⚠️ No recent wind data from {station['name']}")
            
            # If no data from any station
            self.get_logger().warning(f"❌ No recent wind data available from any nearby stations")
            return None
            
        except Exception as e:
            self.get_logger().error(f"❌ Error getting wind direction: {e}")
            return None

    def calculate_distance(self, lat1, lon1, lat2, lon2):
        """Calculate approximate distance between two points in km"""
        # Simplified distance calculation (good enough for finding nearby stations)
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        # Rough conversion: 1 degree ≈ 111 km
        distance = math.sqrt(dlat**2 + dlon**2) * 111
        return distance

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

            wind_direction = self.get_wind_direction_for_location(fire_lat, fire_lon)
            if wind_direction is not None:
                # Publish wind direction
                self.wind_direction_pub.publish(Float32(data=wind_direction))
                self.get_logger().info(f"✅ Published wind direction for {task_id}: {wind_direction:.1f}°")
            else:
                self.get_logger().warning(f"⚠️ Could not get wind direction for {task_id}")

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