import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import random
import math
import heapq
from datetime import datetime
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class FireTaskPublisher(Node):
    def __init__(self):
        super().__init__('fire_task_publisher')
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.publisher_ = self.create_publisher(String, '/fire_tasks', qos)
        self.priority_status_pub = self.create_publisher(String, '/fire_priorities', qos)
        
        self.task_id = 0
        self.fire_queue = []  # Priority queue: (priority_level, fire_id, fire_data)
        self.current_fires = {}
        self.prioritized_fires = {}
        self.completed_fires = set()
        
        self.sim_center_lat = 63.709577
        self.sim_center_lon = 25.125234
        
        self.local_risk_map = [
            {"coords": [5, 8], "type": "hospital_area", "risk_factors": ["hospital", "population"]},
            {"coords": [-3, 12], "type": "suburban", "risk_factors": ["residential", "population"]},
            {"coords": [0, -5], "type": "power_plant", "risk_factors": ["power_plant", "critical"]},
            {"coords": [25, -15], "type": "forest", "risk_factors": ["vegetation", "remote"]},
            {"coords": [-20, 30], "type": "scrubland", "risk_factors": ["vegetation", "dry"]},
            {"coords": [18, 5], "type": "agricultural", "risk_factors": ["vegetation", "population"]},
            {"coords": [15, 25], "type": "grassland", "risk_factors": ["vegetation", "wind"]},
            {"coords": [-25, -20], "type": "industrial", "risk_factors": ["industrial", "chemical"]},
            {"coords": [35, 40], "type": "wildland", "risk_factors": ["vegetation", "remote"]},
        ]
        
        self.land_cover_fire_risk = {
            'forest': 0.8, 'wood': 0.8,
            'scrub': 0.9, 'heath': 0.8, 'scrubland': 0.9,
            'grassland': 0.7, 'meadow': 0.6,
            'farmland': 0.4, 'orchard': 0.5, 'agricultural': 0.4,
            'residential': 0.2, 'commercial': 0.1, 'urban_edge': 0.3, 'suburban': 0.25,
            'industrial': 0.3, 'power_plant': 0.4,
            'wildland': 0.9, 'unknown': 0.5
        }
        
        self.infrastructure_priorities = {
            'hospital': 1.0, 'clinic': 0.9, 'emergency': 1.0,
            'school': 0.8, 'university': 0.8, 'kindergarten': 0.9,
            'fire_station': 0.9, 'police': 0.7,
            'power': 0.9, 'substation': 0.8, 'power_plant': 1.0,
            'water_treatment': 0.8, 'waste_treatment': 0.7,
            'fuel': 0.6, 'industrial': 0.5, 'chemical': 0.8,
            'residential': 0.6, 'nursing_home': 1.0, 'critical': 1.0,
            'population': 0.7, 'infrastructure': 0.6
        }
        
        self.weights = {
            'population': 0.4,
            'vegetation': 0.3,
            'infrastructure': 0.2,
            'fire_intensity': 0.1
        }
        
        self.fire_timer = self.create_timer(30.0, self.process_fire_tasks)
        self.subscription = self.create_subscription(String, '/task_done', self.on_task_done, qos)

        self.get_logger().info("🔥 Fire Task Publisher with Priority Queue started")
        self.get_logger().info(f"📍 Location: {self.sim_center_lat}, {self.sim_center_lon}")

    def process_fire_tasks(self):
        self.detect_and_prioritize_fire()
        
        # Publish highest-priority task from queue
        if self.fire_queue:
            # Sort by priority_level (lower = higher priority)
            heapq.heapify(self.fire_queue)
            priority_level, fire_id, fire_data = heapq.heappop(self.fire_queue)
            lat, lon = self.local_to_gps(fire_data['location'][0], fire_data['location'][1])
            self.prioritize_fire(fire_id, fire_data, lat, lon)

    def detect_and_prioritize_fire(self):
        self.task_id += 1
        fire_id = f'fire_{self.task_id}'
        
        location = [
            random.uniform(-50, 50),
            random.uniform(-50, 50),
            random.uniform(0, 50)
        ]
        
        nearest_area = self._find_nearest_area(location[0], location[1])
        location_type = nearest_area["type"]
        risk_factors = nearest_area["risk_factors"]
        
        fire_characteristics = self._generate_fire_characteristics(location_type, risk_factors)
        
        fire_data = {
            'fire_id': fire_id,
            'location': location,
            'detection_time': datetime.now().isoformat(),
            'location_type': location_type,
            'risk_factors': risk_factors,
            **fire_characteristics
        }
        
        # Temporary prioritization to get priority_level for queue
        temp_priority = self._compute_priority(fire_data)
        heapq.heappush(self.fire_queue, (temp_priority['priority_level'], fire_id, fire_data))
        self.current_fires[fire_id] = fire_data
        
        self.get_logger().info(
            f"🔥 FIRE DETECTED: {fire_id} at {location_type} "
            f"location - Confidence: {fire_characteristics['confidence']}%, "
            f"Brightness: {fire_characteristics['brightness']}K, "
            f"Priority: {temp_priority['priority_name']}"
        )

    def _compute_priority(self, fire_data):
        location_type = fire_data.get('location_type', 'unknown')
        risk_factors = fire_data.get('risk_factors', [])
        
        population_score = 0.1
        for factor in risk_factors:
            if factor in ['population', 'hospital', 'residential']:
                if location_type in ['hospital_area']:
                    population_score = 1.0
                elif location_type in ['urban_edge', 'suburban']:
                    population_score = 0.8
                else:
                    population_score = 0.6
                break
        
        vegetation_score = self.land_cover_fire_risk.get(location_type, 0.5)
        
        infrastructure_score = 0.0
        for factor in risk_factors:
            if factor in self.infrastructure_priorities:
                infrastructure_score = max(infrastructure_score, 
                                         self.infrastructure_priorities[factor])
        
        fire_intensity_score = self.get_fire_intensity_score(fire_data)
        weather_modifier = 1.2
        
        vegetation_score = min(1.0, vegetation_score * weather_modifier)
        
        priority_score = (
            population_score * self.weights['population'] +
            vegetation_score * self.weights['vegetation'] +
            infrastructure_score * self.weights['infrastructure'] +
            fire_intensity_score * self.weights['fire_intensity']
        ) * 100
        
        if priority_score >= 80:
            priority_level, priority_name = 1, "CRITICAL"
        elif priority_score >= 60:
            priority_level, priority_name = 2, "HIGH"
        elif priority_score >= 40:
            priority_level, priority_name = 3, "MEDIUM"
        elif priority_score >= 20:
            priority_level, priority_name = 4, "LOW"
        else:
            priority_level, priority_name = 5, "VERY_LOW"
        
        return {
            'priority_level': priority_level,
            'priority_name': priority_name,
            'priority_score': priority_score,
            'factors': {
                'population': population_score,
                'vegetation': vegetation_score,
                'infrastructure': infrastructure_score,
                'fire_intensity': fire_intensity_score,
                'weather_modifier': weather_modifier
            }
        }

    def _find_nearest_area(self, x, y):
        min_distance = float('inf')
        nearest_area = None
        
        for area in self.local_risk_map:
            ax, ay = area["coords"]
            distance = math.sqrt((x - ax)**2 + (y - ay)**2)
            if distance < min_distance:
                min_distance = distance
                nearest_area = area
        
        return nearest_area or {"type": "unknown", "risk_factors": []}

    def _generate_fire_characteristics(self, location_type, risk_factors):
        if "population" in risk_factors:
            confidence = random.randint(85, 98)
        elif "infrastructure" in risk_factors:
            confidence = random.randint(80, 95)
        else:
            confidence = random.randint(60, 85)
        
        if location_type in ["forest", "scrubland", "grassland"]:
            brightness = random.randint(320, 450)
        elif location_type in ["industrial", "power_plant"]:
            brightness = random.randint(400, 550)
        else:
            brightness = random.randint(300, 380)
        
        if location_type in ["forest", "industrial", "power_plant"]:
            frp = random.uniform(20, 100)
        else:
            frp = random.uniform(5, 40)
        
        return {
            'confidence': confidence,
            'brightness': brightness,
            'frp': frp,
            'satellite': random.choice(['MODIS_Terra', 'MODIS_Aqua', 'VIIRS_NPP']),
            'scan_angle': random.uniform(-45, 45),
            'acq_time': f"{random.randint(0,23):02d}{random.randint(0,59):02d}"
        }

    def prioritize_fire(self, fire_id, fire_data, lat, lon):
        self.get_logger().info(f"🔍 Prioritizing {fire_id} at ({lat:.4f}, {lon:.4f})")
        self._local_prioritize_fire(fire_id, fire_data, lat, lon)

    def _local_prioritize_fire(self, fire_id, fire_data, lat, lon):
        priority = self._compute_priority(fire_data)
        self._finalize_priority(fire_id, fire_data, lat, lon, priority['factors'])

    def _finalize_priority(self, fire_id, fire_data, lat, lon, scores):
        priority_score = (
            scores['population'] * self.weights['population'] +
            scores['vegetation'] * self.weights['vegetation'] +
            scores['infrastructure'] * self.weights['infrastructure'] +
            scores['fire_intensity'] * self.weights['fire_intensity']
        ) * 100
        
        if priority_score >= 80:
            priority_level, priority_name = 1, "CRITICAL"
        elif priority_score >= 60:
            priority_level, priority_name = 2, "HIGH"
        elif priority_score >= 40:
            priority_level, priority_name = 3, "MEDIUM"
        elif priority_score >= 20:
            priority_level, priority_name = 4, "LOW"
        else:
            priority_level, priority_name = 5, "VERY_LOW"
        
        priority_data = {
            'fire_id': fire_id,
            'priority_level': priority_level,
            'priority_name': priority_name,
            'priority_score': priority_score,
            'factors': scores,
            'location': fire_data['location'],
            'gps_coords': [lat, lon],
            'timestamp': datetime.now().isoformat(),
            'location_type': fire_data.get('location_type', 'unknown'),
            'risk_factors': fire_data.get('risk_factors', [])
        }
        
        self.prioritized_fires[fire_id] = priority_data
        
        self.get_logger().info(
            f"📊 {fire_id} PRIORITY: {priority_name} (score: {priority_score:.1f}) - "
            f"Type: {fire_data.get('location_type', 'unknown')} - "
            f"Pop: {scores['population']:.2f}, Veg: {scores['vegetation']:.2f}, "
            f"Infra: {scores['infrastructure']:.2f}"
        )
        
        fire_task = {
            'task_id': fire_id,
            'location': fire_data['location'],
            'priority': priority_level,
            'priority_name': priority_name,
            'priority_score': priority_score
        }
        
        msg = String(data=json.dumps(fire_task))
        self.publisher_.publish(msg)
        self.get_logger().info(f"🔥 Published fire task: {fire_task}")
        
        self.publish_priority_status()

    def get_vegetation_risk(self, fire_data):
        location_type = fire_data.get('location_type', 'unknown')
        return self.land_cover_fire_risk.get(location_type, 0.5)

    def get_infrastructure_risk(self, fire_data):
        risk_factors = fire_data.get('risk_factors', [])
        max_risk = 0.0
        for factor in risk_factors:
            if factor in self.infrastructure_priorities:
                risk = self.infrastructure_priorities[factor]
                max_risk = max(max_risk, risk)
        return max_risk

    def get_fire_intensity_score(self, fire_data):
        confidence = fire_data.get('confidence', 70)
        brightness = fire_data.get('brightness', 300)
        frp = fire_data.get('frp', 20)
        
        confidence_norm = confidence / 100.0
        brightness_norm = min(brightness, 500) / 500.0
        frp_norm = min(frp, 100) / 100.0
        
        intensity_score = (confidence_norm * 0.4 + brightness_norm * 0.4 + frp_norm * 0.2)
        return min(1.0, intensity_score)

    def publish_priority_status(self):
        status_data = {
            'timestamp': datetime.now().isoformat(),
            'total_fires': len(self.prioritized_fires),
            'fires_by_priority': {},
            'pending_fires': len(self.prioritized_fires) - len(self.completed_fires),
            'queued_fires': len(self.fire_queue)
        }
        
        for priority_level in range(1, 6):
            fires_at_level = [
                f for f in self.prioritized_fires.values()
                if f['priority_level'] == priority_level
            ]
            status_data['fires_by_priority'][priority_level] = len(fires_at_level)
        
        self.priority_status_pub.publish(String(data=json.dumps(status_data)))

    def on_task_done(self, msg):
        try:
            if msg.data.startswith('{'):
                data = json.loads(msg.data)
                task_id = data['task_id']
                drone_id = data['drone_id']
            else:
                parts = msg.data.split(' ')
                task_id = parts[0] if len(parts) > 0 else 'unknown'
                drone_id = parts[3] if len(parts) > 3 else 'unknown'
            
            self.get_logger().info(f"✅ Task completed: {msg.data}")
            
            self.completed_fires.add(task_id)
            
            if task_id in self.current_fires:
                fire_info = self.current_fires[task_id]
                if task_id in self.prioritized_fires:
                    priority_info = self.prioritized_fires[task_id]
                    self.get_logger().info(
                        f"🚒 {priority_info['priority_name']} priority fire at "
                        f"{fire_info['location_type']} extinguished by {drone_id}"
                    )
                del self.current_fires[task_id]
            
        except Exception as e:
            self.get_logger().error(f"Error processing task completion: {e}")

    def local_to_gps(self, x, y):
        lat = self.sim_center_lat + (y / 110540)
        lon = self.sim_center_lon + (x / (111320 * math.cos(math.radians(lat))))
        return lat, lon

def main(args=None):
    rclpy.init(args=args)
    
    fire_publisher = FireTaskPublisher()
    
    try:
        rclpy.spin(fire_publisher)
    except KeyboardInterrupt:
        fire_publisher.get_logger().info("🔥 Fire Task Publisher shutting down...")
    finally:
        fire_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
