# Wildfire_BiddingPkg_LaunchFile
This is a follow on from bidding_pkg which contain the launch files as well as setup.py

When run the launch file will first ask how many drones you would like, what type of path planner will be used, the level of autonomy as well as weather conditions. 


#  Fire Response System

An interactive ROS2-based drone coordination system for autonomous fire response missions with real-time environmental data integration.

## Overview

This system provides a configurable launch interface for deploying multiple drones to respond to fire incidents. It features dynamic task allocation, intelligent path planning with environmental awareness, and real-time monitoring through an interactive terminal UI.

## Features

- **Interactive Configuration UI**: Tkinter-based setup interface for mission parameters
- **Multi-Drone Coordination**: Support for 1-10 drones with dynamic task allocation
- **Intelligent Path Planning**: Multiple algorithms including multi-waypoint, sector-based, dynamic clustering, and priority sweep
- **Environmental Integration**: Real-time weather data and wind conditions via Google Earth Engine
- **Flexible Control Modes**: Autonomous or pilot-assisted operation
- **Real-Time Visualization**: Terminal-based UI for mission monitoring

## System Architecture

### Core Components

1. **Fire Task Publisher**: Generates and publishes fire incident tasks
2. **Allocator Node**: Distributes tasks to drones using bidding algorithms
3. **Task Visualiser**: Provides visual feedback of task assignments
4. **Environmental Nodes**:
   - `test_gee`: Google Earth Engine integration
   - `wind_data`: Real-time wind information
   - `fire_precipitation`: Precipitation data collection

### Per-Drone Components

Each drone spawns three nodes:
- **Bidding Node**: Evaluates and bids on available tasks
- **Path Planner**: Generates optimal routes based on selected algorithm and environmental conditions
- **Drone Controller**: Executes movement commands in autonomous or pilot mode

## Installation

### Prerequisites

```bash
# ROS2 Jazzy
# Python 3.8+
# tkinter (usually included with Python)
# Gazebo Ignition (gz-sim)
# ArduPilot SITL
# MAVROS
# Mission Planner
```

### Setup

```bash
# Clone repository
cd ~/ros2_ws/src
git clone <repository-url>

# Build package
cd ~/ros2_ws
colcon build --packages-select bidding_pkg

# Source workspace
source install/setup.bash
```

## Simulation Environment

This system uses **Gazebo Ignition** for physics simulation and **ArduPilot SITL** (Software-In-The-Loop) for flight control, with **MAVROS** providing the ROS2 interface.

### Starting the Simulation Stack

#### 1. Launch Gazebo Ignition

```bash
gz sim -v4 -r alti_transition_runway_sensorsuite.sdf
```

- `-v4`: Verbose level 4 for detailed logging
- `-r`: Start simulation running immediately
- Uses the altitude transition runway sensor suite world

#### 2. Start ArduPilot SITL

```bash
sim_vehicle.py -v ArduPlane --model JSON --console --map --custom-location=63.4188137258128,10.401553034756608,0,0
```

**Parameters:**
- `-v ArduPlane`: Vehicle type (fixed-wing aircraft)
- `--model JSON`: Use JSON communication protocol with Gazebo
- `--console`: Open MAVProxy console for manual commands
- `--map`: Display real-time map visualization
- `--custom-location`: GPS coordinates (Trondheim, Norway)
  - Latitude: 63.4188137258128
  - Longitude: 10.401553034756608
  - Altitude: 0m
  - Heading: 0°

#### 3. Launch MAVROS Bridge (Per Drone)

```bash
ros2 launch mavros apm.launch fcu_url:=tcp://127.0.0.1:5762 namespace:=drone_1/mavros
```

**Parameters:**
- `fcu_url`: Flight controller connection (TCP to ArduPilot SITL)
- `namespace`: ROS2 namespace for multi-drone support

**For multiple drones**, increment the port and namespace:

```bash
# Drone 1
ros2 launch mavros apm.launch fcu_url:=tcp://127.0.0.1:5762 namespace:=drone_1/mavros

# Drone 2
ros2 launch mavros apm.launch fcu_url:=tcp://127.0.0.1:5772 namespace:=drone_2/mavros

# Drone 3
ros2 launch mavros apm.launch fcu_url:=tcp://127.0.0.1:5782 namespace:=drone_3/mavros
```

### Mission Planner Integration

Connect **Mission Planner** for real-time monitoring and manual override:

1. Open Mission Planner
2. Select connection type: **TCP**
3. Enter host: `127.0.0.1`
4. Enter port: `5762` (or appropriate SITL port)
5. Click **Connect**

Mission Planner provides:
- Real-time telemetry visualization
- Flight path monitoring
- Manual waypoint injection
- Parameter tuning
- Emergency override capabilities

## Usage

### Complete System Startup Sequence

#### Step 1: Start Simulation Environment

```bash
# Terminal 1: Launch Gazebo Ignition
gz sim -v4 -r alti_transition_runway_sensorsuite.sdf
```

#### Step 2: Start ArduPilot SITL

```bash
# Terminal 2: Launch ArduPilot with custom Trondheim location
sim_vehicle.py -v ArduPlane --model JSON --console --map --custom-location=63.4188137258128,10.401553034756608,0,0
```

#### Step 3: Launch MAVROS Bridges

```bash
# Terminal 3: Drone 1 MAVROS
ros2 launch mavros apm.launch fcu_url:=tcp://127.0.0.1:5762 namespace:=drone_1/mavros

# Terminal 4: Drone 2 MAVROS (if using multiple drones)
ros2 launch mavros apm.launch fcu_url:=tcp://127.0.0.1:5772 namespace:=drone_2/mavros

# Add more terminals as needed for additional drones

#If doing flight tests use this line to connect with MAVPROXY
 mavproxy.py --master=tcp:127.0.0.1:5763 --out=127.0.0.1:14550
```

#### Step 4: Launch Fire Response System

```bash
# Terminal N: Launch the coordination system
ros2 launch bidding_pkg interactive_integrated_launch.py


```

### Configuration Parameters

The interactive UI will prompt for:

| Parameter | Options | Description |
|-----------|---------|-------------|
| **Number of Drones** | 1-10 | Fleet size for mission |
| **Control Mode** | `autonomous`, `pilot` | Operation mode |
| **Path Planning Algorithm** | `multi_waypoint`, `sector_based`, `dynamic_clustering`, `priority_sweep` | Route optimization strategy |
| **Weather Conditions** | `clear`, `windy`, `rainy`, `storm` | Environmental parameters |

### Path Planning Algorithms

- **Multi-Waypoint**: Sequential waypoint navigation with dynamic adjustments
- **Sector-Based**: Divides area into sectors for systematic coverage
- **Dynamic Clustering**: Groups nearby fires for efficient response
- **Priority Sweep**: Prioritizes high-risk areas based on wind direction and intensity

### Path Planner Parameters

Advanced parameters (configured in launch file):

```python
'algorithm': algorithm,              # Selected algorithm
'weather': weather,                  # Weather condition
'downstream_angle_range': 45.0,      # Wind direction consideration (degrees)
'priority_boost': 1.5,               # Priority multiplier for downwind fires
'waypoint_tolerance': 0.00003,       # GPS precision threshold
'detour_threshold': 25.0,            # Max acceptable path deviation (meters)
'nearby_radius': 35.0,               # Radius for fire clustering (meters)
'wind_direction_threshold': 1.0,     # Wind alignment sensitivity
'replan_throttle_seconds': 120.0     # Minimum time between replans
```

## System Output

Upon launch, you'll see:

```
======================================================================
 FIRE RESPONSE SYSTEM CONFIGURATION
======================================================================
  Drones:     3
  Mode:       autonomous
  Algorithm:  multi_waypoint
  Weather:    windy
======================================================================

 Launching 13 nodes total.
```

## Node Communication

The system uses ROS2 topics for inter-node communication:

- Fire tasks published to allocation system
- Bidding process coordinates task assignment
- Path planners receive environmental updates
- Controllers execute planned trajectories via MAVROS
- UI displays real-time status

### MAVROS Topics

Each drone communicates through namespaced MAVROS topics:

```bash
# Command topics
/drone_1/mavros/setpoint_position/local
/drone_1/mavros/setpoint_velocity/cmd_vel

# State topics
/drone_1/mavros/state
/drone_1/mavros/local_position/pose
/drone_1/mavros/global_position/global

# Mission topics
/drone_1/mavros/mission/waypoints
```

### Monitoring Communications

```bash
# List all active topics
ros2 topic list

# Monitor specific drone state
ros2 topic echo /drone_1/mavros/state

# Watch global position
ros2 topic echo /drone_1/mavros/global_position/global
```

## Troubleshooting

### UI Won't Launch
```bash
# Check tkinter installation
python3 -c "import tkinter"

# If missing, install:
sudo apt-get install python3-tk
```

### Gazebo Connection Issues
```bash
# Verify Gazebo Ignition is running
gz topic -l

# Check Gazebo version
gz sim --version

# Restart Gazebo if needed
killall -9 gz
gz sim -v4 -r alti_transition_runway_sensorsuite.sdf
```

### ArduPilot SITL Problems
```bash
# Check if SITL is running
ps aux | grep sim_vehicle

# Verify port availability
netstat -tulpn | grep 5762

# Kill stuck SITL instances
pkill -9 -f sim_vehicle.py
```

### MAVROS Connection Failures
```bash
# Test MAVROS connection
ros2 topic list | grep mavros

# Check if FCU is reachable
ros2 topic echo /drone_1/mavros/state

# Verify TCP connection
telnet 127.0.0.1 5762


```

### Node Communication Issues
```bash
# Verify ROS2 domain
echo $ROS_DOMAIN_ID

# Check active nodes
ros2 node list

# Monitor topics
ros2 topic list
```

### Mission Planner Connection Issues
- Ensure ArduPilot SITL is running before connecting
- Verify correct port number (default: 5762)
- Check firewall settings for localhost connections
- Try MAVProxy console as alternative: already included with `sim_vehicle.py --console`

### Multi-Drone Port Conflicts
If running multiple drones, ensure each has unique ports:
- Drone 1: 5762, 5763 (MAVLink ports)
- Drone 2: 5772, 5773
- Drone 3: 5782, 5783

Edit `sim_vehicle.py` instances or use port forwarding flags.


### Environmental Data Errors
- Ensure Google Earth Engine credentials are configured
     :You will have to make your own account for Google Earth Engine - it is free and you just get the API access key
- Check internet connectivity for weather data
     :I've made it so that if you aren't on the internet it takes the last known weather from that area
- Verify API quotas and rate limits

## Development

### Adding Custom Algorithms

1. Implement algorithm in `path_planner` node
2. Add algorithm name to UI dropdown values
3. Update parameter handling in launch file

### Extending Drone Capabilities

Modify the dynamic drone node generation section (lines 98-136) to add additional per-drone nodes or parameters.


---

**Note**: This system is designed for simulation and research purposes. Always follow local regulations when deploying autonomous drone systems.
