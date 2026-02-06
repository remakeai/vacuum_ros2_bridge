# vacuum_ros2_bridge

ROS2 Jazzy bridge for SangamIO vacuum robot controller. This package enables communication between ROS2 and vacuum robots running the SangamIO firmware (part of [VacuumTiger](https://github.com/codetiger/VacuumTiger)).

## Overview

The bridge connects to [SangamIO](https://github.com/codetiger/VacuumTiger) via TCP and translates between its protobuf-based protocol and standard ROS2 messages. It provides:

- **Sensor data**: LiDAR scans, IMU, wheel odometry, bumpers, cliffs, battery status
- **Actuator control**: Drive motors, vacuum, brushes, water pump, LEDs
- **Standard interfaces**: Uses `sensor_msgs`, `geometry_msgs`, `nav_msgs` for compatibility with the ROS2 ecosystem

## Prerequisites

- ROS2 Jazzy
- Vacuum robot running [SangamIO](https://github.com/codetiger/VacuumTiger) (TCP port 5555)
- Network connectivity to the robot

## Installation

```bash
# Clone into your workspace
cd ~/ros2_ws/src
git clone <repository_url> vacuum_ros2_bridge

# Build
cd ~/ros2_ws
colcon build --packages-select vacuum_ros2_bridge
source install/setup.bash
```

## Usage

### Launch the bridge

```bash
# Default IP (192.168.1.143)
ros2 launch vacuum_ros2_bridge bridge.launch.py

# Custom IP address
ros2 launch vacuum_ros2_bridge bridge.launch.py robot_ip:=192.168.1.100

# All parameters
ros2 launch vacuum_ros2_bridge bridge.launch.py \
    robot_ip:=192.168.1.100 \
    robot_port:=5555 \
    frame_id:=base_link \
    odom_frame_id:=odom \
    lidar_frame_id:=laser
```

### Teleoperate the robot

```bash
# Using teleop_twist_keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### View sensor data

```bash
# LiDAR scan
ros2 topic echo /scan

# Odometry
ros2 topic echo /odom

# Battery status
ros2 topic echo /battery

# Full vacuum status
ros2 topic echo /vacuum_status
```

### Control actuators

```bash
# Start vacuum at 50% power
ros2 service call /set_actuator vacuum_ros2_bridge/srv/SetActuator \
    "{component: 'vacuum', speed: 50}"

# Start main brush at 80%
ros2 service call /set_actuator vacuum_ros2_bridge/srv/SetActuator \
    "{component: 'main_brush', speed: 80}"

# Start side brush at 100%
ros2 service call /set_actuator vacuum_ros2_bridge/srv/SetActuator \
    "{component: 'side_brush', speed: 100}"

# Enable water pump at 30%
ros2 service call /set_actuator vacuum_ros2_bridge/srv/SetActuator \
    "{component: 'water_pump', speed: 30}"

# Stop an actuator (speed: 0)
ros2 service call /set_actuator vacuum_ros2_bridge/srv/SetActuator \
    "{component: 'vacuum', speed: 0}"
```

### Control LEDs

```bash
# Set LED state (0-18)
ros2 service call /set_led vacuum_ros2_bridge/srv/SetLed "{state: 5}"
```

### Enable/disable LiDAR

```bash
# Enable LiDAR motor
ros2 service call /set_lidar vacuum_ros2_bridge/srv/SetLidar "{enable: true}"

# Disable LiDAR motor
ros2 service call /set_lidar vacuum_ros2_bridge/srv/SetLidar "{enable: false}"
```

## Topics

### Published Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | ~5 Hz | LiDAR scan (360 points) |
| `/imu` | `sensor_msgs/Imu` | ~110 Hz | Gyroscope and accelerometer |
| `/odom` | `nav_msgs/Odometry` | ~110 Hz | Wheel odometry |
| `/battery` | `sensor_msgs/BatteryState` | ~110 Hz | Battery voltage and percentage |
| `/bumper` | `vacuum_ros2_bridge/Bumper` | ~110 Hz | Left/right bumper state |
| `/cliff` | `vacuum_ros2_bridge/Cliff` | ~110 Hz | Four cliff sensors |
| `/vacuum_status` | `vacuum_ros2_bridge/VacuumStatus` | ~110 Hz | Complete robot status |

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands (linear.x, angular.z) |
| `/actuator_cmd` | `vacuum_ros2_bridge/ActuatorCommand` | Direct actuator control |
| `/led_cmd` | `vacuum_ros2_bridge/LedCommand` | LED state control |

## Services

| Service | Type | Description |
|---------|------|-------------|
| `/set_actuator` | `vacuum_ros2_bridge/SetActuator` | Control vacuum, brushes, water pump |
| `/set_led` | `vacuum_ros2_bridge/SetLed` | Set LED state (0-18) |
| `/set_lidar` | `vacuum_ros2_bridge/SetLidar` | Enable/disable LiDAR motor |

## Custom Messages

### Bumper.msg
```
std_msgs/Header header
bool left
bool right
```

### Cliff.msg
```
std_msgs/Header header
bool left_side
bool left_front
bool right_front
bool right_side
```

### VacuumStatus.msg
```
std_msgs/Header header
float32 battery_voltage      # Volts (13.5-15.5V typical)
uint8 battery_percent        # 0-100%
bool is_charging
bool is_docked
bool dustbox_attached
uint8 water_tank_level       # 0-100% (2-in-1 mop box only)
bool start_button_pressed
bool dock_button_pressed
```

### ActuatorCommand.msg
```
string component             # vacuum, main_brush, side_brush, water_pump
uint8 speed                  # 0-100%
```

## Frame IDs

The bridge uses three configurable frame IDs:

- `base_link` - Robot base frame (bumpers, IMU)
- `odom` - Odometry frame
- `laser` - LiDAR frame

## Robot Parameters

Default physical parameters used for odometry:

| Parameter | Value | Description |
|-----------|-------|-------------|
| Wheel base | 0.233 m | Distance between wheels |
| Ticks per meter | 4464 | Encoder resolution |

## Troubleshooting

### Cannot connect to SangamIO
- Verify the robot IP address: `ping 192.168.1.143`
- Check that SangamIO is running on the robot
- Ensure TCP port 5555 is accessible

### No LiDAR data
- Check if LiDAR motor is enabled: `ros2 service call /set_lidar ... "{enable: true}"`
- Verify LiDAR is spinning (audible motor sound)

### Odometry drift
- The odometry is computed from wheel encoders only
- For better localization, fuse with IMU data using `robot_localization`

## License

Apache 2.0
