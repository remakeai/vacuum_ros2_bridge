#!/usr/bin/env python3
"""
ROS2 Bridge Node for SangamIO Vacuum Robot

This node bridges between SangamIO (running on the vacuum) and ROS2.

Published topics:
  /scan (sensor_msgs/LaserScan) - LiDAR scan at ~5Hz
  /imu (sensor_msgs/Imu) - IMU data at ~110Hz
  /odom (nav_msgs/Odometry) - Wheel odometry at ~110Hz
  /battery (sensor_msgs/BatteryState) - Battery status
  /bumper (vacuum_ros2_bridge/Bumper) - Bumper sensors
  /cliff (vacuum_ros2_bridge/Cliff) - Cliff sensors
  /vacuum_status (vacuum_ros2_bridge/VacuumStatus) - General status

Subscribed topics:
  /cmd_vel (geometry_msgs/Twist) - Velocity commands
  /actuator_cmd (vacuum_ros2_bridge/ActuatorCommand) - Actuator control
  /led_cmd (vacuum_ros2_bridge/LedCommand) - LED control

Services:
  /set_actuator (vacuum_ros2_bridge/SetActuator) - Set actuator speed
  /set_led (vacuum_ros2_bridge/SetLed) - Set LED state
  /set_lidar (vacuum_ros2_bridge/SetLidar) - Enable/disable LiDAR
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan, Imu, BatteryState
from geometry_msgs.msg import Twist, Quaternion, Vector3
from nav_msgs.msg import Odometry

# Custom messages (will be available after building)
from vacuum_ros2_bridge.msg import Bumper, Cliff, VacuumStatus, ActuatorCommand, LedCommand
from vacuum_ros2_bridge.srv import SetActuator, SetLed, SetLidar

# SangamIO client
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from vacuum_ros2_bridge.sangamio_client import SangamIOClient, SensorData


class VacuumBridgeNode(Node):
    """ROS2 bridge node for SangamIO vacuum robot."""

    # Robot physical parameters
    WHEEL_BASE = 0.233  # meters
    TICKS_PER_METER = 4464.0  # encoder ticks per meter

    def __init__(self):
        super().__init__('vacuum_bridge')

        # Declare parameters
        self.declare_parameter('robot_ip', '192.168.1.143')
        self.declare_parameter('robot_port', 5555)
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('lidar_frame_id', 'laser')

        # Get parameters
        robot_ip = self.get_parameter('robot_ip').value
        robot_port = self.get_parameter('robot_port').value
        self.frame_id = self.get_parameter('frame_id').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.lidar_frame_id = self.get_parameter('lidar_frame_id').value

        # SangamIO client
        self.client = SangamIOClient(robot_ip, robot_port)
        self.client.on_sensor_update(self._on_sensor_update)
        self.client.on_lidar_scan(self._on_lidar_scan)

        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers
        self.scan_pub = self.create_publisher(LaserScan, '/scan', sensor_qos)
        self.imu_pub = self.create_publisher(Imu, '/imu', sensor_qos)
        self.odom_pub = self.create_publisher(Odometry, '/odom', sensor_qos)
        self.battery_pub = self.create_publisher(BatteryState, '/battery', 10)
        self.bumper_pub = self.create_publisher(Bumper, '/bumper', sensor_qos)
        self.cliff_pub = self.create_publisher(Cliff, '/cliff', sensor_qos)
        self.status_pub = self.create_publisher(VacuumStatus, '/vacuum_status', 10)

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, 10)
        self.actuator_sub = self.create_subscription(
            ActuatorCommand, '/actuator_cmd', self._actuator_callback, 10)
        self.led_sub = self.create_subscription(
            LedCommand, '/led_cmd', self._led_callback, 10)

        # Services
        self.set_actuator_srv = self.create_service(
            SetActuator, '/set_actuator', self._set_actuator_callback)
        self.set_led_srv = self.create_service(
            SetLed, '/set_led', self._set_led_callback)
        self.set_lidar_srv = self.create_service(
            SetLidar, '/set_lidar', self._set_lidar_callback)

        # Odometry state
        self.last_left_ticks = None
        self.last_right_ticks = None
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_odom_time = None

        # Connect to robot
        self.get_logger().info(f'Connecting to SangamIO at {robot_ip}:{robot_port}...')
        if self.client.connect():
            self.client.start_receiving()
            self.get_logger().info('Connected to SangamIO')
        else:
            self.get_logger().error('Failed to connect to SangamIO')

    def _on_sensor_update(self, topic: str, data: SensorData):
        """Handle sensor data update from SangamIO."""
        now = self.get_clock().now().to_msg()

        # Publish IMU
        self._publish_imu(data, now)

        # Publish odometry
        self._publish_odom(data, now)

        # Publish bumper
        self._publish_bumper(data, now)

        # Publish cliff
        self._publish_cliff(data, now)

        # Publish battery (at lower rate)
        self._publish_battery(data, now)

        # Publish vacuum status
        self._publish_status(data, now)

    def _on_lidar_scan(self, points: list, rpm: float):
        """Handle LiDAR scan from SangamIO."""
        if not points:
            return

        now = self.get_clock().now().to_msg()

        msg = LaserScan()
        msg.header.stamp = now
        msg.header.frame_id = self.lidar_frame_id

        # LiDAR parameters
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi
        msg.angle_increment = 2.0 * math.pi / 360  # ~1 degree
        msg.time_increment = 0.0  # Not available
        msg.scan_time = 1.0 / 5.0  # ~5 Hz
        msg.range_min = 0.15  # 15 cm
        msg.range_max = 8.0  # 8 m

        # Initialize ranges array with inf (no return)
        ranges = [float('inf')] * 360
        intensities = [0.0] * 360

        # Fill in actual measurements
        for point in points:
            # Convert angle to index (0-359)
            angle_deg = math.degrees(point.angle_rad) % 360
            idx = int(angle_deg) % 360

            # Only update if this is a valid measurement
            if 0 < point.distance_m < msg.range_max:
                ranges[idx] = point.distance_m
                intensities[idx] = float(point.quality)

        msg.ranges = ranges
        msg.intensities = intensities

        self.scan_pub.publish(msg)

    def _publish_imu(self, data: SensorData, stamp):
        """Publish IMU message."""
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id

        # Angular velocity (rad/s) - raw values need scaling
        # Typical gyro scale: 2000 dps full scale, 16-bit = 2000/32768 = 0.061 dps/LSB
        GYRO_SCALE = 0.061 * math.pi / 180.0  # Convert to rad/s
        msg.angular_velocity.x = data.gyro_x * GYRO_SCALE
        msg.angular_velocity.y = data.gyro_y * GYRO_SCALE
        msg.angular_velocity.z = data.gyro_z * GYRO_SCALE

        # Linear acceleration (m/s^2) - raw values need scaling
        # Typical accel scale: 2g full scale, 16-bit = 2*9.81/32768 = 0.0006 m/s²/LSB
        ACCEL_SCALE = 2.0 * 9.81 / 32768.0
        msg.linear_acceleration.x = data.accel_x * ACCEL_SCALE
        msg.linear_acceleration.y = data.accel_y * ACCEL_SCALE
        msg.linear_acceleration.z = data.accel_z * ACCEL_SCALE

        # Covariance (unknown, use -1)
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity_covariance[0] = 0.01
        msg.linear_acceleration_covariance[0] = 0.1

        self.imu_pub.publish(msg)

    def _publish_odom(self, data: SensorData, stamp):
        """Publish odometry message from wheel encoders."""
        left_ticks = data.wheel_left
        right_ticks = data.wheel_right

        if self.last_left_ticks is None:
            self.last_left_ticks = left_ticks
            self.last_right_ticks = right_ticks
            self.last_odom_time = self.get_clock().now()
            return

        # Handle tick wraparound (u16)
        def tick_diff(new, old):
            diff = new - old
            if diff > 32768:
                diff -= 65536
            elif diff < -32768:
                diff += 65536
            return diff

        dl = tick_diff(left_ticks, self.last_left_ticks) / self.TICKS_PER_METER
        dr = tick_diff(right_ticks, self.last_right_ticks) / self.TICKS_PER_METER

        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks

        # Compute odometry
        d = (dl + dr) / 2.0
        dtheta = (dr - dl) / self.WHEEL_BASE

        # Update pose
        self.x += d * math.cos(self.theta + dtheta / 2.0)
        self.y += d * math.sin(self.theta + dtheta / 2.0)
        self.theta += dtheta

        # Normalize theta to [-pi, pi]
        while self.theta > math.pi:
            self.theta -= 2.0 * math.pi
        while self.theta < -math.pi:
            self.theta += 2.0 * math.pi

        # Compute velocities
        now = self.get_clock().now()
        if self.last_odom_time:
            dt = (now - self.last_odom_time).nanoseconds / 1e9
            if dt > 0:
                vx = d / dt
                vth = dtheta / dt
            else:
                vx = vth = 0.0
        else:
            vx = vth = 0.0
        self.last_odom_time = now

        # Build message
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame_id
        msg.child_frame_id = self.frame_id

        # Position
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0

        # Orientation (quaternion from yaw)
        msg.pose.pose.orientation = self._yaw_to_quaternion(self.theta)

        # Velocity
        msg.twist.twist.linear.x = vx
        msg.twist.twist.angular.z = vth

        # Covariance (simple diagonal)
        msg.pose.covariance[0] = 0.01  # x
        msg.pose.covariance[7] = 0.01  # y
        msg.pose.covariance[35] = 0.01  # yaw
        msg.twist.covariance[0] = 0.01  # vx
        msg.twist.covariance[35] = 0.01  # vth

        self.odom_pub.publish(msg)

    def _publish_bumper(self, data: SensorData, stamp):
        """Publish bumper message."""
        msg = Bumper()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.left = data.bumper_left
        msg.right = data.bumper_right
        self.bumper_pub.publish(msg)

    def _publish_cliff(self, data: SensorData, stamp):
        """Publish cliff message."""
        msg = Cliff()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.left_side = data.cliff_left_side
        msg.left_front = data.cliff_left_front
        msg.right_front = data.cliff_right_front
        msg.right_side = data.cliff_right_side
        self.cliff_pub.publish(msg)

    def _publish_battery(self, data: SensorData, stamp):
        """Publish battery state."""
        msg = BatteryState()
        msg.header.stamp = stamp
        msg.voltage = data.battery_voltage
        msg.percentage = data.battery_percent / 100.0
        msg.power_supply_status = (
            BatteryState.POWER_SUPPLY_STATUS_CHARGING if data.is_charging
            else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        )
        msg.present = True
        self.battery_pub.publish(msg)

    def _publish_status(self, data: SensorData, stamp):
        """Publish vacuum status."""
        msg = VacuumStatus()
        msg.header.stamp = stamp
        msg.battery_voltage = data.battery_voltage
        msg.battery_percent = data.battery_percent
        msg.is_charging = data.is_charging
        msg.is_docked = data.is_docked
        msg.dustbox_attached = data.dustbox_attached
        msg.water_tank_level = data.water_tank_level
        msg.start_button_pressed = data.start_button
        msg.dock_button_pressed = data.dock_button
        self.status_pub.publish(msg)

    def _cmd_vel_callback(self, msg: Twist):
        """Handle velocity command."""
        self.client.set_velocity(msg.linear.x, msg.angular.z)

    def _actuator_callback(self, msg: ActuatorCommand):
        """Handle actuator command."""
        component = msg.component.lower()
        speed = msg.speed

        if component == 'vacuum':
            self.client.set_vacuum(speed)
        elif component == 'main_brush':
            self.client.set_main_brush(speed)
        elif component == 'side_brush':
            self.client.set_side_brush(speed)
        elif component == 'water_pump':
            self.client.set_water_pump(speed)
        else:
            self.get_logger().warn(f'Unknown actuator: {component}')

    def _led_callback(self, msg: LedCommand):
        """Handle LED command."""
        self.client.set_led(msg.state)

    def _set_actuator_callback(self, request, response):
        """Handle set_actuator service."""
        component = request.component.lower()
        speed = request.speed

        success = False
        if component == 'vacuum':
            success = self.client.set_vacuum(speed)
        elif component == 'main_brush':
            success = self.client.set_main_brush(speed)
        elif component == 'side_brush':
            success = self.client.set_side_brush(speed)
        elif component == 'water_pump':
            success = self.client.set_water_pump(speed)
        else:
            response.success = False
            response.message = f'Unknown actuator: {component}'
            return response

        response.success = success
        response.message = 'OK' if success else 'Failed to send command'
        return response

    def _set_led_callback(self, request, response):
        """Handle set_led service."""
        success = self.client.set_led(request.state)
        response.success = success
        response.message = 'OK' if success else 'Failed to send command'
        return response

    def _set_lidar_callback(self, request, response):
        """Handle set_lidar service."""
        success = self.client.set_lidar(request.enable)
        response.success = success
        response.message = 'OK' if success else 'Failed to send command'
        return response

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Quaternion:
        """Convert yaw angle to quaternion."""
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def destroy_node(self):
        """Clean up on shutdown."""
        self.client.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VacuumBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
