"""
SangamIO TCP/UDP client for communicating with the vacuum robot.

Protocol:
- TCP port 5555: Commands (protobuf) and sensor streaming
- Messages are length-prefixed (4-byte big-endian length + protobuf payload)
"""

import socket
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, Any, Optional
import logging

# We'll use a simplified approach without protobuf compilation
# by manually parsing the key fields we need

logger = logging.getLogger(__name__)


@dataclass
class LidarPoint:
    """Single LiDAR measurement point."""
    angle_rad: float
    distance_m: float
    quality: int


@dataclass
class SensorData:
    """Container for all sensor data from SangamIO."""
    timestamp_us: int = 0

    # Bumpers
    bumper_left: bool = False
    bumper_right: bool = False

    # Cliffs
    cliff_left_side: bool = False
    cliff_left_front: bool = False
    cliff_right_front: bool = False
    cliff_right_side: bool = False

    # Battery
    battery_voltage: float = 0.0
    battery_percent: int = 0
    is_charging: bool = False
    is_docked: bool = False

    # Attachments
    dustbox_attached: bool = False
    water_tank_level: int = 0

    # Buttons
    start_button: bool = False
    dock_button: bool = False

    # Encoders (raw ticks)
    wheel_left: int = 0
    wheel_right: int = 0

    # IMU (raw values, already transformed to ROS frame by SangamIO)
    gyro_x: float = 0.0
    gyro_y: float = 0.0
    gyro_z: float = 0.0
    accel_x: float = 0.0
    accel_y: float = 0.0
    accel_z: float = 0.0

    # Tilt (gravity vector)
    tilt_x: float = 0.0
    tilt_y: float = 0.0
    tilt_z: float = 0.0

    # LiDAR scan
    lidar_points: list = field(default_factory=list)
    lidar_rpm: float = 0.0


class SangamIOClient:
    """Client for communicating with SangamIO daemon."""

    def __init__(self, host: str, port: int = 5555):
        self.host = host
        self.port = port
        self.socket: Optional[socket.socket] = None
        self.running = False
        self.recv_thread: Optional[threading.Thread] = None
        self.sensor_data = SensorData()
        self.lock = threading.Lock()
        self.callbacks: Dict[str, Callable] = {}

    def connect(self) -> bool:
        """Connect to SangamIO daemon."""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5.0)
            self.socket.connect((self.host, self.port))
            self.socket.settimeout(None)
            logger.info(f"Connected to SangamIO at {self.host}:{self.port}")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to SangamIO: {e}")
            return False

    def disconnect(self):
        """Disconnect from SangamIO."""
        self.running = False
        if self.recv_thread:
            self.recv_thread.join(timeout=2.0)
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
            self.socket = None
        logger.info("Disconnected from SangamIO")

    def start_receiving(self):
        """Start background thread to receive sensor data."""
        self.running = True
        self.recv_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.recv_thread.start()

    def on_sensor_update(self, callback: Callable[[str, SensorData], None]):
        """Register callback for sensor updates."""
        self.callbacks['sensor'] = callback

    def on_lidar_scan(self, callback: Callable[[list, float], None]):
        """Register callback for LiDAR scans."""
        self.callbacks['lidar'] = callback

    def _receive_loop(self):
        """Background thread to receive and parse messages."""
        buffer = bytearray()

        while self.running and self.socket:
            try:
                # Receive data
                data = self.socket.recv(4096)
                if not data:
                    logger.warning("Connection closed by SangamIO")
                    break

                buffer.extend(data)

                # Parse complete messages (length-prefixed protobuf)
                while len(buffer) >= 4:
                    msg_len = struct.unpack('>I', buffer[:4])[0]
                    if len(buffer) < 4 + msg_len:
                        break  # Wait for more data

                    msg_data = bytes(buffer[4:4+msg_len])
                    buffer = buffer[4+msg_len:]

                    self._parse_message(msg_data)

            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    logger.error(f"Receive error: {e}")
                break

    def _parse_message(self, data: bytes):
        """Parse a protobuf message from SangamIO.

        This is a simplified parser that extracts key fields without
        full protobuf compilation. For production, use compiled protobuf.
        """
        try:
            # Simple protobuf field parsing
            # Field 1 (topic): string
            # Field 2 (sensor_group): nested message
            # Field 3 (command): nested message

            idx = 0
            topic = ""

            while idx < len(data):
                if idx >= len(data):
                    break

                # Read field tag
                tag = data[idx]
                field_num = tag >> 3
                wire_type = tag & 0x07
                idx += 1

                if field_num == 1 and wire_type == 2:  # topic (string)
                    length, idx = self._read_varint(data, idx)
                    topic = data[idx:idx+length].decode('utf-8', errors='ignore')
                    idx += length

                elif field_num == 2 and wire_type == 2:  # sensor_group
                    length, idx = self._read_varint(data, idx)
                    sensor_data = data[idx:idx+length]
                    idx += length
                    self._parse_sensor_group(topic, sensor_data)

                else:
                    # Skip unknown fields
                    if wire_type == 0:  # varint
                        _, idx = self._read_varint(data, idx)
                    elif wire_type == 2:  # length-delimited
                        length, idx = self._read_varint(data, idx)
                        idx += length
                    elif wire_type == 5:  # 32-bit
                        idx += 4
                    elif wire_type == 1:  # 64-bit
                        idx += 8
                    else:
                        break

        except Exception as e:
            logger.debug(f"Parse error: {e}")

    def _parse_sensor_group(self, topic: str, data: bytes):
        """Parse SensorGroup message."""
        idx = 0
        timestamp_us = 0
        values = {}

        while idx < len(data):
            tag = data[idx]
            field_num = tag >> 3
            wire_type = tag & 0x07
            idx += 1

            if field_num == 2 and wire_type == 0:  # timestamp_us
                timestamp_us, idx = self._read_varint(data, idx)

            elif field_num == 3 and wire_type == 2:  # values map entry
                length, idx = self._read_varint(data, idx)
                entry_data = data[idx:idx+length]
                idx += length
                key, value = self._parse_map_entry(entry_data)
                if key:
                    values[key] = value
            else:
                # Skip
                if wire_type == 0:
                    _, idx = self._read_varint(data, idx)
                elif wire_type == 2:
                    length, idx = self._read_varint(data, idx)
                    idx += length
                elif wire_type == 5:
                    idx += 4
                elif wire_type == 1:
                    idx += 8

        # Update sensor data based on topic
        with self.lock:
            self.sensor_data.timestamp_us = timestamp_us

            if 'sensor_status' in topic:
                self._update_sensor_status(values)
                if 'sensor' in self.callbacks:
                    self.callbacks['sensor'](topic, self.sensor_data)

            elif 'lidar' in topic:
                self._update_lidar(values)
                if 'lidar' in self.callbacks:
                    self.callbacks['lidar'](self.sensor_data.lidar_points,
                                           self.sensor_data.lidar_rpm)

    def _parse_map_entry(self, data: bytes) -> tuple:
        """Parse a map entry (key-value pair)."""
        idx = 0
        key = ""
        value = None

        while idx < len(data):
            tag = data[idx]
            field_num = tag >> 3
            wire_type = tag & 0x07
            idx += 1

            if field_num == 1 and wire_type == 2:  # key (string)
                length, idx = self._read_varint(data, idx)
                key = data[idx:idx+length].decode('utf-8', errors='ignore')
                idx += length

            elif field_num == 2 and wire_type == 2:  # value (SensorValue)
                length, idx = self._read_varint(data, idx)
                value_data = data[idx:idx+length]
                idx += length
                value = self._parse_sensor_value(value_data)
            else:
                # Skip
                if wire_type == 0:
                    _, idx = self._read_varint(data, idx)
                elif wire_type == 2:
                    length, idx = self._read_varint(data, idx)
                    idx += length
                elif wire_type == 5:
                    idx += 4
                elif wire_type == 1:
                    idx += 8

        return key, value

    def _parse_sensor_value(self, data: bytes):
        """Parse SensorValue oneof."""
        if len(data) < 2:
            return None

        idx = 0
        tag = data[idx]
        field_num = tag >> 3
        wire_type = tag & 0x07
        idx += 1

        if field_num == 1 and wire_type == 0:  # bool
            val, _ = self._read_varint(data, idx)
            return bool(val)
        elif field_num == 2 and wire_type == 0:  # u32
            val, _ = self._read_varint(data, idx)
            return val
        elif field_num == 4 and wire_type == 0:  # i32
            val, _ = self._read_varint(data, idx)
            # Handle signed zigzag
            return val
        elif field_num == 6 and wire_type == 5:  # f32
            return struct.unpack('<f', data[idx:idx+4])[0]
        elif field_num == 7 and wire_type == 1:  # f64
            return struct.unpack('<d', data[idx:idx+8])[0]
        elif field_num == 10 and wire_type == 2:  # Vector3
            length, idx = self._read_varint(data, idx)
            return self._parse_vector3(data[idx:idx+length])
        elif field_num == 11 and wire_type == 2:  # PointCloud2D
            length, idx = self._read_varint(data, idx)
            return self._parse_pointcloud(data[idx:idx+length])

        return None

    def _parse_vector3(self, data: bytes) -> tuple:
        """Parse Vector3 message."""
        x = y = z = 0.0
        idx = 0
        while idx < len(data):
            tag = data[idx]
            field_num = tag >> 3
            idx += 1

            if field_num in (1, 2, 3) and idx + 4 <= len(data):
                val = struct.unpack('<f', data[idx:idx+4])[0]
                idx += 4
                if field_num == 1:
                    x = val
                elif field_num == 2:
                    y = val
                else:
                    z = val
            else:
                break
        return (x, y, z)

    def _parse_pointcloud(self, data: bytes) -> list:
        """Parse PointCloud2D message."""
        points = []
        idx = 0

        while idx < len(data):
            tag = data[idx]
            field_num = tag >> 3
            wire_type = tag & 0x07
            idx += 1

            if field_num == 1 and wire_type == 2:  # points (repeated LidarPoint)
                length, idx = self._read_varint(data, idx)
                point_data = data[idx:idx+length]
                idx += length
                point = self._parse_lidar_point(point_data)
                if point:
                    points.append(point)
            else:
                if wire_type == 0:
                    _, idx = self._read_varint(data, idx)
                elif wire_type == 2:
                    length, idx = self._read_varint(data, idx)
                    idx += length
                elif wire_type == 5:
                    idx += 4
                else:
                    break

        return points

    def _parse_lidar_point(self, data: bytes) -> Optional[LidarPoint]:
        """Parse LidarPoint message."""
        angle = distance = 0.0
        quality = 0
        idx = 0

        while idx < len(data):
            if idx >= len(data):
                break
            tag = data[idx]
            field_num = tag >> 3
            wire_type = tag & 0x07
            idx += 1

            if wire_type == 5 and idx + 4 <= len(data):  # float
                val = struct.unpack('<f', data[idx:idx+4])[0]
                idx += 4
                if field_num == 1:
                    angle = val
                elif field_num == 2:
                    distance = val
            elif wire_type == 0:  # varint
                val, idx = self._read_varint(data, idx)
                if field_num == 3:
                    quality = val
            else:
                break

        if distance > 0:
            return LidarPoint(angle, distance, quality)
        return None

    def _update_sensor_status(self, values: dict):
        """Update sensor data from sensor_status values."""
        # Bumpers
        if 'bumper_left' in values:
            self.sensor_data.bumper_left = bool(values['bumper_left'])
        if 'bumper_right' in values:
            self.sensor_data.bumper_right = bool(values['bumper_right'])

        # Cliffs
        if 'cliff_left_side' in values:
            self.sensor_data.cliff_left_side = bool(values['cliff_left_side'])
        if 'cliff_left_front' in values:
            self.sensor_data.cliff_left_front = bool(values['cliff_left_front'])
        if 'cliff_right_front' in values:
            self.sensor_data.cliff_right_front = bool(values['cliff_right_front'])
        if 'cliff_right_side' in values:
            self.sensor_data.cliff_right_side = bool(values['cliff_right_side'])

        # Battery
        if 'battery_voltage' in values:
            self.sensor_data.battery_voltage = float(values['battery_voltage'])
        if 'battery_level' in values:
            self.sensor_data.battery_percent = int(values['battery_level'])
        if 'is_charging' in values:
            self.sensor_data.is_charging = bool(values['is_charging'])
        if 'is_dock_connected' in values:
            self.sensor_data.is_docked = bool(values['is_dock_connected'])

        # Attachments
        if 'dustbox_attached' in values:
            self.sensor_data.dustbox_attached = bool(values['dustbox_attached'])
        if 'water_tank_level' in values:
            self.sensor_data.water_tank_level = int(values['water_tank_level'])

        # Buttons
        if 'start_button' in values:
            self.sensor_data.start_button = bool(values['start_button'])
        if 'dock_button' in values:
            self.sensor_data.dock_button = bool(values['dock_button'])

        # Encoders
        if 'wheel_left' in values:
            self.sensor_data.wheel_left = int(values['wheel_left'])
        if 'wheel_right' in values:
            self.sensor_data.wheel_right = int(values['wheel_right'])

        # IMU - gyro
        if 'gyro' in values and isinstance(values['gyro'], tuple):
            self.sensor_data.gyro_x = values['gyro'][0]
            self.sensor_data.gyro_y = values['gyro'][1]
            self.sensor_data.gyro_z = values['gyro'][2]

        # IMU - accel
        if 'accel' in values and isinstance(values['accel'], tuple):
            self.sensor_data.accel_x = values['accel'][0]
            self.sensor_data.accel_y = values['accel'][1]
            self.sensor_data.accel_z = values['accel'][2]

        # Tilt
        if 'tilt' in values and isinstance(values['tilt'], tuple):
            self.sensor_data.tilt_x = values['tilt'][0]
            self.sensor_data.tilt_y = values['tilt'][1]
            self.sensor_data.tilt_z = values['tilt'][2]

    def _update_lidar(self, values: dict):
        """Update LiDAR data."""
        if 'scan' in values and isinstance(values['scan'], list):
            self.sensor_data.lidar_points = values['scan']
        if 'rpm' in values:
            self.sensor_data.lidar_rpm = float(values['rpm'])

    @staticmethod
    def _read_varint(data: bytes, idx: int) -> tuple:
        """Read a varint from data starting at idx."""
        result = 0
        shift = 0
        while idx < len(data):
            b = data[idx]
            idx += 1
            result |= (b & 0x7F) << shift
            if (b & 0x80) == 0:
                break
            shift += 7
        return result, idx

    # Command methods

    def send_command(self, command_bytes: bytes):
        """Send a command to SangamIO."""
        if not self.socket:
            return False
        try:
            # Length-prefix the message
            msg = struct.pack('>I', len(command_bytes)) + command_bytes
            self.socket.sendall(msg)
            return True
        except Exception as e:
            logger.error(f"Send error: {e}")
            return False

    def _build_component_control(self, component_id: str, action_type: int,
                                  config: dict = None) -> bytes:
        """Build a ComponentControl command message."""
        # Simplified protobuf encoding
        # Message -> topic="command", command=ComponentControl

        # Build ComponentAction
        action_data = bytearray()
        # Field 1: type (enum as varint)
        action_data.append(0x08)  # field 1, wire type 0
        action_data.append(action_type)

        # Field 2: config map entries
        if config:
            for key, value in config.items():
                entry = self._encode_config_entry(key, value)
                action_data.append(0x12)  # field 2, wire type 2
                action_data.extend(self._encode_varint(len(entry)))
                action_data.extend(entry)

        # Build ComponentControl
        control_data = bytearray()
        # Field 1: id (string)
        id_bytes = component_id.encode('utf-8')
        control_data.append(0x0A)  # field 1, wire type 2
        control_data.extend(self._encode_varint(len(id_bytes)))
        control_data.extend(id_bytes)
        # Field 2: action
        control_data.append(0x12)  # field 2, wire type 2
        control_data.extend(self._encode_varint(len(action_data)))
        control_data.extend(action_data)

        # Build Command (oneof field 1 = component_control)
        cmd_data = bytearray()
        cmd_data.append(0x0A)  # field 1, wire type 2
        cmd_data.extend(self._encode_varint(len(control_data)))
        cmd_data.extend(control_data)

        # Build Message
        msg_data = bytearray()
        # Field 1: topic = "command"
        topic = b"command"
        msg_data.append(0x0A)  # field 1, wire type 2
        msg_data.extend(self._encode_varint(len(topic)))
        msg_data.extend(topic)
        # Field 3: command
        msg_data.append(0x1A)  # field 3, wire type 2
        msg_data.extend(self._encode_varint(len(cmd_data)))
        msg_data.extend(cmd_data)

        return bytes(msg_data)

    def _encode_config_entry(self, key: str, value) -> bytes:
        """Encode a config map entry."""
        entry = bytearray()
        # Field 1: key (string)
        key_bytes = key.encode('utf-8')
        entry.append(0x0A)
        entry.extend(self._encode_varint(len(key_bytes)))
        entry.extend(key_bytes)

        # Field 2: value (SensorValue)
        value_data = self._encode_sensor_value(value)
        entry.append(0x12)
        entry.extend(self._encode_varint(len(value_data)))
        entry.extend(value_data)

        return bytes(entry)

    def _encode_sensor_value(self, value) -> bytes:
        """Encode a SensorValue."""
        data = bytearray()
        if isinstance(value, bool):
            data.append(0x08)  # field 1, wire type 0
            data.append(1 if value else 0)
        elif isinstance(value, int):
            data.append(0x10)  # field 2 (u32), wire type 0
            data.extend(self._encode_varint(value))
        elif isinstance(value, float):
            data.append(0x35)  # field 6 (f32), wire type 5
            data.extend(struct.pack('<f', value))
        return bytes(data)

    @staticmethod
    def _encode_varint(value: int) -> bytes:
        """Encode an integer as a varint."""
        result = bytearray()
        while value > 127:
            result.append((value & 0x7F) | 0x80)
            value >>= 7
        result.append(value & 0x7F)
        return bytes(result)

    # High-level command methods

    def set_velocity(self, linear: float, angular: float) -> bool:
        """Set robot velocity (m/s, rad/s)."""
        cmd = self._build_component_control(
            "drive", 3,  # CONFIGURE
            {"linear": linear, "angular": angular}
        )
        return self.send_command(cmd)

    def stop(self) -> bool:
        """Emergency stop."""
        cmd = self._build_component_control("drive", 2)  # RESET
        return self.send_command(cmd)

    def set_vacuum(self, speed: int) -> bool:
        """Set vacuum motor speed (0-100%)."""
        action = 0 if speed > 0 else 1  # ENABLE or DISABLE
        cmd = self._build_component_control(
            "vacuum", action,
            {"speed": speed} if speed > 0 else None
        )
        return self.send_command(cmd)

    def set_main_brush(self, speed: int) -> bool:
        """Set main brush speed (0-100%)."""
        action = 0 if speed > 0 else 1
        cmd = self._build_component_control(
            "main_brush", action,
            {"speed": speed} if speed > 0 else None
        )
        return self.send_command(cmd)

    def set_side_brush(self, speed: int) -> bool:
        """Set side brush speed (0-100%)."""
        action = 0 if speed > 0 else 1
        cmd = self._build_component_control(
            "side_brush", action,
            {"speed": speed} if speed > 0 else None
        )
        return self.send_command(cmd)

    def set_water_pump(self, speed: int) -> bool:
        """Set water pump speed (0-100%)."""
        action = 0 if speed > 0 else 1
        cmd = self._build_component_control(
            "water_pump", action,
            {"speed": speed} if speed > 0 else None
        )
        return self.send_command(cmd)

    def set_led(self, state: int) -> bool:
        """Set LED state (0-18)."""
        cmd = self._build_component_control(
            "led", 3,  # CONFIGURE
            {"state": state}
        )
        return self.send_command(cmd)

    def set_lidar(self, enabled: bool) -> bool:
        """Enable/disable LiDAR motor."""
        action = 0 if enabled else 1  # ENABLE or DISABLE
        cmd = self._build_component_control("lidar", action)
        return self.send_command(cmd)
