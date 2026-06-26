"""
SangamIO TCP/UDP client for communicating with the vacuum robot.

Protocol:
- TCP port 5555: Commands (protobuf) and sensor streaming
- Messages are length-prefixed (4-byte big-endian length + protobuf payload)
"""

import socket
import select
import struct
import math
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

    def __init__(self, host: str, port: int = 5555, udp_port: Optional[int] = None):
        self.host = host
        self.port = port
        # SangamIO streams sensor/LiDAR telemetry over UDP (not TCP) to the
        # client's IP on the same port as the TCP command connection. See
        # SangamIO main.rs: udp_addr = SocketAddr::new(client_ip, udp_streaming_port),
        # where udp_streaming_port defaults to the TCP bind port. TCP is
        # commands-only and its open connection registers us for UDP streaming.
        self.udp_port = udp_port if udp_port is not None else port
        # Reconnect if no telemetry arrives for this long (seconds).
        self.telemetry_timeout = 3.0
        # If no telemetry arrives within this shorter window, first ask SangamIO to
        # switch this client to TCP telemetry (NAT/remote fallback) before giving up
        # and reconnecting. Must be < telemetry_timeout.
        self.udp_fallback_after = 1.5
        # Sticky once TCP telemetry has worked: request it immediately on (re)connect
        # so remote sessions don't re-pay the UDP-silence wait on every reconnect.
        # Reset whenever a UDP datagram is seen (we're back on a network where UDP works).
        self._prefer_tcp = False
        self.socket: Optional[socket.socket] = None
        self.udp_socket: Optional[socket.socket] = None
        self.running = False
        self.connected = False
        self.last_rx_time = 0.0
        self.worker: Optional[threading.Thread] = None
        self.sensor_data = SensorData()
        self.lock = threading.Lock()
        self.send_lock = threading.Lock()
        self.callbacks: Dict[str, Callable] = {}

    def on_sensor_update(self, callback: Callable[[str, SensorData], None]):
        """Register callback for sensor updates."""
        self.callbacks['sensor'] = callback

    def on_lidar_scan(self, callback: Callable[[list, float], None]):
        """Register callback for LiDAR scans."""
        self.callbacks['lidar'] = callback

    def on_status(self, callback: Callable[[bool], None]):
        """Register a connection-status callback, called with True on connect
        and False on disconnect."""
        self.callbacks['status'] = callback

    def start(self):
        """Start the self-healing connection worker: connects to SangamIO and
        keeps both the TCP command channel and the UDP telemetry stream alive,
        reconnecting with backoff if the connection closes or telemetry stalls."""
        if self.running:
            return
        self.running = True
        self.worker = threading.Thread(target=self._worker, daemon=True)
        self.worker.start()

    # Backwards-compatible aliases (older callers used connect()+start_receiving()).
    def connect(self) -> bool:
        self.start()
        return True

    def start_receiving(self):
        self.start()

    def disconnect(self):
        """Stop the worker and close the sockets."""
        self.running = False
        if self.worker and self.worker is not threading.current_thread():
            self.worker.join(timeout=3.0)
        self._close_sockets()
        logger.info("Disconnected from SangamIO")

    def _open(self) -> bool:
        """Bind the UDP telemetry socket, then open the TCP command connection
        (which registers this client for UDP streaming). Returns True on success."""
        try:
            self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.udp_socket.bind(("0.0.0.0", self.udp_port))

            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5.0)
            self.socket.connect((self.host, self.port))
            self.socket.settimeout(None)
            logger.info(
                f"Connected to SangamIO at {self.host}:{self.port} "
                f"(UDP telemetry on :{self.udp_port})")
            return True
        except Exception as e:
            logger.warning(f"Connect to SangamIO failed: {e}")
            self._close_sockets()
            return False

    def _close_sockets(self):
        for attr in ('socket', 'udp_socket'):
            s = getattr(self, attr)
            if s is not None:
                try:
                    s.close()
                except Exception:
                    pass
            setattr(self, attr, None)

    def _set_connected(self, value: bool):
        if value != self.connected:
            self.connected = value
            cb = self.callbacks.get('status')
            if cb:
                try:
                    cb(value)
                except Exception:
                    pass

    def _worker(self):
        """Connect -> receive (TCP + UDP) -> on loss, reconnect with backoff."""
        backoff = 1.0
        while self.running:
            if not self._open():
                self._set_connected(False)
                time.sleep(backoff)
                backoff = min(backoff * 2.0, 10.0)
                continue
            backoff = 1.0
            self.last_rx_time = time.time()
            self._set_connected(True)
            self._receive_until_lost()
            self._set_connected(False)
            self._close_sockets()
            if self.running:
                time.sleep(1.0)

    def _receive_until_lost(self):
        """Read telemetry from UDP (fast path) and/or the TCP channel, until the
        connection drops or telemetry stalls. Each frame -- a UDP datagram or a
        TCP framed message -- is one length-prefixed protobuf message (see
        udp_publisher.rs). If UDP telemetry does not arrive (e.g. the client is
        behind NAT / remote), ask SangamIO to stream over TCP instead, which
        traverses NAT exactly like the command channel already does."""
        tcp_buffer = bytearray()
        requested_tcp = False
        # Sticky fast-path: if TCP telemetry worked before, request it up front so
        # we don't re-pay the UDP-silence wait on this (re)connection.
        if self._prefer_tcp:
            self.request_tcp_telemetry()
            requested_tcp = True
        while self.running:
            try:
                socks = [s for s in (self.socket, self.udp_socket) if s is not None]
                readable, _, _ = select.select(socks, [], [], 0.5)
            except Exception:
                return  # a socket was closed underneath us -> reconnect

            now = time.time()
            for s in readable:
                if s is self.udp_socket:
                    try:
                        datagram, _addr = s.recvfrom(65535)
                    except Exception:
                        continue
                    if datagram and len(datagram) >= 4:
                        self.last_rx_time = now
                        self._prefer_tcp = False  # UDP works here -> prefer it
                        msg_len = struct.unpack('>I', datagram[:4])[0]
                        self._parse_message(datagram[4:4 + msg_len])
                elif s is self.socket:
                    try:
                        data = s.recv(4096)
                    except Exception:
                        return
                    if not data:
                        logger.warning("SangamIO closed the TCP connection")
                        return
                    # TCP carries command traffic and, after a TCP-telemetry
                    # fallback, the sensor stream -- same [len][protobuf] framing.
                    tcp_buffer.extend(data)
                    while len(tcp_buffer) >= 4:
                        mlen = struct.unpack('>I', tcp_buffer[:4])[0]
                        if len(tcp_buffer) < 4 + mlen:
                            break
                        self.last_rx_time = now  # TCP telemetry feeds the watchdog
                        self._parse_message(bytes(tcp_buffer[4:4 + mlen]))
                        tcp_buffer = tcp_buffer[4 + mlen:]

            # Two-stage watchdog: first try a TCP-telemetry fallback, then give up.
            silent = now - self.last_rx_time
            if silent > self.udp_fallback_after and not requested_tcp:
                logger.info(
                    f"No telemetry for {silent:.1f}s; requesting TCP-telemetry fallback")
                self.request_tcp_telemetry()
                requested_tcp = True
                self._prefer_tcp = True
            elif silent > self.telemetry_timeout:
                logger.warning(
                    f"No telemetry for {self.telemetry_timeout:.1f}s; reconnecting")
                return

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
        elif field_num == 4 and wire_type == 0:  # i32 (proto int32: two's-complement varint)
            val, _ = self._read_varint(data, idx)
            # Negative int32 is varint-encoded sign-extended to 64 bits; decode signed.
            if val >= (1 << 63):
                val -= (1 << 64)
            return val
        elif field_num == 5 and wire_type == 0:  # i64 (proto int64)
            val, _ = self._read_varint(data, idx)
            if val >= (1 << 63):
                val -= (1 << 64)
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
        elif field_num == 9 and wire_type == 2:  # bytes (packed scan blob)
            length, idx = self._read_varint(data, idx)
            return bytes(data[idx:idx+length])

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

        # IMU - SangamIO sends each axis as a separate scalar (gyro_x/y/z,
        # accel_x/y/z, tilt_x/y/z), not a Vector3. See SangamIO gd32/reader.rs.
        if 'gyro_x' in values:
            self.sensor_data.gyro_x = values['gyro_x']
        if 'gyro_y' in values:
            self.sensor_data.gyro_y = values['gyro_y']
        if 'gyro_z' in values:
            self.sensor_data.gyro_z = values['gyro_z']

        if 'accel_x' in values:
            self.sensor_data.accel_x = values['accel_x']
        if 'accel_y' in values:
            self.sensor_data.accel_y = values['accel_y']
        if 'accel_z' in values:
            self.sensor_data.accel_z = values['accel_z']

        if 'tilt_x' in values:
            self.sensor_data.tilt_x = values['tilt_x']
        if 'tilt_y' in values:
            self.sensor_data.tilt_y = values['tilt_y']
        if 'tilt_z' in values:
            self.sensor_data.tilt_z = values['tilt_z']

    def _update_lidar(self, values: dict):
        """Update LiDAR data."""
        if 'scan_packed' in values and isinstance(
                values['scan_packed'], (bytes, bytearray)):
            self.sensor_data.lidar_points = self._unpack_packed_scan(
                values['scan_packed'])
        elif 'scan' in values and isinstance(values['scan'], list):  # legacy
            self.sensor_data.lidar_points = values['scan']
        if 'rpm' in values:
            self.sensor_data.lidar_rpm = float(values['rpm'])

    @staticmethod
    def _unpack_packed_scan(blob: bytes) -> list:
        """Unpack SangamIO's packed scan (see delta2d/mod.rs publish_scan):
        u16 num_bins, then per bin u16 distance (0.25mm units, 0 = no return) +
        u8 quality, little-endian. Bin i is angle i*2pi/num_bins. The raw->meters
        conversion (x0.00025) happens here on the PC, not on the robot."""
        if len(blob) < 2:
            return []
        num_bins = struct.unpack_from('<H', blob, 0)[0]
        if num_bins == 0:
            return []
        inc = 2.0 * math.pi / num_bins
        points = []
        off = 2
        for i in range(num_bins):
            if off + 3 > len(blob):
                break
            dist_raw, quality = struct.unpack_from('<HB', blob, off)
            off += 3
            if dist_raw > 0:
                points.append(LidarPoint(i * inc, dist_raw * 0.00025, quality))
        return points

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
        """Send a length-prefixed command over the TCP channel. Reconnect-safe:
        grabs the current socket and serializes concurrent sends."""
        sock = self.socket
        if sock is None:
            return False
        try:
            msg = struct.pack('>I', len(command_bytes)) + command_bytes
            with self.send_lock:
                sock.sendall(msg)
            return True
        except Exception as e:
            logger.warning(f"Send error: {e}")
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

    # Telemetry transport control (UDP is the low-latency LAN default; TCP is a
    # fallback for clients behind NAT / remote where UDP unicast can't get back).
    # "telemetry" is a reserved pseudo-component intercepted by SangamIO's TCP
    # receiver and not forwarded to the device driver.

    def request_tcp_telemetry(self) -> bool:
        """Ask SangamIO to stream sensor telemetry over this TCP connection
        instead of UDP. ENABLE = TCP."""
        return self.send_command(self._build_component_control("telemetry", 0))

    def request_udp_telemetry(self) -> bool:
        """Ask SangamIO to go back to UDP telemetry. DISABLE = UDP."""
        return self.send_command(self._build_component_control("telemetry", 1))
