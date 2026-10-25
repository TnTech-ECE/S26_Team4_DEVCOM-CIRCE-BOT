#!/usr/bin/env python3
"""
USB serial bridge between the Pi 5 and the CIRCE Teensy 4.0.

Teensy frames -> ROS topics (heartbeat, ultrasonic, GPS, e-stop, BMS) and ROS topics ->
Teensy frames (spool cmd, odom speed, e-stop reset, heartbeat). Every frame is logged.
The e-stop reaches the drive motors through this node. Protocol mirrors
include/protocol.h. Reconnects automatically if /dev/teensy goes away.
"""

import datetime
from pathlib import Path
import signal
import struct
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import BatteryState, NavSatFix, NavSatStatus, Range
import serial
from std_msgs.msg import Bool, UInt32

# --- Protocol: mirror of include/protocol.h ---
MAX_FRAME = 64
FRAME_OVERHEAD = 4

MSG_TEENSY_HEARTBEAT = 0x01
MSG_ULTRASONIC = 0x20
MSG_GPS_FIX = 0x30
MSG_ESTOP = 0x40
MSG_SPOOL_STATUS = 0x41
MSG_BMS_STATUS = 0x50
MSG_BMS_CELLS = 0x51
MSG_PI_HEARTBEAT = 0x02
MSG_SPOOL_CMD = 0x10
MSG_ODOM_SPEED = 0x11
MSG_ESTOP_RESET = 0x12

BMS_MAX_CELLS = 24

# msg_id -> (name, struct format, field names), Teensy -> Pi only.
INCOMING = {
    MSG_TEENSY_HEARTBEAT: (
        'TEENSY_HEARTBEAT', '<IIIHHIBfB',
        ('counter', 'uptime_ms', 'rx_ok', 'rx_bad', 'tx_dropped', 'last_pi_counter',
         'spool_enabled', 'last_speed', 'flags')),
    MSG_ULTRASONIC: (
        'ULTRASONIC', '<BBHH',
        ('sensor_id', 'status', 'distance_mm', 'bad_frames')),
    MSG_GPS_FIX: (
        'GPS_FIX', '<iiffffIHBBB',
        ('lat_e7', 'lon_e7', 'alt_m', 'hdop', 'speed_mps', 'course_deg', 'fix_age_ms',
         'bad_sentences', 'satellites', 'fix_quality', 'status')),
    MSG_ESTOP: (
        'ESTOP', '<BB',
        ('active', 'button_pressed')),
    MSG_SPOOL_STATUS: (
        'SPOOL_STATUS', '<fffB',
        ('cable_on_m', 'radius_m', 'target_rad_s', 'flags')),
    MSG_BMS_STATUS: (
        'BMS_STATUS', '<BHhHHHHHbbbBBBBB',
        ('bms_id', 'voltage_cv', 'current_ca', 'alarms', 'mode', 'cycles', 'capacity_ah',
         'bad_frames', 'temp_mos_c', 'temp_1_c', 'temp_2_c', 'soc_pct', 'cell_count',
         'battery_type', 'protocol_version', 'status')),
    MSG_BMS_CELLS: (
        'BMS_CELLS', f'<BB{BMS_MAX_CELLS}H',
        ('bms_id', 'count') + tuple(f'_cell{i}' for i in range(BMS_MAX_CELLS))),
}

ULTRASONIC_OK = 0
ULTRASONIC_OUT_OF_RANGE = 1
ULTRASONIC_TIMEOUT = 2
ULTRASONIC_MIN_M = 0.03
ULTRASONIC_MAX_M = 4.5

ENUMS = {
    ('ULTRASONIC', 'status'): {0: 'OK', 1: 'OUT_OF_RANGE', 2: 'TIMEOUT'},
    ('GPS_FIX', 'status'): {0: 'FIX', 1: 'NO_FIX', 2: 'NO_DATA'},
    ('BMS_STATUS', 'status'): {0: 'OK', 1: 'NO_DATA'},
    ('BMS_STATUS', 'battery_type'): {0: 'LiFePO4', 1: 'Li-ion', 2: 'LTO'},
}
GPS_STATUS_FIX = 0
BMS_OK = 0

# Bitmask names, bit 0 first.
HB_FLAGS = ('pi_alive', 'estop', 'spool_run')
SPOOL_FLAGS = ('run', 'at_reserve', 'full', 'saturated')
BMS_ALARMS = ('low_capacity', 'mos_overtemp', 'charge_overvoltage',
              'discharge_undervoltage', 'battery_overtemp', 'charge_overcurrent',
              'discharge_overcurrent', 'cell_imbalance', 'box_overtemp',
              'battery_undertemp', 'cell_overvoltage', 'cell_undervoltage',
              'protection_309_a', 'protection_309_b')
BMS_MODE = ('charge_on', 'discharge_on', 'balancing', 'battery_dropped')
BITS = {
    ('TEENSY_HEARTBEAT', 'flags'): HB_FLAGS,
    ('SPOOL_STATUS', 'flags'): SPOOL_FLAGS,
    ('BMS_STATUS', 'alarms'): BMS_ALARMS,
    ('BMS_STATUS', 'mode'): BMS_MODE,
}

# BMS alarm -> BatteryState health, most serious first.
BMS_HEALTH = (
    ({'mos_overtemp', 'battery_overtemp', 'box_overtemp'},
     BatteryState.POWER_SUPPLY_HEALTH_OVERHEAT),
    ({'charge_overvoltage', 'cell_overvoltage'},
     BatteryState.POWER_SUPPLY_HEALTH_OVERVOLTAGE),
    ({'discharge_undervoltage', 'cell_undervoltage'},
     BatteryState.POWER_SUPPLY_HEALTH_DEAD),
    ({'battery_undertemp'}, BatteryState.POWER_SUPPLY_HEALTH_COLD),
)
BMS_TECHNOLOGY = {0: BatteryState.POWER_SUPPLY_TECHNOLOGY_LIFE,
                  1: BatteryState.POWER_SUPPLY_TECHNOLOGY_LION}

FMT_PI_HEARTBEAT = '<I'
FMT_SPOOL_CMD = '<B'
FMT_ODOM_SPEED = '<f'


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else crc << 1
            crc &= 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    out = bytearray([0])
    code_idx = 0
    code = 1
    for b in data:
        if b == 0:
            out[code_idx] = code
            code_idx = len(out)
            out.append(0)
            code = 1
        else:
            out.append(b)
            code += 1
            if code == 0xFF:
                out[code_idx] = code
                code_idx = len(out)
                out.append(0)
                code = 1
    out[code_idx] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    out = bytearray()
    i = 0
    while i < len(data):
        code = data[i]
        i += 1
        if code == 0 or i + code - 1 > len(data):
            raise ValueError('malformed COBS block')
        out += data[i:i + code - 1]
        i += code - 1
        if code < 0xFF and i < len(data):
            out.append(0)
    return bytes(out)


def bit_names(value: int, names) -> list:
    out = [n for i, n in enumerate(names) if value & (1 << i)]
    unknown = value & ~((1 << len(names)) - 1)
    if unknown:
        out.append(f'0x{unknown:x}')
    return out


def build_frame(msg_id: int, seq: int, payload: bytes) -> bytes:
    body = bytes([msg_id, seq & 0xFF]) + payload
    return cobs_encode(body + struct.pack('<H', crc16(body))) + b'\x00'


class TeensyBridge(Node):

    def __init__(self):
        super().__init__('teensy_bridge')
        self._port_name = self.declare_parameter('port', '/dev/teensy').value
        odom_topic = self.declare_parameter('odom_topic', '/odometry/filtered').value
        heartbeat_hz = self.declare_parameter('heartbeat_hz', 1.0).value
        log_file = self.declare_parameter('log_file', '').value
        self._gps_frame = self.declare_parameter('gps_frame', 'gps_link').value
        self._gps_sigma_per_hdop = self.declare_parameter('gps_sigma_per_hdop', 2.5).value
        us_topics = self.declare_parameter(
            'ultrasonic_topics', ['/ultrasonic/sensor_0', '/ultrasonic/sensor_1']).value
        self._us_frames = self.declare_parameter(
            'ultrasonic_frames', ['ultrasonic_0_link', 'ultrasonic_1_link']).value
        self._us_fov = self.declare_parameter('ultrasonic_fov_rad', 1.047).value
        if len(us_topics) != len(self._us_frames):
            raise ValueError('ultrasonic_topics and ultrasonic_frames differ in length')
        battery_topics = self.declare_parameter(
            'battery_topics', ['/battery/bms_0', '/battery/bms_1']).value
        self._battery_frame = self.declare_parameter('battery_frame', 'base_link').value
        estop_trigger_topic = self.declare_parameter(
            'estop_trigger_topic', '/soft_estop/trigger').value
        estop_reset_topic = self.declare_parameter(
            'estop_reset_topic', '/soft_estop/reset').value
        self._teensy_timeout = self.declare_parameter('teensy_timeout_s', 3.0).value

        if not log_file:
            stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            log_file = f'~/circe_logs/teensy_rx_{stamp}.txt'
        log_path = Path(log_file).expanduser()
        log_path.parent.mkdir(parents=True, exist_ok=True)
        # Line-buffered so the log survives the node being killed.
        self._log = open(log_path, 'a', buffering=1)
        self.get_logger().info(f'Logging Teensy frames to {log_path}')

        self._port = None
        self._rx = bytearray()
        self._synced = False
        self._expected_seq = None
        self._tx_seq = 0
        self._pi_counter = 0
        self._connected = None
        self._last_teensy_hb = None
        self._estop_active = None
        self._last_estop_trigger = 0.0
        self._bms_cells = [[] for _ in battery_topics]
        self._bms_alarms = [0 for _ in battery_topics]

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub_connected = self.create_publisher(Bool, '/teensy/connected', latched)
        self._pub_heartbeat = self.create_publisher(UInt32, '/teensy/heartbeat', 10)
        self._pub_gps = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self._pub_us = [self.create_publisher(Range, t, 10) for t in us_topics]
        self._pub_estop = self.create_publisher(Bool, '/teensy/estop', latched)
        self._pub_estop_trigger = self.create_publisher(Bool, estop_trigger_topic, 10)
        self._pub_battery = [self.create_publisher(BatteryState, t, 10)
                             for t in battery_topics]
        self._set_connected(False)

        self.create_subscription(Bool, '/spool/cmd', self._on_spool_cmd, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_subscription(Bool, estop_reset_topic, self._on_estop_reset, 10)

        self.create_timer(0.01, self._poll)
        self.create_timer(1.0 / heartbeat_hz, self._send_heartbeat)
        self.create_timer(1.0, self._connect)
        self._connect()


    def _set_connected(self, connected):
        if connected != self._connected:
            self._connected = connected
            self._pub_connected.publish(Bool(data=connected))

    def _connect(self):
        if self._port is not None:
            # Port open but firmware silent.
            stale = (self._last_teensy_hb is None or
                     time.monotonic() - self._last_teensy_hb > self._teensy_timeout)
            if stale and self._connected:
                self._write_log('TEENSY_SILENT')
                self.get_logger().warning(
                    f'{self._port_name} open but no Teensy heartbeat for '
                    f'{self._teensy_timeout:.0f} s')
            if stale:
                self._set_connected(False)
            return
        try:
            # Non-blocking reads; write_timeout keeps a stuck Teensy from stalling us.
            self._port = serial.Serial(self._port_name, timeout=0, write_timeout=0.05)
        except (serial.SerialException, OSError):
            return
        self._port.reset_input_buffer()
        self._rx.clear()
        self._synced = False
        self._expected_seq = None
        self._last_teensy_hb = None
        self._write_log(f'CONNECTED {self._port_name}')
        self.get_logger().info(f'Connected to {self._port_name}')

    def _disconnect(self, reason):
        try:
            self._port.close()
        except (serial.SerialException, OSError):
            pass
        self._port = None
        self._set_connected(False)
        self._write_log(f'DISCONNECTED {reason}')
        self.get_logger().warning(f'Lost {self._port_name}: {reason}; retrying every 1 s')


    def _poll(self):
        if self._port is None:
            return
        try:
            data = self._port.read(self._port.in_waiting or 1)
        except (serial.SerialException, OSError) as e:
            self._disconnect(e)
            return
        if not data:
            return
        self._rx += data
        while True:
            end = self._rx.find(b'\x00')
            if end < 0:
                break
            encoded = bytes(self._rx[:end])
            del self._rx[:end + 1]
            if not self._synced:
                # Skip the partial frame before the first delimiter.
                self._synced = True
                continue
            if encoded:
                self._handle_encoded(encoded)
        if len(self._rx) > 2 * MAX_FRAME:
            self._write_log(f'BAD_OVERFLOW len={len(self._rx)}')
            self._rx.clear()
            self._synced = False

    def _handle_encoded(self, encoded):
        try:
            frame = cobs_decode(encoded)
        except ValueError:
            self._write_log(f'BAD_COBS raw={encoded.hex()}')
            return
        if len(frame) < FRAME_OVERHEAD or len(frame) > MAX_FRAME:
            self._write_log(f'BAD_LENGTH len={len(frame)} raw={frame.hex()}')
            return
        body, (crc,) = frame[:-2], struct.unpack('<H', frame[-2:])
        if crc16(body) != crc:
            self._write_log(f'BAD_CRC raw={frame.hex()}')
            return

        msg_id, seq, payload = body[0], body[1], body[2:]
        if self._expected_seq is not None and seq != self._expected_seq:
            self._write_log(f'DROPPED {(seq - self._expected_seq) & 0xFF}')
        self._expected_seq = (seq + 1) & 0xFF

        if msg_id not in INCOMING:
            self._write_log(f'UNKNOWN id=0x{msg_id:02x} seq={seq} payload={payload.hex()}')
            return
        name, fmt, fields = INCOMING[msg_id]
        if len(payload) != struct.calcsize(fmt):
            self._write_log(f'BAD_PAYLOAD {name} seq={seq} len={len(payload)}')
            return
        raw = struct.unpack(fmt, payload)
        if msg_id == MSG_BMS_CELLS:
            count = min(raw[1], BMS_MAX_CELLS)
            values = {'bms_id': raw[0], 'count': count, 'cell_mv': list(raw[2:2 + count])}
        else:
            values = dict(zip(fields, raw))
        text = ' '.join(f'{k}={self._format(name, k, v)}' for k, v in values.items())
        self._write_log(f'{name} seq={seq} {text}')

        if msg_id == MSG_TEENSY_HEARTBEAT:
            self._last_teensy_hb = time.monotonic()
            self._set_connected(True)
            self._pub_heartbeat.publish(UInt32(data=values['counter']))
        elif msg_id == MSG_GPS_FIX:
            self._publish_gps(values)
        elif msg_id == MSG_ULTRASONIC:
            self._publish_ultrasonic(values)
        elif msg_id == MSG_ESTOP:
            self._handle_estop(values)
        elif msg_id == MSG_BMS_CELLS:
            if self._check_bms_id(values['bms_id']):
                self._bms_cells[values['bms_id']] = values['cell_mv']
        elif msg_id == MSG_BMS_STATUS:
            if self._check_bms_id(values['bms_id']):
                self._publish_battery(values)

    def _handle_estop(self, v):
        active = bool(v['active'])
        if active != self._estop_active:
            self._pub_estop.publish(Bool(data=active))
            if active:
                self.get_logger().error(
                    'E-STOP latched on the Teensy'
                    + (' (button down or wire broken)' if v['button_pressed'] else ''))
            elif self._estop_active is not None:
                self.get_logger().info('E-stop cleared on the Teensy')
        now = time.monotonic()
        # Re-trigger every second while latched.
        if active and (not self._estop_active or now - self._last_estop_trigger >= 1.0):
            self._pub_estop_trigger.publish(Bool(data=True))
            self._last_estop_trigger = now
        self._estop_active = active

    def _check_bms_id(self, bid):
        if bid < len(self._pub_battery):
            return True
        self.get_logger().warning(
            f'BMS frame from bms_id {bid}, but battery_topics has only '
            f'{len(self._pub_battery)} entries', throttle_duration_sec=10.0)
        return False

    def _publish_battery(self, v):
        bid = v['bms_id']
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._battery_frame
        nan = float('nan')
        msg.charge = msg.capacity = nan
        if v['status'] != BMS_OK:
            msg.present = False
            msg.voltage = msg.current = msg.temperature = msg.percentage = nan
            msg.design_capacity = nan
            self._pub_battery[bid].publish(msg)
            return

        alarms = v['alarms']
        if alarms != self._bms_alarms[bid]:
            names = bit_names(alarms, BMS_ALARMS)
            self.get_logger().warning(
                f'BMS {bid} alarms: {", ".join(names) or "cleared"}')
            self._bms_alarms[bid] = alarms

        msg.present = True
        msg.voltage = v['voltage_cv'] / 100.0
        msg.current = v['current_ca'] / 100.0
        msg.temperature = float(max(v['temp_1_c'], v['temp_2_c']))
        msg.percentage = v['soc_pct'] / 100.0
        msg.design_capacity = float(v['capacity_ah']) if v['capacity_ah'] else nan
        msg.power_supply_technology = BMS_TECHNOLOGY.get(
            v['battery_type'], BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN)
        if msg.current > 0.05:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        elif msg.current < -0.05:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        elif v['soc_pct'] >= 100:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
        else:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING

        set_alarms = set(bit_names(alarms, BMS_ALARMS)) - {'low_capacity'}
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        if set_alarms:
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNSPEC_FAILURE
            for bits, health in BMS_HEALTH:
                if set_alarms & bits:
                    msg.power_supply_health = health
                    break
        msg.cell_voltage = [mv / 1000.0 for mv in self._bms_cells[bid]]
        self._pub_battery[bid].publish(msg)

    def _publish_ultrasonic(self, v):
        sid = v['sensor_id']
        if sid >= len(self._pub_us):
            self.get_logger().warning(
                f'ULTRASONIC from sensor_id {sid}, but ultrasonic_topics has only '
                f'{len(self._pub_us)} entries', throttle_duration_sec=10.0)
            return
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._us_frames[sid]
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = self._us_fov
        msg.min_range = ULTRASONIC_MIN_M
        msg.max_range = ULTRASONIC_MAX_M
        distance = v['distance_mm'] / 1000.0
        status = v['status']
        if status == ULTRASONIC_OK:
            msg.range = distance
            msg.variance = (0.01 + 0.003 * distance) ** 2
        elif status == ULTRASONIC_OUT_OF_RANGE:
            msg.range = float('-inf') if distance < ULTRASONIC_MIN_M else float('inf')
        else:
            # Sensor silent: NaN so it can't read as a clear path.
            msg.range = float('nan')
        self._pub_us[sid].publish(msg)

    def _publish_gps(self, v):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._gps_frame
        msg.status.service = (NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GLONASS
                              | NavSatStatus.SERVICE_GALILEO)
        if v['status'] != GPS_STATUS_FIX:
            msg.status.status = NavSatStatus.STATUS_NO_FIX
            msg.latitude = msg.longitude = msg.altitude = float('nan')
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            self._pub_gps.publish(msg)
            return
        msg.status.status = (NavSatStatus.STATUS_SBAS_FIX if v['fix_quality'] == 2
                             else NavSatStatus.STATUS_FIX)
        msg.latitude = v['lat_e7'] / 1e7
        msg.longitude = v['lon_e7'] / 1e7
        # MSL altitude, not ellipsoid.
        msg.altitude = float(v['alt_m'])
        sigma = max(v['hdop'], 0.5) * self._gps_sigma_per_hdop
        msg.position_covariance = [sigma ** 2, 0.0, 0.0,
                                   0.0, sigma ** 2, 0.0,
                                   0.0, 0.0, (2.0 * sigma) ** 2]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self._pub_gps.publish(msg)

    @staticmethod
    def _format(name, field, value):
        if (name, field) in ENUMS:
            return ENUMS[(name, field)].get(value, value)
        if (name, field) in BITS:
            return '|'.join(bit_names(value, BITS[(name, field)])) or '-'
        if isinstance(value, list):
            return ','.join(str(x) for x in value)
        if field.endswith('_e7'):
            return f'{value / 1e7:.7f}'
        return f'{value:.3f}' if isinstance(value, float) else str(value)


    def _send(self, msg_id, payload):
        if self._port is None:
            return
        try:
            self._port.write(build_frame(msg_id, self._tx_seq, payload))
        except serial.SerialTimeoutException:
            self.get_logger().warning('Teensy not reading; dropped a frame',
                                      throttle_duration_sec=5.0)
            return
        except (serial.SerialException, OSError) as e:
            self._disconnect(e)
            return
        self._tx_seq = (self._tx_seq + 1) & 0xFF

    def _send_heartbeat(self):
        self._send(MSG_PI_HEARTBEAT, struct.pack(FMT_PI_HEARTBEAT, self._pi_counter))
        self._pi_counter = (self._pi_counter + 1) & 0xFFFFFFFF

    def _on_spool_cmd(self, msg):
        self._send(MSG_SPOOL_CMD, struct.pack(FMT_SPOOL_CMD, 1 if msg.data else 0))

    def _on_odom(self, msg):
        self._send(MSG_ODOM_SPEED, struct.pack(FMT_ODOM_SPEED, msg.twist.twist.linear.x))

    def _on_estop_reset(self, msg):
        if msg.data:
            self._send(MSG_ESTOP_RESET, b'')


    def _write_log(self, line):
        stamp = datetime.datetime.now().isoformat(timespec='milliseconds')
        self._log.write(f'{stamp} {line}\n')

    def destroy_node(self):
        if self._port is not None:
            self._port.close()
        self._log.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = TeensyBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Ignore a second Ctrl-C during shutdown.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
