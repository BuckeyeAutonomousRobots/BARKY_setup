"""ROS2 bridge for the stage-1, permanently non-actuating Nano firmware."""
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger
import serial

from .protocol import Framer, command, encode, telemetry


class HardwareBridge(Node):
    def __init__(self):
        super().__init__('hardware_bridge_node')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('frame_id', 'imu_link')
        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.frame_id = self.get_parameter('frame_id').value
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', qos_profile_sensor_data)
        self.status_pub = self.create_publisher(String, '/hardware/status', 10)
        self.ack_pub = self.create_publisher(String, '/hardware/test_ack', 10)
        self.create_subscription(Float64MultiArray, '/hardware/test_targets', self.targets, 10)
        self.create_service(Trigger, '/hardware/send_test', self.send_test)
        self.device = None
        self.framer = Framer()
        self.sequence = 0
        self.last_attempt = -float('inf')
        self.last_heartbeat = -float('inf')
        self.last_rx = self.last_imu = self.last_status = None
        self.nano = None
        self.last_uptime = None
        self.reset_count = 0
        self.schema_errors = 0
        self.last_error = ''
        self.pending = {}  # sequence -> local monotonic send time
        self.ack_timeouts = 0
        self.last_ack = None
        self.create_timer(0.005, self.poll)
        self.create_timer(0.2, self.publish_status)
        self.get_logger().info('Stage 1: dry-run commands only; servo output is absent.')

    def next_seq(self):
        self.sequence = (self.sequence + 1) & 0xFFFFFFFF
        return self.sequence

    def disconnect(self, reason):
        if self.device is not None:
            try:
                self.device.close()
            except (OSError, serial.SerialException):
                pass
        self.device = None
        self.last_error = str(reason)
        self.framer = Framer()
        self.last_rx = self.last_imu = self.last_status = None
        self.nano = None
        self.last_uptime = None
        self.pending.clear()  # No command replay on reconnection.

    def write_frame(self, frame):
        if self.device is None:
            return False
        try:
            if self.device.write(frame) != len(frame):
                raise serial.SerialException('partial write')
            return True
        except (OSError, serial.SerialException) as exc:
            self.disconnect(exc)
            return False

    def poll(self):
        now = time.monotonic()
        if self.device is None:
            if now - self.last_attempt < 2.0:
                return
            self.last_attempt = now
            try:
                self.device = serial.Serial(self.port, self.baud, timeout=0,
                                            write_timeout=0.02, exclusive=True)
                self.device.reset_input_buffer()
                self.last_heartbeat = -float('inf')
                self.last_error = ''
                self.get_logger().info(f'Opened {self.port}; waiting for Nano status.')
            except (OSError, serial.SerialException) as exc:
                self.disconnect(exc)
                return
        try:
            raw = self.device.read(min(self.device.in_waiting, 4096))
        except (OSError, serial.SerialException) as exc:
            self.disconnect(exc)
            return
        for fields in self.framer.feed(raw):
            try:
                kind, data = telemetry(fields)
            except ValueError:
                self.schema_errors += 1
                continue
            self.last_rx = now
            if kind == 'I':
                self.last_imu = now
                msg = Imu()
                # Receipt timestamp, not synchronized Nano sample time.
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.frame_id
                msg.orientation_covariance[0] = -1.0
                # All-zero accel/gyro covariance means unknown, per sensor_msgs.
                _, values = data
                msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = values[:3]
                msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = values[3:]
                self.imu_pub.publish(msg)
            elif kind == 'S':
                uptime, imu_ok, alive, have_cmd, last_seq, errors, drops = data
                # Modular comparison distinguishes rollover from typical resets.
                if self.last_uptime is not None and ((uptime - self.last_uptime) & 0xFFFFFFFF) > 0x7FFFFFFF:
                    self.reset_count += 1
                    self.pending.clear()
                    self.last_imu = None
                    self.last_ack = None
                self.last_uptime = uptime
                self.last_status = now
                self.nano = dict(protocol=1, uptime_ms=uptime, imu_initialized=bool(imu_ok),
                                 host_heartbeat_alive=bool(alive), have_test_command=bool(have_cmd),
                                 last_test_sequence=last_seq, rx_errors=errors, tx_drops=drops)
                if have_cmd:
                    self.acknowledge(last_seq, 'status', now)
            else:
                self.acknowledge(data, 'ack_packet', now)
        # H is a link heartbeat, not a motion command or motion watchdog refresh.
        if now - self.last_heartbeat >= 0.1:
            self.last_heartbeat = now
            self.write_frame(encode(f'H,{self.next_seq()}'))
        for seq, sent in list(self.pending.items()):
            if now - sent > 1.0:
                del self.pending[seq]
                self.ack_timeouts += 1
                self.get_logger().warning(f'Test command {seq}: no acknowledgement in 1 s')

    def acknowledge(self, seq, source, now):
        if seq not in self.pending:
            return
        latency = now - self.pending.pop(seq)
        self.last_ack = dict(sequence=seq, result='DRY_RUN', source=source,
                             round_trip_ms=round(latency * 1000, 2), servo_output=False)
        self.ack_pub.publish(String(data=json.dumps(self.last_ack)))
        self.get_logger().info(f'Test command {seq} acknowledged; no servo motion.')

    def send_targets(self, values):
        now = time.monotonic()
        if self.last_status is None or now - self.last_status > 0.6:
            return False, 'No fresh Nano status; command not queued.'
        if len(self.pending) >= 8:
            return False, 'Too many pending test commands; wait for acknowledgements.'
        seq = self.next_seq()
        try:
            frame = command(seq, values)
        except ValueError as exc:
            return False, str(exc)
        if not self.write_frame(frame):
            return False, 'Serial write failed; command not queued for retry.'
        self.pending[seq] = now
        return True, f'Sent dry-run sequence {seq}; check /hardware/test_ack for receipt.'

    def targets(self, msg):
        success, detail = self.send_targets(msg.data)
        if not success:
            self.get_logger().warning(detail)

    def send_test(self, request, response):
        del request
        response.success, response.message = self.send_targets([0.0] * 12)
        return response

    def publish_status(self):
        now = time.monotonic()
        def age(stamp):
            return None if stamp is None else round(now - stamp, 3)
        status_age = age(self.last_status)
        imu_age = age(self.last_imu)
        state = dict(stage='transport_test', servo_output=False, armed=False,
                     serial_open=self.device is not None,
                     status_fresh=status_age is not None and status_age < 0.6,
                     imu_fresh=imu_age is not None and imu_age < 0.2,
                     telemetry_age_s=age(self.last_rx), status_age_s=status_age, imu_age_s=imu_age,
                     nano=self.nano, detected_resets=self.reset_count,
                     frame_errors=self.framer.errors, schema_errors=self.schema_errors,
                     pending_test_commands=len(self.pending), ack_timeouts=self.ack_timeouts,
                     last_test_ack=self.last_ack, last_error=self.last_error)
        self.status_pub.publish(String(data=json.dumps(state, allow_nan=False)))


def main(args=None):
    rclpy.init(args=args)
    node = HardwareBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.disconnect('bridge shutdown')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
