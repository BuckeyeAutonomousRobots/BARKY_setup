"""Exercise bridge logic with mocked ROS and serial; not a ROS integration test."""
import importlib
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/nova_hardware'))
from nova_hardware.protocol import encode, decode


class Message:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


class FakeImu:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None, frame_id='')
        self.orientation_covariance = [0.0] * 9
        self.linear_acceleration = types.SimpleNamespace(x=0, y=0, z=0)
        self.angular_velocity = types.SimpleNamespace(x=0, y=0, z=0)


class FakeNode:
    def __init__(self, name):
        self.params = {}
        self.log = Mock()

    def declare_parameter(self, key, value): self.params[key] = value
    def get_parameter(self, key): return types.SimpleNamespace(value=self.params[key])
    def create_publisher(self, *args): return Mock()
    def create_subscription(self, *args): pass
    def create_service(self, *args): pass
    def create_timer(self, *args): pass
    def get_logger(self): return self.log
    def get_clock(self): return Mock()


class FakeSerial:
    def __init__(self, *args, **kwargs):
        self.input = b''
        self.written = []
        self.fail = False
        self.closed = False

    @property
    def in_waiting(self): return len(self.input)
    def reset_input_buffer(self): self.input = b''
    def read(self, count):
        if self.fail: raise OSError('unplugged')
        result, self.input = self.input[:count], self.input[count:]
        return result
    def write(self, frame):
        self.written.append(frame)
        return len(frame)
    def close(self): self.closed = True


def module(**attrs):
    value = types.ModuleType('test_stub')
    value.__dict__.update(attrs)
    return value


class BridgeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        stubs = {
            'rclpy': module(), 'rclpy.node': module(Node=FakeNode),
            'rclpy.qos': module(qos_profile_sensor_data=object()),
            'sensor_msgs': module(), 'sensor_msgs.msg': module(Imu=FakeImu),
            'std_msgs': module(), 'std_msgs.msg': module(Float64MultiArray=Message, String=Message),
            'std_srvs': module(), 'std_srvs.srv': module(Trigger=Message),
            'serial': module(Serial=FakeSerial, SerialException=OSError),
        }
        cls.modules = patch.dict(sys.modules, stubs)
        cls.modules.start()
        cls.bridge = importlib.import_module('nova_hardware.bridge')

    @classmethod
    def tearDownClass(cls):
        sys.modules.pop('nova_hardware.bridge', None)
        cls.modules.stop()

    def setUp(self):
        self.clock = patch.object(self.bridge.time, 'monotonic', return_value=10.0)
        self.time = self.clock.start()
        self.addCleanup(self.clock.stop)
        self.node = self.bridge.HardwareBridge()
        self.node.poll()

    def feed(self, payload):
        self.node.device.input += encode(payload)
        self.node.poll()

    def ready(self): self.feed('S,1,1000,1,1,0,0,0,0')

    def test_fresh_status_required(self):
        self.assertFalse(self.node.send_targets([0.0] * 12)[0])
        self.ready()
        self.assertTrue(self.node.send_targets([0.0] * 12)[0])
        self.time.return_value = 11.0
        self.assertFalse(self.node.send_targets([0.0] * 12)[0])

    def test_roundtrip_ack(self):
        self.ready()
        self.node.send_targets([0.25] * 12)
        frame = decode(self.node.device.written[-1])
        self.feed(f'A,{frame[1]},DRY_RUN')
        self.assertFalse(self.node.pending)
        self.assertEqual(self.node.last_ack['result'], 'DRY_RUN')

    def test_disconnect_clears_pending_no_replay(self):
        self.ready()
        self.node.send_targets([0.0] * 12)
        old = self.node.device
        old.fail = True
        self.node.poll()
        self.assertTrue(old.closed)
        self.assertIsNone(self.node.device)
        self.assertFalse(self.node.pending)
        self.time.return_value = 13.0
        self.node.poll()
        self.assertTrue(all(decode(f)[0] == 'H' for f in self.node.device.written))

    def test_reset_clears_pending(self):
        self.ready()
        self.node.send_targets([0.0] * 12)
        self.feed('S,1,10,1,1,0,0,0,0')
        self.assertEqual(self.node.reset_count, 1)
        self.assertFalse(self.node.pending)

    def test_imu_and_invalid_data(self):
        self.feed('I,1,0,0,9.80665,0,0,0')
        msg = self.node.imu_pub.publish.call_args.args[0]
        self.assertEqual(msg.orientation_covariance[0], -1.0)
        self.assertAlmostEqual(msg.linear_acceleration.z, 9.80665)
        self.feed('I,2,nan,0,0,0,0,0')
        self.assertEqual(self.node.imu_pub.publish.call_count, 1)
        self.assertEqual(self.node.schema_errors, 1)

    def test_timeout_is_reported(self):
        self.ready()
        self.node.send_targets([0.0] * 12)
        self.time.return_value = 11.1
        self.node.poll()
        self.assertFalse(self.node.pending)
        self.assertEqual(self.node.ack_timeouts, 1)
