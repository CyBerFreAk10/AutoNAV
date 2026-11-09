"""ROS 2 driver node for the ST VL53L5CX multizone ToF sensor (I2C).

Publishes
  tof/distance_mm  std_msgs/Int16MultiArray   rows x cols, mm, -1 = invalid zone
  tof/status       std_msgs/UInt8MultiArray   rows x cols, raw ST target status
  tof/depth        sensor_msgs/Image          32FC1, metres, NaN = invalid (REP 118)
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray, UInt8MultiArray, MultiArrayDimension

import vl53l5cx_ctypes as vl53
from smbus2 import SMBus


def _layout(msg, rows, cols):
    msg.layout.dim = [
        MultiArrayDimension(label='rows', size=rows, stride=rows * cols),
        MultiArrayDimension(label='cols', size=cols, stride=cols),
    ]
    msg.layout.data_offset = 0
    return msg


class VL53L5CXNode(Node):
    def __init__(self):
        super().__init__('vl53l5cx_node')
        p = self.declare_parameter
        bus = p('i2c_bus', 1).value
        addr = p('i2c_address', 0x29).value
        res = p('resolution', 8).value
        freq = p('frequency_hz', 15).value
        self.frame_id = p('frame_id', 'tof_link').value
        sharpener = p('sharpener_percent', 5).value
        order = p('target_order', 'strongest').value
        self.valid = np.array(p('valid_statuses', [5, 6, 9]).value, dtype=np.uint8)
        self.flip_h = p('flip_horizontal', False).value
        self.flip_v = p('flip_vertical', False).value

        if res not in (4, 8):
            raise ValueError('resolution must be 4 or 8')
        max_hz = 15 if res == 8 else 60
        if not 1 <= freq <= max_hz:
            self.get_logger().warn(f'frequency_hz {freq} out of range for {res}x{res}; clamping')
            freq = min(max(freq, 1), max_hz)
        self.n = res

        self.get_logger().info(
            f'Initialising VL53L5CX on /dev/i2c-{bus} @ 0x{addr:02x} (firmware upload, ~2-10 s)...')
        self.sensor = vl53.VL53L5CX(i2c_addr=addr, i2c_dev=SMBus(bus))
        self.sensor.set_resolution(res * res)
        self.sensor.set_ranging_frequency_hz(freq)
        self.sensor.set_sharpener_percent(sharpener)
        self.sensor.set_target_order(
            vl53.TARGET_ORDER_CLOSEST if order == 'closest' else vl53.TARGET_ORDER_STRONGEST)
        self.sensor.set_ranging_mode(vl53.RANGING_MODE_CONTINUOUS)
        self.sensor.start_ranging()
        self.get_logger().info(f'Ranging {res}x{res} @ {freq} Hz')

        self.pub_dist = self.create_publisher(Int16MultiArray, 'tof/distance_mm', 10)
        self.pub_status = self.create_publisher(UInt8MultiArray, 'tof/status', 10)
        self.pub_depth = self.create_publisher(Image, 'tof/depth', 10)

        # Poll a few times faster than the ranging rate.
        self.timer = self.create_timer(1.0 / (freq * 3), self._poll)

    def _orient(self, a):
        if self.flip_h:
            a = np.fliplr(a)
        if self.flip_v:
            a = np.flipud(a)
        return np.ascontiguousarray(a)

    def _poll(self):
        try:
            if not self.sensor.data_ready():
                return
            data = self.sensor.get_data()
        except (OSError, RuntimeError) as e:
            self.get_logger().warn(f'I2C read failed: {e}', throttle_duration_sec=2.0)
            return

        stamp = self.get_clock().now().to_msg()
        cells = self.n * self.n
        dist = np.ctypeslib.as_array(data.distance_mm[0])[:cells].astype(np.int16)
        status = np.ctypeslib.as_array(data.target_status[0])[:cells].astype(np.uint8)
        dist = self._orient(dist.reshape(self.n, self.n))
        status = self._orient(status.reshape(self.n, self.n))
        ok = np.isin(status, self.valid)

        d = np.where(ok, dist, -1).astype(np.int16)
        m = _layout(Int16MultiArray(), self.n, self.n)
        m.data = d.flatten().tolist()
        self.pub_dist.publish(m)

        s = _layout(UInt8MultiArray(), self.n, self.n)
        s.data = status.flatten().tobytes()
        self.pub_status.publish(s)

        depth = np.where(ok, dist / 1000.0, math.nan).astype(np.float32)
        img = Image()
        img.header.stamp = stamp
        img.header.frame_id = self.frame_id
        img.height = img.width = self.n
        img.encoding = '32FC1'
        img.is_bigendian = 0
        img.step = self.n * 4
        img.data = depth.tobytes()
        self.pub_depth.publish(img)

    def destroy_node(self):
        try:
            self.sensor.stop_ranging()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VL53L5CXNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
