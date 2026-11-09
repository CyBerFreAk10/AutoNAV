"""ROS 2 -> ESP32 serial bridge (runs on the Pi).

Sends single-character drive commands over USB serial:
  w forward   s back   a left   d right   x stop

Subscribes
  rover/cmd   std_msgs/String       'w' | 's' | 'a' | 'd' | 'x'
  cmd_vel     geometry_msgs/Twist   mapped to the same letters (works with teleop_twist_keyboard)
  tof/obstacle_detected, tof/cliff_detected (std_msgs/Bool)  only if use_tof_safety
Publishes
  rover/sent        std_msgs/String  every command actually written to the ESP32
  rover/esp32_rx    std_msgs/String  '#' status lines the ESP32 prints (for debugging)
  imu               sensor_msgs/Imu  parsed from the firmware's CSV stream
                    (t_ms,ax_mg,ay_mg,az_mg,gx_dps,gy_dps,gz_dps,temp_C,yaw_deg)
Safety
  - sends 'x' if no command arrives for watchdog_s (e.g. laptop Wi-Fi drops)
  - sends 'x' on shutdown
"""
import math
import time

import rclpy
import serial
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String

VALID = {'w', 's', 'a', 'd', 'x'}
NAMES = {'w': 'FORWARD', 's': 'BACK', 'a': 'LEFT', 'd': 'RIGHT', 'x': 'STOP'}


class Esp32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')
        p = lambda n, d: self.declare_parameter(n, d).value  # noqa: E731
        self.port = p('port', '/dev/ttyUSB0')
        self.baud = p('baud', 115200)
        self.eol = p('line_ending', '\\n').encode().decode('unicode_escape')
        self.watchdog_s = p('watchdog_s', 0.5)
        self.reset_on_open = p('reset_on_open', False)
        self.use_safety = p('use_tof_safety', False)
        self.deadband = p('cmd_vel_deadband', 0.05)
        self.imu_frame = p('imu_frame', 'imu_link')

        self.ser = None
        self.rx_buf = b''
        self.sent = None          # last char written
        self.last_cmd_time = 0.0
        self.obstacle = self.cliff = False

        self.pub_sent = self.create_publisher(String, 'rover/sent', 10)
        self.pub_rx = self.create_publisher(String, 'rover/esp32_rx', 10)
        self.pub_imu = self.create_publisher(Imu, 'imu', 50)
        self.create_subscription(String, 'rover/cmd', self._on_char, 10)
        self.create_subscription(Twist, 'cmd_vel', self._on_twist, 10)
        if self.use_safety:
            self.create_subscription(Bool, 'tof/obstacle_detected',
                                     lambda m: setattr(self, 'obstacle', m.data), 10)
            self.create_subscription(Bool, 'tof/cliff_detected',
                                     lambda m: setattr(self, 'cliff', m.data), 10)

        self._open()
        self.create_timer(0.05, self._tick)
        self.get_logger().info(
            f'Listening on rover/cmd and cmd_vel -> {self.port} @ {self.baud} '
            f'(watchdog {self.watchdog_s}s, ToF safety {"on" if self.use_safety else "off"})')

    # ------------------------------------------------------------------ serial
    def _open(self):
        try:
            s = serial.Serial()
            s.port, s.baudrate, s.timeout, s.write_timeout = self.port, self.baud, 0, 0.2
            if not self.reset_on_open:
                # ESP32 dev boards reboot when DTR/RTS toggle on open; keep them released.
                s.dtr = False
                s.rts = False
            s.open()
            self.ser = s
            self.sent = None
            self.get_logger().info(f'Opened {self.port}')
        except (serial.SerialException, OSError) as e:
            self.ser = None
            self.get_logger().error(f'Cannot open {self.port}: {e} (retrying)',
                                    throttle_duration_sec=5.0)

    def _write(self, c):
        if self.ser is None:
            return
        try:
            self.ser.write((c + self.eol).encode())
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'Serial write failed: {e}')
            self._close()
            return
        if c != self.sent:
            self.get_logger().info(f'-> {c} ({NAMES[c]})')
        self.sent = c
        self.pub_sent.publish(String(data=c))

    def _close(self):
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        self.ser = None

    # ------------------------------------------------------------------ commands
    def _command(self, c):
        self.last_cmd_time = time.monotonic()
        if c == 'w' and self.use_safety and (self.obstacle or self.cliff):
            self.get_logger().warn(
                f'Forward blocked: {"cliff" if self.cliff else "obstacle"} ahead',
                throttle_duration_sec=1.0)
            c = 'x'
        if c != self.sent:   # ESP32 keeps doing the last command, so only send changes
            self._write(c)

    def _on_char(self, msg):
        c = msg.data.strip().lower()[:1]
        if c in VALID:
            self._command(c)
        else:
            self.get_logger().warn(f'Ignoring unknown command "{msg.data}"')

    def _on_twist(self, msg):
        lin, ang = msg.linear.x, msg.angular.z
        if max(abs(lin), abs(ang)) < self.deadband:
            c = 'x'
        elif abs(ang) > abs(lin):
            c = 'a' if ang > 0 else 'd'
        else:
            c = 'w' if lin > 0 else 's'
        self._command(c)

    def _tick(self):
        if self.ser is None:
            self._open()
            return
        # watchdog
        if (self.watchdog_s > 0 and self.sent not in (None, 'x')
                and time.monotonic() - self.last_cmd_time > self.watchdog_s):
            self.get_logger().warn('No commands received - stopping (watchdog)')
            self._write('x')
        # safety while already driving forward
        if self.use_safety and self.sent == 'w' and (self.obstacle or self.cliff):
            self.get_logger().warn(f'{"Cliff" if self.cliff else "Obstacle"} - stopping')
            self._write('x')
        # read anything the ESP32 prints
        try:
            data = self.ser.read(256)
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'Serial read failed: {e}')
            self._close()
            return
        if data:
            self.rx_buf += data
            *lines, self.rx_buf = self.rx_buf.split(b'\n')
            for line in lines:
                text = line.decode(errors='replace').strip()
                if not text:
                    continue
                if text[0].isdigit():
                    self._publish_imu(text)      # CSV IMU sample - don't log
                else:
                    self.get_logger().info(f'ESP32: {text}')
                    self.pub_rx.publish(String(data=text))

    def _publish_imu(self, text):
        try:
            _, ax, ay, az, gx, gy, gz, _, yaw = (float(v) for v in text.split(','))
        except ValueError:
            return
        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.imu_frame
        g = 9.80665e-3   # mg -> m/s^2
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = ax * g, ay * g, az * g
        d = math.pi / 180.0
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = gx * d, gy * d, gz * d
        y = yaw * d      # integrated yaw only; roll/pitch unknown
        m.orientation.z, m.orientation.w = math.sin(y / 2), math.cos(y / 2)
        m.orientation_covariance = [1e3, 0.0, 0.0, 0.0, 1e3, 0.0, 0.0, 0.0, 0.05]
        m.angular_velocity_covariance[8] = 1e-4
        m.linear_acceleration_covariance[0] = m.linear_acceleration_covariance[4] = 0.05
        self.pub_imu.publish(m)

    def stop(self):
        if self.ser:
            self._write('x')
            self._close()


def main(args=None):
    rclpy.init(args=args)
    node = Esp32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
