"""ROS 2 -> ESP32 serial bridge (runs on the Pi).

Sends single-character drive commands over USB serial:
  w forward   s back   a left   d right   x stop

Subscribes
  rover/cmd   std_msgs/String       'w' | 's' | 'a' | 'd' | 'x'
  cmd_vel     geometry_msgs/Twist   mapped to the same letters (works with teleop_twist_keyboard)
  tof/obstacle_detected, tof/cliff_detected (std_msgs/Bool)  only if use_tof_safety:
    forward ('w') is blocked / stopped while either is true, or while no ToF update has
    arrived for tof_timeout_s (fail-safe). Backing up and turning stay allowed.
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
import os
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
        # /dev/rover_esp32 follows the physical USB port (udev rule from setup_serial_ports);
        # ttyUSB numbers swap between the ESP32 and the lidar when the USB devices re-enumerate.
        self.port = p('port', '/dev/rover_esp32' if os.path.exists('/dev/rover_esp32')
                      else '/dev/ttyUSB0')
        self.baud = p('baud', 115200)
        self.eol = p('line_ending', '\\n').encode().decode('unicode_escape')
        self.watchdog_s = p('watchdog_s', 0.5)
        self.reset_on_open = p('reset_on_open', False)
        self.use_safety = p('use_tof_safety', True)
        self.tof_timeout_s = p('tof_timeout_s', 1.0)   # no ToF data this long = unsafe
        self.deadband = p('cmd_vel_deadband', 0.05)
        self.imu_frame = p('imu_frame', 'imu_link')
        self.drive_pwm = int(p('drive_pwm', 0))   # 0 = keep the firmware default (200)
        self.fw_watchdog_ms = int(p('esp32_watchdog_ms', 1000))   # ESP32 stops by itself; 0 = off
        self.heartbeat_s = p('heartbeat_s', 0.3)
        self.last_write = 0.0
        self.last_rx_line = ''

        self.ser = None
        self.rx_buf = b''
        self.sent = None          # last char written
        self.last_cmd_time = 0.0
        self.obstacle = self.cliff = False
        self.last_tof = 0.0

        self.pub_sent = self.create_publisher(String, 'rover/sent', 10)
        self.pub_rx = self.create_publisher(String, 'rover/esp32_rx', 10)
        self.pub_imu = self.create_publisher(Imu, 'imu', 50)
        self.create_subscription(String, 'rover/cmd', self._on_char, 10)
        self.create_subscription(Twist, 'cmd_vel', self._on_twist, 10)
        if self.use_safety:
            self.create_subscription(Bool, 'tof/obstacle_detected', self._on_obstacle, 10)
            self.create_subscription(Bool, 'tof/cliff_detected', self._on_cliff, 10)

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
            if self.reset_on_open:
                # Toggling DTR/RTS on open triggers the ESP32 auto-reset circuit (reboot +
                # ~22 s IMU calibration). Left untouched, the port opens without a reboot.
                s.dtr = False
                s.rts = False
            s.open()
            self.ser = s
            self.sent = None
            self.get_logger().info(f'Opened {self.port}')
            self._send_setup()
        except (serial.SerialException, OSError) as e:
            self.ser = None
            self.get_logger().error(f'Cannot open {self.port}: {e} (retrying)',
                                    throttle_duration_sec=5.0)

    def _send_setup(self):
        # Settings the firmware forgets when it reboots:
        #  - 'v': speed used by w/a/s/d
        #  - 'wd': the ESP32 stops the motors by itself if no motor command arrives in time.
        #    The bridge re-sends the current command every heartbeat_s, so if the Pi, this node
        #    or the USB link dies, the rover stops within esp32_watchdog_ms.
        if self.ser is None:
            return
        try:
            if self.drive_pwm > 0:
                self.ser.write(f'\nv {max(0, min(255, self.drive_pwm))}\n'.encode())
            self.ser.write(f'\nwd {max(0, self.fw_watchdog_ms)}\n'.encode())
        except (serial.SerialException, OSError):
            pass

    def _write(self, c):
        if self.ser is None:
            return
        try:
            # Leading newline: the ESP32 then treats any line noise already in its buffer as a
            # separate (rejected) line instead of gluing it onto this command. Empty lines are
            # ignored by the firmware.
            self.ser.write(('\n' + c + self.eol).encode())
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'Serial write failed: {e}')
            self._close()
            return
        if c != self.sent:
            self.get_logger().info(f'-> {c} ({NAMES[c]})')
        self.sent = c
        self.last_write = time.monotonic()
        self.pub_sent.publish(String(data=c))

    def _close(self):
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        self.ser = None

    # ------------------------------------------------------------------ ToF safety
    def _on_obstacle(self, m):
        self.obstacle, self.last_tof = m.data, time.monotonic()

    def _on_cliff(self, m):
        self.cliff, self.last_tof = m.data, time.monotonic()

    def _hazard(self):
        """Why forward driving is unsafe right now, or None."""
        if not self.use_safety:
            return None
        if time.monotonic() - self.last_tof > self.tof_timeout_s:
            return 'no ToF data'
        if self.cliff:
            return 'cliff'
        if self.obstacle:
            return 'obstacle'
        return None

    # ------------------------------------------------------------------ commands
    def _command(self, c):
        self.last_cmd_time = time.monotonic()
        hazard = self._hazard() if c == 'w' else None
        if hazard:
            self.get_logger().warn(f'Forward blocked: {hazard}', throttle_duration_sec=1.0)
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
        hazard = self._hazard() if self.sent == 'w' else None
        if hazard:
            self.get_logger().warn(f'{hazard} - stopping')
            self._write('x')
        # heartbeat: keep the ESP32 watchdog fed while moving
        if (self.fw_watchdog_ms > 0 and self.sent in ('w', 's', 'a', 'd')
                and time.monotonic() - self.last_write > self.heartbeat_s):
            self._write(self.sent)
        # read anything the ESP32 prints
        try:
            # Drain everything waiting: the IMU stream (~100 lines/s, ~5.5 kB/s) outruns a fixed
            # 256 bytes per 50 ms tick, and an overflowing tty buffer drops bytes mid-line.
            data = self.ser.read(max(1, self.ser.in_waiting))
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
                    if text != self.last_rx_line:       # heartbeat re-sends repeat the same ack
                        self.get_logger().info(f'ESP32: {text}')
                        self.pub_rx.publish(String(data=text))
                    self.last_rx_line = text
                    if 'calibration complete' in text or 'IMU init failed' in text:
                        self._send_setup()   # ESP32 rebooted: restore speed + watchdog

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
