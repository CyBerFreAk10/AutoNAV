#!/usr/bin/env python3
"""Keyboard teleop for the rover (run on the LAPTOP).

Self-contained: only needs ROS 2 (rclpy + std_msgs), so you can copy just this file:
    source /opt/ros/jazzy/setup.bash
    python3 rover_teleop.py

Keys: w forward  s back  a left  d right  x / space stop  q quit
The current command is re-published at 10 Hz so the Pi's watchdog knows the laptop
is still connected; if the laptop drops off, the rover stops by itself.
"""
import select
import sys
import termios
import tty

import rclpy
from std_msgs.msg import String

KEYS = {'w': 'w', 's': 's', 'a': 'a', 'd': 'd', 'x': 'x', ' ': 'x'}
NAMES = {'w': 'FORWARD', 's': 'BACK   ', 'a': 'LEFT   ', 'd': 'RIGHT  ', 'x': 'STOP   '}


def main():
    if not sys.stdin.isatty():
        sys.exit('rover_teleop needs an interactive terminal')
    rclpy.init()
    node = rclpy.create_node('rover_teleop')
    pub = node.create_publisher(String, 'rover/cmd', 10)
    sent = {'c': '?'}
    node.create_subscription(String, 'rover/sent', lambda m: sent.__setitem__('c', m.data), 10)

    old = termios.tcgetattr(sys.stdin)
    cmd = 'x'
    print('Keys: w forward  s back  a left  d right  x/space stop  q quit')
    print('Waiting for the Pi bridge... (rover/sent shows what reached the ESP32)\n')
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.1)[0]:
                k = sys.stdin.read(1).lower()
                if k == 'q':
                    break
                cmd = KEYS.get(k, cmd)
            pub.publish(String(data=cmd))
            rclpy.spin_once(node, timeout_sec=0)
            sys.stdout.write(f'\r  command: {NAMES[cmd]}   ESP32 got: {sent["c"]}   ')
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        for _ in range(3):
            pub.publish(String(data='x'))
        print('\nStopped.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
