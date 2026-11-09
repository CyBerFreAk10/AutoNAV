"""Terminal viewer: prints tof/distance_mm as a live, colour-coded 2D matrix."""
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int16MultiArray, Int8MultiArray


def _colour(mm, max_mm):
    """Near = red, far = blue (256-colour ANSI background)."""
    if mm < 0:
        return '\033[48;5;236m\033[38;5;244m'
    ramp = [196, 202, 208, 214, 220, 190, 154, 118, 49, 45, 39, 33, 27, 21]
    i = min(int(mm / max_mm * (len(ramp) - 1)), len(ramp) - 1)
    return f'\033[48;5;{ramp[i]}m\033[38;5;16m'


class MatrixViewer(Node):
    def __init__(self):
        super().__init__('tof_matrix_viewer')
        self.max_mm = self.declare_parameter('max_mm', 2000).value
        self.colour = self.declare_parameter('colour', True).value and sys.stdout.isatty()
        self.cls, self.obst, self.cliff = None, None, None
        self.create_subscription(Int16MultiArray, 'tof/distance_mm', self._cb, 10)
        # Optional: hazard node outputs (shown if tof_hazard_node is running)
        self.create_subscription(Int8MultiArray, 'tof/zone_class',
                                 lambda m: setattr(self, 'cls', list(m.data)), 10)
        self.create_subscription(Bool, 'tof/obstacle_detected',
                                 lambda m: setattr(self, 'obst', m.data), 10)
        self.create_subscription(Bool, 'tof/cliff_detected',
                                 lambda m: setattr(self, 'cliff', m.data), 10)
        sys.stdout.write('\033[2J')

    def _cb(self, msg):
        rows = msg.layout.dim[0].size if msg.layout.dim else 8
        cols = msg.layout.dim[1].size if len(msg.layout.dim) > 1 else len(msg.data) // rows
        reset = '\033[0m'
        out = ['\033[H', f'VL53L5CX {rows}x{cols}  distance (mm)   ---- invalid   '
               f'O obstacle   C cliff\n\n']
        cls = self.cls if self.cls and len(self.cls) == len(msg.data) else None
        out.append('      ' + ''.join(f'{c:^7}' for c in range(cols)) + '\n')
        for r in range(rows):
            line = f'  {r:>2}  '
            for c in range(cols):
                v = msg.data[r * cols + c]
                mark = {1: 'O', 2: 'C'}.get(cls[r * cols + c], ' ') if cls else ' '
                txt = ('  ----' if v < 0 else f' {v:5d}') + mark
                line += (_colour(v, self.max_mm) + txt + reset) if self.colour else txt
            out.append(line + '\n')
        valid = [v for v in msg.data if v >= 0]
        if valid:
            out.append(f'\n  min {min(valid)} mm   max {max(valid)} mm   '
                       f'valid {len(valid)}/{len(msg.data)}      \n')
        else:
            out.append(f'\n  no valid zones{" " * 30}\n')
        if self.obst is not None:
            out.append(f'  OBSTACLE: {"YES" if self.obst else "no "}   '
                       f'CLIFF: {"YES" if self.cliff else "no "}      \n')
        sys.stdout.write(''.join(out))
        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    node = MatrixViewer()
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
