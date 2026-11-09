"""GUI viewer: live matplotlib heatmap of tof/distance_mm with the mm value in each cell.
Needs a display (monitor, VNC, or ssh -X)."""
import threading

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray


class HeatmapViewer(Node):
    def __init__(self):
        super().__init__('tof_heatmap_viewer')
        self.max_mm = self.declare_parameter('max_mm', 2000).value
        self.show_values = self.declare_parameter('show_values', True).value
        self.lock = threading.Lock()
        self.frame = None
        self.create_subscription(Int16MultiArray, 'tof/distance_mm', self._cb, 10)

    def _cb(self, msg):
        rows = msg.layout.dim[0].size if msg.layout.dim else 8
        a = np.array(msg.data, dtype=np.float32).reshape(rows, -1)
        a[a < 0] = np.nan
        with self.lock:
            self.frame = a


def main(args=None):
    rclpy.init(args=args)
    node = HeatmapViewer()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    fig, ax = plt.subplots(figsize=(6, 5.5))
    fig.canvas.manager.set_window_title('VL53L5CX ToF')
    cmap = plt.get_cmap('turbo_r').copy()
    cmap.set_bad('#444444')
    im, texts, shape = None, [], None

    try:
        while rclpy.ok() and plt.fignum_exists(fig.number):
            with node.lock:
                a = node.frame
            if a is not None:
                if a.shape != shape:  # (re)build the plot for this resolution
                    ax.clear()
                    shape = a.shape
                    im = ax.imshow(a, cmap=cmap, vmin=0, vmax=node.max_mm)
                    ax.set_xticks(range(shape[1]))
                    ax.set_yticks(range(shape[0]))
                    if not texts:
                        fig.colorbar(im, ax=ax, label='distance (mm)')
                    texts = [[ax.text(c, r, '', ha='center', va='center', fontsize=9)
                              for c in range(shape[1])] for r in range(shape[0])]
                im.set_data(a)
                if node.show_values:
                    for r in range(shape[0]):
                        for c in range(shape[1]):
                            v = a[r, c]
                            texts[r][c].set_text('--' if np.isnan(v) else f'{int(v)}')
                            texts[r][c].set_color('white' if np.isnan(v) or v > node.max_mm * 0.6 else 'black')
                valid = a[~np.isnan(a)]
                ax.set_title(f'{shape[0]}x{shape[1]} ToF   min {valid.min():.0f} mm' if valid.size
                             else 'no valid zones')
            plt.pause(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
