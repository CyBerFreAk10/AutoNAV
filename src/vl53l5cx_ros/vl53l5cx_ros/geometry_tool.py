"""Plan the mounting angle before drilling holes.

  ros2 run vl53l5cx_ros tof_geometry <height_m> <tilt_down_deg> [resolution]
  ros2 run vl53l5cx_ros tof_geometry 0.10 20
"""
import sys

from .geometry import TofGeometry


def main():
    a = sys.argv[1:]
    if len(a) < 2:
        print(__doc__)
        sys.exit(1)
    n = int(a[2]) if len(a) > 2 else 8
    print(TofGeometry(n, float(a[0]), float(a[1])).describe())


if __name__ == '__main__':
    main()
