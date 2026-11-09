# AutoNav - Autonomous SLAM Warehouse Rover

AutoNav is a fully autonomous ground vehicle designed for comprehensive warehouse mapping and shortest-path navigation. Powered by an NVIDIA Jetson/Raspberry Pi and built upon a robust ROS 2 software stack, this rover utilizes a 360-degree 2D LiDAR, an IMU, and 8x8 ToF sensors for full spatial awareness. The system performs frontier exploration and mapping autonomously, bypassing manual teleoperation to deliver accurate, hands-free environmental data.

## Project Status

**Current Phase:** Hardware Integration & Simulation Validation

*   **Simulation:** Running warehouse environments in Gazebo to validate path planning and obstacle avoidance logic.
*   **Methodology:** Completed block diagrams for hardware integration, SLAM processing, and path planning.
*   **Documentation:** Market research section drafted to highlight the product gap for fully autonomous, non-manual mapping rovers.
*   **Hardware Interface:** Completed the ESP32 firmware that manages the low-level hardware layer.

## Hardware Architecture

### Low-Level Control (ESP32)
*   **Motor Control:** L298N motor driver interfacing with DC motors.
*   **Sensors:** ICM-20948 IMU for precise acceleration, gyro, and integrated yaw data.
*   **Safety:** Features a configurable communication watchdog to halt movement if connection to the main compute unit is lost.
*   **Integration:** Streams calibrated IMU data and accepts signed PWM commands over serial at 115200 baud.

### High-Level Compute (Jetson/Raspberry Pi)
*   **Perception:** Integrates the 360-degree 2D LiDAR and 8x8 ToF sensors.
*   **Mapping:** SLAM Toolbox handles frontier exploration and builds the initial map.
*   **Navigation:** Nav2 manages point-to-point movement utilizing A* for global planning and DWA for local, dynamic obstacle avoidance.

## Setup & Usage (ESP32 Firmware)

1.  Flash the provided ESP32 `.ino` code using the Arduino IDE.
2.  Open the Serial Monitor at 115200 baud.
3.  The firmware will automatically initialize and calibrate the IMU. Do not move the rover during this time.
4.  Once calibrated, the ESP32 will output a continuous CSV stream of IMU data (`t_ms, ax_mg, ay_mg, az_mg, gx_dps, gy_dps, gz_dps, temp_C, yaw_deg`).
5.  Use the serial commands (e.g., `w`, `s`, `a`, `d`, `stop`) to manually verify motor connections before integrating with the ROS 2 stack.

### Wiring Configuration

**Motor Control (L298N)**
* **ENA:** GPIO 25 (Left PWM)
* **IN1 / IN2:** GPIO 26 / GPIO 27 (Left Direction)
* **ENB:** GPIO 14 (Right PWM)
* **IN3 / IN4:** GPIO 33 / GPIO 32 (Right Direction)
* *Note: ENA/ENB hardware jumpers must be removed for PWM control.*

**IMU Integration (ICM-20948)**
* **SDA:** GPIO 21
* **SCL:** GPIO 22
* **Power:** 3.3V Logic
  
## Next Steps

*   ~~Serial bridge from ROS 2 to the ESP32~~ done (`rover_control`, w/s/a/d/x). Next: proportional `cmd_vel` -> `m <left> <right>` PWM.
*   ~~Publish the ESP32's IMU data stream as `sensor_msgs/Imu`~~ done (`/imu`). Next: fuse IMU yaw with lidar odometry (robot_localization EKF).
*   Transition from simulation to physical hardware testing for autonomous mapping.

## ROS 2 Software

This repository is a colcon workspace (ROS 2 Jazzy, Raspberry Pi 5, Ubuntu 24.04).

| Package | What it does |
|---|---|
| `vl53l5cx_ros` | VL53L5CX 8x8 ToF driver (I2C), obstacle + cliff detection, terminal / heatmap matrix viewers |
| `rover_control` | ESP32 serial bridge (`/rover/cmd`, `/cmd_vel` -> w/s/a/d/x, IMU CSV -> `/imu`) and laptop keyboard teleop |
| `rover_dashboard` | Web dashboard on port 8080: status, hold-to-drive, stored maps, start/waypoint/end routes |
| `rover_nav` | Lidar odometry, SLAM (slam_toolbox), global localization, boundary-first explorer, 2D simulator |
| `ydlidar_ros2_driver` | Upstream YDLidar driver (git submodule); X4 settings live in `rover_nav/config/ydlidar_x4.yaml` |

### Setup

```bash
git clone --recurse-submodules https://github.com/CyBerFreAk10/AutoNAV.git ~/autonav_ws
# YDLidar SDK (needed by the driver): https://github.com/YDLIDAR/YDLidar-SDK
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-nav2-map-server ros-jazzy-rmw-cyclonedds-cpp \
    python3-scipy python3-serial python3-pil
pip install --user --break-system-packages vl53l5cx-ctypes smbus2
cd ~/autonav_ws && colcon build && source install/setup.bash
```

Network: Pi and laptop use `ROS_DOMAIN_ID=7`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`.

### Running

```bash
ros2 launch rover_control esp32_bridge.launch.py              # ESP32 motors + IMU
ros2 launch vl53l5cx_ros vl53l5cx.launch.py                   # ToF + obstacle/cliff detection
ros2 launch rover_dashboard dashboard.launch.py               # http://<pi-ip>:8080
python3 rover_teleop.py                                       # on the laptop: w/a/s/d/x

ros2 launch rover_nav mapping.launch.py                       # record a map (drive manually)
ros2 launch rover_nav mapping.launch.py explore:=true         # autonomous boundary-first mapping
ros2 launch rover_nav mapping.launch.py continue_map:=<name>  # extend a saved map from any start pose
ros2 launch rover_nav localization.launch.py map:=<name>      # localize in a saved map (unknown start pose)
ros2 launch rover_nav mapping.launch.py sim:=true explore:=true  # simulator (use another ROS_DOMAIN_ID!)
```

Maps are saved in `~/maps` (`.yaml`/`.pgm` image + slam_toolbox `.posegraph`/`.data`).

### Mapping strategy (`rover_nav/explorer`)

1. Drive to the nearest wall and follow it (wall on the right) until the robot is back at the loop start.
2. The loop's winding direction tells the outer boundary (counter-clockwise) from a free-standing object (clockwise); objects are recorded and the search continues.
3. Every obstacle cluster inside the boundary loop is an inner structure: plan a path beside it and follow it until that loop closes.
4. Visit remaining unseen gaps inside the boundary, then save the map.

Localization: `laser_odometry` (point-to-line ICP, no wheel encoders needed) feeds slam_toolbox; `global_localizer` finds the pose in a saved map with no initial guess (exhaustive scan matching, rejects ambiguous matches) and re-localizes automatically when the scan stops fitting the map.

Simulator results (8 x 6 m room, 3 inner structures): full exploration completed; mean pose error 0.8 cm; global localization correct from 20/20 random poses (median 2.8 cm / 0.1°).
