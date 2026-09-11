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

*   Develop the ROS 2 node (micro-ROS or standard serial bridge) to translate Nav2 `cmd_vel` messages into raw PWM commands for the ESP32.
*   Publish the ESP32's IMU data stream as a ROS 2 `sensor_msgs/Imu` topic.
*   Transition from Gazebo simulation to physical hardware testing for autonomous mapping.
