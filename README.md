# AutoNav - Autonomous SLAM Warehouse Rover

AutoNav is a fully autonomous ground vehicle designed for comprehensive warehouse mapping and shortest-path navigation. Powered by a Raspberry Pi 5 (Jetson-compatible) and built upon a ROS 2 software stack, this rover utilizes a 360-degree 2D LiDAR, an IMU, and an 8x8 ToF sensor for full spatial awareness. The system explores and maps an area autonomously, bypassing manual teleoperation to deliver accurate, hands-free environmental data.

## Contents

1. [Project Status](#project-status)
2. [System Architecture](#system-architecture)
3. [Hardware](#hardware)
4. [ESP32 Firmware](#esp32-firmware)
5. [ROS 2 Workspace](#ros-2-workspace)
6. [Installation](#installation)
7. [Quick Start](#quick-start)
8. [Autonomous Mapping](#autonomous-mapping)
9. [Package Reference](#package-reference)
   - [rover_bringup - one-command bring-up](#rover_bringup---one-command-bring-up)
   - [vl53l5cx_ros - ToF sensor, obstacle & cliff detection](#vl53l5cx_ros---tof-sensor-obstacle--cliff-detection)
   - [rover_control - ESP32 bridge & teleop](#rover_control---esp32-bridge--teleop)
   - [rover_dashboard - web dashboard](#rover_dashboard---web-dashboard)
   - [rover_nav - mapping, localization, exploration](#rover_nav---mapping-localization-exploration)
10. [Topics, Services & Frames](#topics-services--frames)
11. [Hardware Bring-up Checklist](#hardware-bring-up-checklist)
12. [Simulation & Test Results](#simulation--test-results)
13. [Troubleshooting](#troubleshooting)
14. [Development Workflow](#development-workflow)
15. [Roadmap](#roadmap)

---

## Project Status

**Current Phase:** Hardware Integration & Simulation Validation

*   **Hardware Interface:** ESP32 firmware complete (L298N motor control, ICM-20948 IMU stream, watchdog).
*   **ROS 2 stack:** Sensor drivers, ESP32 serial bridge, web dashboard, lidar odometry, SLAM, global localization and an autonomous boundary-first explorer are implemented.
*   **Autonomous mapping:** one-command bring-up (`rover_bringup`), started and monitored from the dashboard (live map, start/stop), optional start at boot.
*   **Simulation:** Full mapping, localization and exploration pipeline validated in the built-in 2D simulator (results [below](#simulation--test-results)).
*   **Next:** First autonomous runs on the physical rover (see the [bring-up checklist](#hardware-bring-up-checklist)).
*   **Methodology:** Block diagrams completed for hardware integration, SLAM processing and path planning.
*   **Documentation:** Market research section drafted to highlight the product gap for fully autonomous, non-manual mapping rovers.

---

## System Architecture

```
 Laptop (Ubuntu, ROS 2)                     Raspberry Pi 5 (ROS 2 Jazzy)                         ESP32
 ──────────────────────                     ────────────────────────────                         ─────
 rover_teleop (keyboard) ──/rover/cmd──┐
 Browser ──HTTP :8080──► rover_dashboard ─┤
                                          ├─► esp32_bridge ──USB serial "w\n"…──► motors (L298N)
 explorer (rover_nav) ────/cmd_vel───────┘        ▲   └──/imu◄── IMU CSV stream ◄── ICM-20948
                                                  │
 YDLidar X4 ─/scan─► laser_odometry ─odom TF─► slam_toolbox ─/map, map TF─► explorer / dashboard
                                                     ▲
                       global_localizer ─/initialpose┘ (or deserialize_map to continue a map)
 VL53L5CX (I2C) ─► vl53l5cx_node ─► tof_hazard_node ─/tof/obstacle_detected, /tof/cliff_detected─► explorer, bridge
```

| Layer | Component |
|---|---|
| Low-level control | ESP32: L298N motor driver, ICM-20948 IMU, serial command interface, watchdog |
| Perception | YDLidar X4 (360°, ~8 m usable), VL53L5CX 8x8 ToF (obstacles + cliffs), IMU |
| Odometry | Lidar scan matching (point-to-line ICP); no wheel encoders required |
| Mapping | slam_toolbox (online async) with loop closure, fed by lidar odometry |
| Localization | slam_toolbox localization mode + global localizer (no initial pose needed) |
| Exploration | Boundary-first explorer: outer wall loop → inner structures → remaining gaps |
| Operator interface | Web dashboard (status, driving, maps, routes) and laptop keyboard teleop |

---

## Hardware

| Part | Notes |
|---|---|
| Raspberry Pi 5 | Ubuntu 24.04, ROS 2 Jazzy |
| ESP32 dev board (CP2102 USB-serial) | Motor + IMU controller, USB to the Pi |
| L298N + 2 DC motors | Differential drive (left / right side) |
| ICM-20948 IMU | I2C on the ESP32 (address 0x69) |
| YDLidar X4 (CP2102 USB-serial) | 128000 baud, 10 Hz, 360° |
| VL53L5CX 8x8 ToF | I2C bus 1 on the Pi (address 0x29), tilted down for cliff + obstacle detection |

### Wiring

**Motor Control (L298N)** - as defined in `ESP32_Code.ino`:

| Signal | ESP32 GPIO | Function |
|---|---|---|
| IN1 / IN2 | 32 / 33 | Left motor (PWM + direction) |
| IN3 / IN4 | 25 / 26 | Right motor (PWM + direction) |

> ENA/ENB jumpers stay **on** (enables tied high). Speed and direction come from PWM on the IN pins: forward = PWM on IN1/IN3 with IN2/IN4 low, reverse the other way round. Both low = brake.

**IMU (ICM-20948)**: SDA GPIO 21, SCL GPIO 22, 3.3 V logic.

**VL53L5CX → Raspberry Pi**: SDA GPIO 2 (pin 3), SCL GPIO 3 (pin 5), 3.3 V, GND. Check with `i2cdetect -y 1` (device at `0x29`). Remove the sensor's protective film - with it on, every zone reads ~0-10 mm.

---

## ESP32 Firmware

### Flashing

1.  Flash `ESP32_Code.ino` using the Arduino IDE (ESP32 board package).
2.  Open the Serial Monitor at **115200 baud**, line ending **Newline**.
3.  The firmware initializes and calibrates the IMU gyro (2000 samples). **Do not move the rover during this time.**
4.  Once calibrated, it streams IMU data as CSV: `t_ms,ax_mg,ay_mg,az_mg,gx_dps,gy_dps,gz_dps,temp_C,yaw_deg`.
5.  Use the commands below to verify the motors before using ROS.

### Serial commands

Commands are **line-based** (terminate with Enter / `\n`). Status replies start with `#`.

| Command | Action |
|---|---|
| `w` / `f` `[ms]` | Forward (optionally for `ms` milliseconds) |
| `s` / `b` `[ms]` | Backward |
| `a` / `l` `[ms]` | Spin left |
| `d` / `r` `[ms]` | Spin right |
| `x` / `stop` | Stop (brake) |
| `m <L> <R>` | Raw signed PWM per side, -255..255 |
| `v [0-255]` | Set / show the speed used by w/a/s/d/test (default 200) |
| `wd <ms>` | Stop if no motor command arrives within `ms` (0 = off, default off) |
| `test` | Start / stop the scripted motor test (fwd, back, left, right) |
| `q` | Toggle the IMU CSV stream |
| `z` | Zero the integrated yaw |
| `cal` | Stop motors and recalibrate the gyro |
| `h` / `?` | Help |

> The ROS bridge sends a command only when it changes, so leave the firmware watchdog (`wd`) off, or a long straight drive will be stopped. The bridge has its own watchdog (see [rover_control](#rover_control---esp32-bridge--teleop)).

---

## ROS 2 Workspace

This repository **is** a colcon workspace (`~/autonav_ws` on the Pi).

```
AutoNAV/
├── ESP32_Code.ino                 ESP32 firmware
├── field_test.sh                  preflight checks + launch everything (field testing)
├── README.md
└── src/
    ├── vl53l5cx_ros/              ToF driver, obstacle/cliff detection, matrix viewers
    ├── rover_control/             ESP32 serial bridge + laptop keyboard teleop
    ├── rover_dashboard/           Web dashboard (port 8080)
    ├── rover_nav/                 Lidar odometry, SLAM configs, global localizer, explorer, simulator
    ├── rover_bringup/             One-command autonomous mapping launch, serial-port setup, boot service
    └── ydlidar_ros2_driver/       Upstream YDLidar driver (git submodule)
```

| Package | Nodes / executables |
|---|---|
| `vl53l5cx_ros` | `vl53l5cx_node`, `tof_hazard_node`, `tof_matrix_viewer`, `tof_heatmap_viewer`, `tof_geometry` |
| `rover_control` | `esp32_bridge`, `rover_teleop` |
| `rover_dashboard` | `dashboard` |
| `rover_nav` | `laser_odometry`, `global_localizer`, `explorer`, `rover_sim` |
| `rover_bringup` | `autonomous_mapping.launch.py`, `setup_serial_ports`, `scripts/install_service.sh` |
| `ydlidar_ros2_driver` | `ydlidar_ros2_driver_node` (X4 settings in `rover_nav/config/ydlidar_x4.yaml`) |

---

## Installation

### Raspberry Pi

```bash
# 1. Clone (with the lidar driver submodule)
git clone --recurse-submodules https://github.com/CyBerFreAk10/AutoNAV.git ~/autonav_ws

# 2. YDLidar SDK (required by ydlidar_ros2_driver)
#    https://github.com/YDLIDAR/YDLidar-SDK  -> cmake, make, sudo make install

# 3. ROS + Python dependencies
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-nav2-map-server ros-jazzy-nav2-msgs \
    ros-jazzy-rmw-cyclonedds-cpp python3-scipy python3-serial python3-pil i2c-tools
pip install --user --break-system-packages vl53l5cx-ctypes smbus2

# 4. Permissions (log out/in afterwards)
sudo usermod -aG dialout,i2c $USER

# 5. Build
cd ~/autonav_ws && source /opt/ros/jazzy/setup.bash && colcon build
echo "source ~/autonav_ws/install/setup.bash" >> ~/.bashrc

# 6. Fixed serial port names (plug in BOTH the ESP32 and the lidar first)
ros2 run rover_bringup setup_serial_ports --install   # -> /dev/rover_esp32, /dev/rover_lidar
```

`vl53l5cx-ctypes` and `smbus2` must be installed for the **system** Python (`/usr/bin/python3`), which ROS uses - not a virtualenv.

### Network (Pi and laptop)

Both machines must be on the same LAN with the same settings in `~/.bashrc`:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=7
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

### Laptop (teleop only)

```bash
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
scp edward@<pi-ip>:~/autonav_ws/src/rover_control/rover_control/rover_teleop.py .
python3 rover_teleop.py
```

If the laptop cannot see the Pi's topics, check the firewall (`sudo ufw disable` to test).

---

## Quick Start

**Autonomous mapping (everything in one command):**

```bash
~/autonav_ws/field_test.sh            # checks everything, then starts the stack
# open http://<pi-ip>:8080 -> "Autonomous mapping" -> Start
```

(`ros2 launch rover_bringup autonomous_mapping.launch.py` starts the stack without the checks.)

See [Autonomous Mapping](#autonomous-mapping) for details. To run parts individually, each line in its own terminal on the Pi (after `source ~/autonav_ws/install/setup.bash`):

```bash
ros2 launch rover_control esp32_bridge.launch.py          # motors + IMU
ros2 launch vl53l5cx_ros vl53l5cx.launch.py               # ToF + obstacle/cliff detection
ros2 launch rover_dashboard dashboard.launch.py           # open http://<pi-ip>:8080
```

Then one of:

```bash
ros2 launch rover_nav mapping.launch.py                       # record a map, drive manually
ros2 launch rover_nav mapping.launch.py explore:=true         # autonomous boundary-first mapping
ros2 launch rover_nav mapping.launch.py continue_map:=<name>  # extend a saved map from any start pose
ros2 launch rover_nav localization.launch.py map:=<name>      # localize in a saved map (unknown start)
```

Try everything without the rover (use a different domain so the real rover never receives commands):

```bash
ROS_DOMAIN_ID=99 ros2 launch rover_nav mapping.launch.py sim:=true explore:=true
```

---

## Autonomous Mapping

### 0. Field test script (recommended)

`field_test.sh` in the repository root builds the workspace if needed, runs the preflight checks and then starts the whole stack:

```bash
cd ~/autonav_ws
./field_test.sh --check-only          # only check (rover does not move)
./field_test.sh --motion --apply      # + measure speed / turn rate and save them (rover moves briefly)
./field_test.sh                       # check, then launch; Start from the dashboard
./field_test.sh --record map_name:=warehouse_a   # + rosbag of the run in ~/autonav_logs
```

| Flag | Meaning |
|---|---|
| `--check-only` | run the checks and exit |
| `--motion` | timed moves (forward/back/left/right, 0.5 s each, stopped by the firmware): measures speed and turn rate, checks that `w` goes forward in the lidar frame, that `a` turns left, and that lidar and IMU agree on the rotation direction |
| `--apply` | write the measured `lin_speed` / `ang_speed` into `rover_nav/config/explorer.yaml` and rebuild |
| `--record` | record `/scan /odom /tf /imu /cmd_vel /map /explorer/state ...` to `~/autonav_logs/bag_<date>` |
| `--force` / `--skip-checks` | launch despite FAILs / without checks |
| anything else | passed to `autonomous_mapping.launch.py` (`explore:=true`, `tof:=false`, `continue_map:=...`) |

The preflight (`ros2 run rover_bringup preflight`) checks: ROS environment and domain, packages built and up to date, Python modules, user groups, **power supply undervoltage / throttling**, CPU temperature, disk space, network / dashboard URL, that the stack is not already running, explorer / bridge / ToF configuration, serial devices (ESP32 vs. lidar identification, fixed names), ESP32 IMU stream (rate, gravity, gyro drift), ToF (I2C, ranging, protective film), lidar (scan rate, valid points, distances front/left/back/right to compare with reality). Each FAIL / WARN prints the fix. Results are also saved to `~/autonav_logs/preflight_<date>.json`; launch output to `~/autonav_logs/run_<date>.log`.

### 1. One-time setup

1. Complete the [bring-up checklist](#hardware-bring-up-checklist) (lidar orientation, rover speeds, ToF film).
2. Give the serial devices fixed names (both plugged in, keep them in the same USB sockets afterwards):
   ```bash
   ros2 run rover_bringup setup_serial_ports            # shows which device is which
   ros2 run rover_bringup setup_serial_ports --install  # writes /etc/udev/rules.d/99-autonav-serial.rules
   ```
   It listens to each device: the one printing `#` lines / IMU CSV at 115200 baud is the ESP32, the other the lidar. Until this is done the launch falls back to `/dev/ttyUSB0` (ESP32) and `/dev/ydlidar` (lidar), which can collide.
3. Optional - start the stack at boot:
   ```bash
   bash ~/autonav_ws/src/rover_bringup/scripts/install_service.sh          # install + start
   bash ~/autonav_ws/src/rover_bringup/scripts/install_service.sh --remove # uninstall
   journalctl -u autonav -f                                                 # logs
   ```
   The rover does **not** drive at boot - it waits for *Start* in the dashboard.

### 2. Run

```bash
ros2 launch rover_bringup autonomous_mapping.launch.py               # wait for Start in the dashboard
ros2 launch rover_bringup autonomous_mapping.launch.py explore:=true # start exploring immediately
ros2 launch rover_bringup autonomous_mapping.launch.py continue_map:=warehouse   # extend a saved map
```

| Argument | Default | Meaning |
|---|---|---|
| `explore` | `false` | start exploring immediately instead of waiting for *Start* |
| `map_name` | `""` | name of the saved map (`explored_<date>` if empty; also settable in the dashboard) |
| `continue_map` | `""` | find the robot in this saved map and keep extending it |
| `maps_dir` | `~/maps` | where maps are saved |
| `tof` | `true` | run the ToF sensor and let the explorer react to cliffs / low obstacles |
| `dashboard` | `true` | run the web dashboard |
| `esp32_port`, `lidar_port` | `""` | override the serial ports |

Started nodes: ESP32 bridge, YDLidar driver, `laser_odometry`, slam_toolbox, `explorer`, VL53L5CX + hazard detection, dashboard.

### 3. Monitor and control from the dashboard

* **Autonomous mapping panel**: explorer state and progress, optional map name, **Start** / **Stop**, **Localize now**, localization fit.
* **● Live SLAM map** (first entry of the map list): the map being built, refreshed every 2 s, with the robot, the closed boundary loop (green) and the explorer's planned path (orange, dashed).
* **Manual override**: any drive command (buttons, W A S D, Stop) stops the explorer first.
* When exploration finishes, the map (image + pose graph) is saved to `~/maps/<name>` and appears in the map list, ready for routes and `localization.launch.py`.

The same from the command line: `ros2 service call /explorer/start std_srvs/srv/Trigger`, `/explorer/stop`, `ros2 param set /explorer map_name warehouse`, `ros2 topic echo /explorer/state`.

---

## Package Reference

### rover_bringup - one-command bring-up

| File | Purpose |
|---|---|
| `launch/autonomous_mapping.launch.py` | ESP32 bridge + lidar + odometry + SLAM + explorer + ToF + dashboard ([arguments](#2-run)) |
| `rover_bringup/preflight.py` (`ros2 run rover_bringup preflight [--motion] [--apply]`) | field-readiness checks (used by `field_test.sh`) |
| `rover_bringup/setup_serial_ports.py` (`ros2 run rover_bringup setup_serial_ports [--install]`) | detects ESP32 vs. lidar and writes udev rules for `/dev/rover_esp32`, `/dev/rover_lidar` |
| `scripts/install_service.sh [--remove]` | systemd service `autonav` that runs the launch file at boot (domain 7, CycloneDDS) |

`rover_nav` launch files also use `/dev/rover_lidar` automatically when it exists.

### vl53l5cx_ros - ToF sensor, obstacle & cliff detection

Driver for the ST VL53L5CX multizone ToF sensor (via `vl53l5cx-ctypes`, ST's ULD library) plus a hazard detector for a downward-tilted mount.

```bash
ros2 launch vl53l5cx_ros vl53l5cx.launch.py                  # driver + hazard detection
ros2 launch vl53l5cx_ros vl53l5cx.launch.py hazards:=false   # driver only
ros2 launch vl53l5cx_ros vl53l5cx.launch.py viewer:=heatmap  # + GUI heatmap (needs a display)
ros2 run vl53l5cx_ros tof_matrix_viewer                      # live colour 8x8 matrix in the terminal
ros2 run vl53l5cx_ros tof_geometry 0.10 20                   # plan the mount: height (m), tilt (deg)
```

**Driver (`vl53l5cx_node`)** - firmware upload on start takes ~2 s.

| Output | Type | Content |
|---|---|---|
| `tof/distance_mm` | `std_msgs/Int16MultiArray` | rows x cols in mm, `-1` = invalid zone |
| `tof/status` | `std_msgs/UInt8MultiArray` | raw ST target status per zone |
| `tof/depth` | `sensor_msgs/Image` (32FC1) | metres, NaN = invalid (works in RViz / rqt_image_view) |

Parameters (`config/vl53l5cx.yaml`): `resolution` (8 → 8x8 up to 15 Hz, 4 → 4x4 up to 60 Hz), `frequency_hz`, `i2c_bus`, `i2c_address`, `valid_statuses` (default 5, 6, 9), `flip_horizontal` / `flip_vertical` (the lens mirrors the image - set these if left/right or up/down look swapped).

**Hazard detection (`tof_hazard_node`)** - converts every zone into a 3D point in `base_link` using the mount height and tilt:

* **Obstacle**: point higher than `obstacle_min_height` (3 cm) and closer than `obstacle_max_distance` (0.6 m).
* **Cliff**: point lower than `cliff_min_drop` (4 cm) below the floor, or no return where floor is expected within `cliff_max_floor_range` (if `invalid_is_cliff`).
* Requires `min_zones` agreeing zones for `debounce_frames` consecutive frames.

`tilt_down_deg` is the optical axis angle **below horizontal**: a sensor tilted 70° from vertical is `20`. 15-30° covers both obstacles and cliffs; ~70° only sees floor 7-13 cm ahead (cliff-only).

**Floor calibration** (recommended after mounting): put the rover on flat floor with ~1 m clear ahead and run

```bash
ros2 service call /tof/calibrate_floor std_srvs/srv/Trigger
```

It measures the real height, tilt and range model and saves them to `~/.ros/vl53l5cx_floor.yaml` (loaded automatically).

| Output | Type |
|---|---|
| `tof/obstacle_detected`, `tof/cliff_detected` | `std_msgs/Bool` |
| `tof/nearest_obstacle` | `std_msgs/Float32` (m, inf = none) |
| `tof/zone_class` | `std_msgs/Int8MultiArray` (0 clear, 1 obstacle, 2 cliff, -1 unknown) |
| `tof/points`, `tof/hazard_points` | `sensor_msgs/PointCloud2` in `base_link` (hazards usable in a Nav2 costmap) |

### rover_control - ESP32 bridge & teleop

```bash
ros2 launch rover_control esp32_bridge.launch.py   # on the Pi
python3 rover_teleop.py                            # on the laptop
```

**`esp32_bridge`** (Pi) writes line commands (`w\n`, `x\n`, ...) to the ESP32:

| Input | Mapping |
|---|---|
| `rover/cmd` (`std_msgs/String`) | `w` / `s` / `a` / `d` / `x` |
| `cmd_vel` (`geometry_msgs/Twist`) | turn if \|angular\| > \|linear\| (`a` / `d`), else `w` / `s`; below `cmd_vel_deadband` = `x` |

* Sends only on change; sends `x` if no command for `watchdog_s` (0.5 s) and on shutdown.
* `use_tof_safety: true` blocks forward while `tof/obstacle_detected` or `tof/cliff_detected` is true.
* Parses the IMU CSV stream into `imu` (`sensor_msgs/Imu`, frame `imu_link`; yaw-only orientation); `#` status lines go to the log and `rover/esp32_rx`.
* Publishes every sent command on `rover/sent`.
* Opens the port without toggling DTR/RTS (`reset_on_open: false`) so the ESP32 does not reboot.

Config: `config/esp32_bridge.yaml` (`port`, `baud` 115200, `line_ending` `"\n"`, `watchdog_s`, `use_tof_safety`).

**`rover_teleop`** (laptop): `w` forward, `s` back, `a` left, `d` right, `x`/space stop, `q` quit. The command latches and is re-published at 10 Hz, so the bridge watchdog stops the rover if the laptop disconnects. The status line shows what actually reached the ESP32.

### rover_dashboard - web dashboard

```bash
ros2 launch rover_dashboard dashboard.launch.py   # http://<pi-ip>:8080
```

* **Status bar**: browser link, ESP32 bridge, ToF live, obstacle, cliff, Nav2, robot pose.
* **Drive**: hold the on-screen buttons or W A S D / arrow keys; release, Space, switching tab or window = stop.
* **ToF**: live 8x8 grid with obstacle (O) / cliff (C) marks and nearest obstacle distance.
* **ESP32 output** and **running nodes**.
* **Maps**: lists `~/maps` (`.yaml` + `.pgm`), zoom / pan; **Save live map** saves the current SLAM map (image + pose graph).
* **Autonomous mapping**: explorer state, Start / Stop (with optional map name), Localize now, localization fit; **● Live SLAM map** with explorer boundary and path overlays. Manual driving stops the explorer.
* **Routes**: *Set start*, *Add waypoint*, *Set end* by clicking on the map (drag to set heading); save named routes per map (`~/maps/<map>.routes.json`); **Go** publishes the start as `/initialpose`, loads the map into `map_server` if running, publishes the route on `/rover/route` (`nav_msgs/Path`) and sends it to Nav2 `navigate_through_poses` if Nav2 is running. **Cancel / stop** cancels navigation and stops the rover.

Config: `config/dashboard.yaml` (`host`, `port`, `maps_dir`, frames). Uses only the Python standard library for HTTP. **There is no login** - anyone on the network can drive the rover; set `host: 127.0.0.1` and use an SSH tunnel on untrusted networks.

JSON API (used by the page): `GET /api/state`, `POST /api/cmd {"c":"w"}`, `GET /api/maps`, `GET /api/maps/<m>/image`, `GET|POST /api/maps/<m>/routes`, `DELETE /api/maps/<m>/routes/<r>`, `POST /api/maps/<m>/go`, `POST /api/nav/cancel`, `POST /api/maps/save {"name":...}`, `GET /api/live/image`, `POST /api/explorer/start {"name":...}`, `POST /api/explorer/stop`, `POST /api/localize`.

### rover_nav - mapping, localization, exploration

#### Launch files

| Command | What runs |
|---|---|
| `mapping.launch.py` | lidar + `laser_odometry` + slam_toolbox (mapping) + `explorer` (idle) |
| `mapping.launch.py explore:=true` | as above, explorer starts immediately |
| `mapping.launch.py continue_map:=<name>` | as above + `global_localizer` finds the pose in `~/maps/<name>` and loads its pose graph there, so the new session extends the old map |
| `localization.launch.py map:=<name>` | lidar + `laser_odometry` + slam_toolbox (localization on `~/maps/<name>.posegraph`) + `global_localizer` (auto initial pose, auto recovery) |
| `... sim:=true [world:=<map.yaml>] [start:=x,y,yaw]` | replaces the lidar with `rover_sim` |

Other arguments: `maps_dir` (default `~/maps`), `map_name` (name used when the explorer saves).

#### `laser_odometry` - odometry without wheel encoders

Point-to-line ICP of each scan against a keyframe (new keyframe every 0.15 m / 0.15 rad), constant-velocity prediction, and degeneracy handling (no sliding along featureless corridors). Publishes the `odom → base_link` TF and `/odom`. This is what slam_toolbox uses as its motion prior.

#### slam_toolbox configuration

`config/slam_mapping.yaml` / `slam_localization.yaml` - shipped defaults tuned for a slow lidar-only rover: `base_frame: base_link`, `max_laser_range: 8.0`, `minimum_travel_distance/heading: 0.15` (denser pose graph), `map_update_interval: 2.0`, loop closure on.

#### `global_localizer` - localization from any position and orientation

1. Builds a likelihood field from the map (distance to nearest wall).
2. Scores the current scan at every free position (20 cm grid) and every heading (4°).
3. Refines the 15 best candidates down to 1 cell / 0.25°.
4. Rejects the result if the scan does not match (`min_score`) or if a different pose explains the scan almost as well (`min_misfit_ratio` - e.g. in a symmetric room); then move or rotate the robot a little and retry.
5. Publishes `/initialpose` (localization) or calls `/slam_toolbox/deserialize_map` at the found pose (continue mapping).

While running it publishes `localization/score` (fraction of the scan explained by the map) and re-localizes automatically if the score stays below `lost_score` for `lost_time` seconds.

```bash
ros2 service call /localization/global_localize std_srvs/srv/Trigger   # trigger manually
```

#### `explorer` - boundary-first exploration

1. **Find a wall**: drive to the nearest obstacle.
2. **Follow the outer boundary** with the wall on the right until the robot returns within `loop_close_radius` of the loop start (after at least `loop_min_length`).
3. **Check the loop**: a right-hand wall loop around the outer boundary is counter-clockwise; around a free-standing object it is clockwise. Objects are recorded as mapped and the robot heads into open space to find the real boundary.
4. **Inner structures**: every obstacle cluster inside the boundary loop that has not been circled is a target. The robot plans a path (A*-style grid planner with wall clearance) to a spot `wall_dist` from it, turns so it is on the right, and follows it until that loop closes. Nearest first.
5. **Gaps**: remaining unseen areas inside the boundary are visited.
6. **Done**: map image and pose graph are saved to `maps_dir` via slam_toolbox.

Safety: ToF cliff → back off and turn; ToF obstacle → treated as blocked; stuck detection (no motion for 8 s) → back off; unreachable targets are skipped.

**Discrete drive**: the ESP32 currently drives with fixed-speed w/a/s/d, so every command is pure translation or pure rotation; gentle turns are made by mixing forward and turn ticks in proportion (sigma-delta). Set `discrete_drive: false` once proportional PWM control is available.

```bash
ros2 service call /explorer/start std_srvs/srv/Trigger
ros2 service call /explorer/stop  std_srvs/srv/Trigger
ros2 topic echo /explorer/state          # JSON: state, phase, progress
```

Key parameters (`config/explorer.yaml`) - **measure these on the real rover**:

| Parameter | Default | Meaning |
|---|---|---|
| `robot_radius` | 0.15 | half the rover width + margin (m) |
| `lin_speed` / `ang_speed` | 0.15 / 0.6 | actual speed on `w` (m/s) and turn rate on `a`/`d` (rad/s) |
| `wall_side` / `wall_dist` | right / 0.35 | which side the wall is kept on, and at what distance |
| `front_stop` | 0.40 | turn away when something is this close in front |
| `loop_min_length` / `loop_close_radius` | 2.5 / 0.35 | loop closure detection |
| `island_min_area` | 0.02 | ignore smaller obstacle blobs (m²) |
| `use_tof` | true | use ToF cliff / obstacle flags (needs the ToF film removed) |

#### `rover_sim` - 2D simulator

Drives a virtual robot from `/cmd_vel` in a ground-truth map and publishes a noisy 360-beam `/scan` at 10 Hz plus `/sim/ground_truth`. Default world: 8 x 6 m room with an alcove, a box, a pillar and an L-shaped block; any map_server `.yaml` can be used with `world:=`.

#### Maps

Saved in `~/maps`: `<name>.yaml` + `<name>.pgm` (image, used by the dashboard) and `<name>.posegraph` + `<name>.data` (slam_toolbox graph, needed for localization and `continue_map`). The explorer and the dashboard's *Save live map* write both. Manually:

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '$HOME/maps/room1'}}"
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$HOME/maps/room1'}"
```

---

## Topics, Services & Frames

| Name | Type | From → To |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | lidar / sim → odometry, SLAM, localizer, explorer |
| `/odom` | `nav_msgs/Odometry` | `laser_odometry` |
| `/map` | `nav_msgs/OccupancyGrid` | slam_toolbox → explorer, localizer |
| `/cmd_vel` | `geometry_msgs/Twist` | explorer (or Nav2) → bridge |
| `/rover/cmd` | `std_msgs/String` | teleop, dashboard → bridge |
| `/rover/sent`, `/rover/esp32_rx` | `std_msgs/String` | bridge → dashboard, teleop |
| `/imu` | `sensor_msgs/Imu` | bridge |
| `/tof/*` | see [vl53l5cx_ros](#vl53l5cx_ros---tof-sensor-obstacle--cliff-detection) | ToF nodes |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | localizer, dashboard → slam_toolbox |
| `/localization/score`, `/localization/status` | `Float32`, `String` (JSON) | global_localizer |
| `/explorer/state`, `/explorer/path`, `/explorer/boundary` | `String` (JSON), `nav_msgs/Path` | explorer |
| `/rover/route` | `nav_msgs/Path` | dashboard |
| `/tof/calibrate_floor`, `/localization/global_localize`, `/explorer/start`, `/explorer/stop` | `std_srvs/Trigger` | services |

TF tree: `map → odom` (slam_toolbox) → `base_link` (laser_odometry) → `laser_frame` (static, z = 0.02 m, from the ydlidar launch). ToF points are computed directly in `base_link` from the configured mount geometry.

---

## Hardware Bring-up Checklist

1. **Separate the serial ports.** The lidar and the ESP32 both use a CP2102 USB-serial chip, and the YDLidar udev rule names every CP2102 `/dev/ydlidar`. With both connected run `ros2 run rover_bringup setup_serial_ports --install` (creates `/dev/rover_esp32` and `/dev/rover_lidar`), and always use the same USB sockets.
2. **First drive with the wheels off the ground**: start the bridge and send `w` / `x` from the teleop or dashboard.
3. **Check the lidar orientation**: put an object directly in front of the rover - it must appear near 0° in `/scan`. Otherwise fix the `base_link → laser_frame` transform in the ydlidar launch file. The wall follower depends on this.
4. **Measure the rover**: `robot_radius`, real `lin_speed` and `ang_speed` → `rover_nav/config/explorer.yaml`. Slower turning gives sharper scans and better maps.
5. **ToF**: remove the protective film, set the mount height / tilt in `vl53l5cx_ros/config/vl53l5cx.yaml`, run `/tof/calibrate_floor`, and verify with `tof_matrix_viewer`. Until then set `use_tof: false` in the explorer config.
6. **Map manually first** (`mapping.launch.py`, drive from the dashboard), save, then test `localization.launch.py` from different start positions.
7. **First autonomous run**: `./field_test.sh --motion --apply`, then `./field_test.sh --record`, press *Start* in the dashboard, keep a hand near *Stop* (or any drive key).

---

## Simulation & Test Results

Measured on the Raspberry Pi 5 with the built-in simulator (360 beams, 1 cm range noise, 5% motion noise):

| Test | Result |
|---|---|
| Lidar odometry, ~5 m square drive + 2 full spins | 2.7 cm / 1.4° drift |
| Autonomous exploration, 8 x 6 m room | Outer loop closed (27.1 m, 33 m²), all 3 inner structures circled, map saved; ~10 min at 0.15 m/s |
| SLAM pose accuracy during exploration (logged second half) | mean 0.8 cm, max 2.6 cm; heading mean 0.5° |
| Saved map vs. true walls | median 0 cm, 95th percentile 6 cm |
| Global localization, 20 random poses and headings, no initial guess | 20/20 correct; median 2.8 cm / 0.1°; 0.8 s each |
| `localization.launch.py` from a new start pose, then driving | initial 0.3 cm; tracking mean 0.5 cm, max 1.0 cm |
| `continue_map` from a new start pose | placed in the old map within 1.7 cm |

Hardware tests (bench):

| Test | Result |
|---|---|
| VL53L5CX on I2C bus 1 | detected at 0x29, 8x8 at 15 Hz |
| ToF hazard classification (synthetic floor / box / cliff frames) | all three classified correctly |
| ESP32 bridge vs. emulated firmware (line protocol + IMU CSV) | commands acknowledged, ~50 Hz `/imu` |
| Dashboard API (all endpoints) | verified with curl, including invalid input |
| Dashboard ↔ explorer (sim) | Start with map name, live map PNG, manual override stops the explorer |
| Serial-port detection | ESP32 (streaming), ESP32 (stream off, answers `h`) and lidar-like binary told apart correctly |

The simulator is cleaner than the real X4 lidar (noise, motion distortion while spinning, reflective surfaces) - expect centimetre-level, not millimetre-level, accuracy on hardware.

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| `Package '...' not found` | The terminal was opened before the build: `source ~/autonav_ws/install/setup.bash` |
| `ros2 topic echo` shows nothing but nodes run | `ros2 daemon stop` and retry |
| Laptop does not see the Pi | Same `ROS_DOMAIN_ID` (7) and RMW on both, same subnet, firewall |
| ToF reads 0-10 mm everywhere | Protective film still on the sensor |
| ToF left/right swapped | `flip_horizontal: true` in `vl53l5cx.yaml` |
| Dashboard: "explorer is not running" | Use `autonomous_mapping.launch.py` or `mapping.launch.py` (they start the explorer) |
| Live map list entry stays empty | slam_toolbox not running or no `/scan` (check the lidar port) |
| Lidar driver opens the ESP32 (or vice versa) | Run `ros2 run rover_bringup setup_serial_ports --install` with both plugged in |
| Rover never moves | Bridge `line_ending` must be `"\n"`; check `/rover/esp32_rx` for `# MOTOR ...` replies; check the port isn't the lidar |
| ESP32 reboots when ROS connects | `reset_on_open: false` (default) in `esp32_bridge.yaml` |
| Explorer keeps backing up | ToF reports cliff (film on / not calibrated) - set `use_tof: false` |
| Global localization "ambiguous" | Symmetric area: rotate / move the rover a little and call `/localization/global_localize` again |
| `localization.launch.py`: `.posegraph not found` | Save maps with the explorer, the dashboard or `serialize_map` (image-only maps cannot be used) |
| `pip install` refused (externally managed) | use `--user --break-system-packages` for the system Python |

---

## Development Workflow

```bash
cd ~/autonav_ws
colcon build --packages-select <package>   # rebuild after changes (incl. config / web files)
git add -A
git commit -m "describe the change"
git push
```

The ydlidar driver is a submodule pinned to an upstream commit; local edits inside it are ignored. Keep rover-specific lidar settings in `rover_nav/config/ydlidar_x4.yaml`.

---

## Roadmap

*   ~~ROS 2 serial bridge to the ESP32~~ - done (`rover_control`).
*   ~~Publish the ESP32 IMU stream as `sensor_msgs/Imu`~~ - done (`/imu`).
*   Proportional drive: map `cmd_vel` to `m <left> <right>` PWM for smooth turns (`discrete_drive: false`).
*   Fuse IMU yaw with lidar odometry (robot_localization EKF) for better rotation estimates.
*   Wheel encoders for odometry independent of the lidar.
*   ~~Dashboard: explorer start/stop, "localize now" and localization score~~ - done, plus live map view.
*   Nav2 configuration (costmaps with lidar + ToF hazard points) for route following.
*   Physical-rover validation of autonomous mapping and multi-session localization.
