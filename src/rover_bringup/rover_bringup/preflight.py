"""AutoNav preflight: check that everything is ready for a field test.

  ros2 run rover_bringup preflight                # all checks, the rover does not move
  ros2 run rover_bringup preflight --motion       # + short timed moves to measure speed / turn
                                                  #   rate and check motor, lidar and IMU directions
  ros2 run rover_bringup preflight --motion --apply   # + write measured speeds to explorer.yaml

Results are PASS / WARN / FAIL. Exit code 1 if anything FAILs.
Stop the running stack first (the checks need the serial ports and the I2C sensor).
"""
import glob
import grp
import json
import math
import os
import re
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time

PASS, WARN, FAIL, INFO = 'PASS', 'WARN', 'FAIL', 'INFO'
COLOR = {PASS: '\033[32m', WARN: '\033[33m', FAIL: '\033[31m', INFO: '\033[36m'}
WS_PACKAGES = ['rover_bringup', 'rover_control', 'rover_dashboard', 'rover_nav', 'vl53l5cx_ros',
               'ydlidar_ros2_driver']


class Report:
    def __init__(self):
        self.rows = []
        self.tty = sys.stdout.isatty()

    def add(self, level, name, msg, fix=''):
        self.rows.append((level, name, msg, fix))
        c, r = (COLOR[level], '\033[0m') if self.tty else ('', '')
        print(f'  {c}{level:4s}{r}  {name:28s} {msg}')
        if fix and level in (WARN, FAIL):
            print(f'        {"":28s} -> {fix}')

    def section(self, title):
        print(f'\n== {title}')

    def summary(self):
        n = {k: sum(1 for r in self.rows if r[0] == k) for k in (PASS, WARN, FAIL)}
        print(f'\nSummary: {n[PASS]} passed, {n[WARN]} warnings, {n[FAIL]} failed')
        if n[FAIL]:
            print('NOT READY - fix the FAIL items above.')
        elif n[WARN]:
            print('READY with warnings - read them before driving.')
        else:
            print('READY for field testing.')
        return n


def sh(cmd, timeout=10):
    try:
        return subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout).stdout
    except subprocess.TimeoutExpired:
        return ''


def share(pkg):
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory(pkg)


def ws_root():
    # <ws>/install/rover_bringup/lib/python3.x/site-packages/rover_bringup/preflight.py
    p = os.path.abspath(__file__)
    while p != '/' and not os.path.isdir(os.path.join(p, 'src', 'rover_bringup')):
        p = os.path.dirname(p)
    return p if p != '/' else os.path.expanduser('~/autonav_ws')


# ====================================================================== software
def check_software(r):
    r.section('Software')
    distro = os.environ.get('ROS_DISTRO')
    r.add(PASS if distro == 'jazzy' else FAIL, 'ROS 2', distro or 'not sourced',
          'source /opt/ros/jazzy/setup.bash')
    dom = os.environ.get('ROS_DOMAIN_ID')
    r.add(PASS if dom == '7' else WARN, 'ROS_DOMAIN_ID', dom or 'unset (0)',
          'the laptop must use the same value (README uses 7)')
    rmw = os.environ.get('RMW_IMPLEMENTATION', 'default (fastdds)')
    r.add(PASS if 'cyclonedds' in rmw else WARN, 'RMW', rmw, 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp')

    missing = []
    for pkg in WS_PACKAGES + ['slam_toolbox', 'nav2_map_server']:
        try:
            share(pkg)
        except Exception:
            missing.append(pkg)
    r.add(FAIL if missing else PASS, 'ROS packages', 'missing: ' + ', '.join(missing) if missing
          else 'all found', 'colcon build in the workspace / apt install the missing ones')

    ws = ws_root()
    stale = []
    for pkg in WS_PACKAGES[:-1]:
        src = os.path.join(ws, 'src', pkg)
        inst = os.path.join(ws, 'install', pkg)
        if not os.path.isdir(src) or not os.path.isdir(inst):
            continue
        newest_src = max((os.path.getmtime(f) for f in glob.glob(f'{src}/**/*', recursive=True)
                          if f.endswith(('.py', '.yaml', '.html', '.xml'))), default=0)
        newest_inst = max((os.path.getmtime(f) for f in glob.glob(f'{inst}/**/*', recursive=True)
                           if os.path.isfile(f)), default=0)
        if newest_src > newest_inst + 1:
            stale.append(pkg)
    r.add(WARN if stale else PASS, 'Build up to date', 'source newer than build: ' + ', '.join(stale)
          if stale else 'yes', f'cd {ws} && colcon build')

    mods = {'serial': 'pyserial', 'scipy': 'scipy', 'numpy': 'numpy', 'PIL': 'pillow',
            'yaml': 'pyyaml', 'smbus2': 'smbus2', 'vl53l5cx_ctypes': 'vl53l5cx-ctypes'}
    bad = []
    for m in mods:
        try:
            __import__(m)
        except Exception:
            bad.append(mods[m])
    r.add(FAIL if bad else PASS, 'Python modules', 'missing: ' + ', '.join(bad) if bad else 'all importable',
          'pip install --user --break-system-packages ' + ' '.join(bad))


# ====================================================================== system
def check_system(r):
    r.section('System')
    groups = {g.gr_name for g in grp.getgrall() if os.environ.get('USER', '') in g.gr_mem}
    groups.add(grp.getgrgid(os.getgid()).gr_name)
    miss = [g for g in ('dialout', 'i2c') if g not in groups]
    r.add(FAIL if miss else PASS, 'User groups', 'missing ' + ', '.join(miss) if miss else 'dialout, i2c',
          'sudo usermod -aG dialout,i2c $USER  (then log out/in)')

    try:
        thr = int(open('/sys/devices/platform/soc/soc:firmware/get_throttled').read().strip(), 0)
        now = []
        if thr & 0x1: now.append('UNDERVOLTAGE NOW')
        if thr & 0x4: now.append('throttled now')
        past = []
        if thr & 0x10000: past.append('undervoltage since boot')
        if thr & 0x40000: past.append('throttled since boot')
        lvl = FAIL if thr & 0x1 else WARN if (thr & 0x50005) else PASS
        r.add(lvl, 'Power supply', ', '.join(now + past) or 'no undervoltage / throttling',
              'use a 5 V / 5 A supply or a better battery regulator; undervoltage causes USB dropouts')
    except Exception:
        r.add(INFO, 'Power supply', 'cannot read throttling state')
    try:
        t = int(open('/sys/class/thermal/thermal_zone0/temp').read()) / 1000
        r.add(PASS if t < 70 else WARN, 'CPU temperature', f'{t:.0f} C', 'add a fan / heatsink')
    except Exception:
        pass

    du = shutil.disk_usage(os.path.expanduser('~'))
    r.add(PASS if du.free > 2e9 else WARN, 'Disk space', f'{du.free / 1e9:.1f} GB free',
          'free some space (maps + bags need room)')
    maps = os.path.expanduser('~/maps')
    os.makedirs(maps, exist_ok=True)
    r.add(PASS if os.access(maps, os.W_OK) else FAIL, 'Maps folder', maps, f'check permissions of {maps}')

    ips = [ip for ip in sh('hostname -I').split() if ':' not in ip]
    r.add(PASS if ips else WARN, 'Network', ', '.join(ips) or 'no IP address',
          'connect Wi-Fi or make the Pi a hotspot (nmcli device wifi hotspot ...) for the dashboard')
    if ips:
        r.add(INFO, 'Dashboard URL', f'http://{ips[0]}:8080')

    busy = []
    with socket.socket() as s:
        if s.connect_ex(('127.0.0.1', 8080)) == 0:
            busy.append('port 8080 in use')
    if sh('systemctl is-active autonav').strip() == 'active':
        busy.append('autonav service is running')
    running = sh("pgrep -af 'esp32_bridge|ydlidar_ros2_driver_node|vl53l5cx_node|explorer|dashboard' "
                 "| grep -v -E 'pgrep|preflight|field_test'").strip()
    if running:
        busy.append(f'{len(running.splitlines())} AutoNav process(es) already running')
    r.add(FAIL if busy else PASS, 'Stack not already running', '; '.join(busy) or 'ok',
          'stop it first: sudo systemctl stop autonav  /  Ctrl-C the running launch')
    return not busy


# ====================================================================== configuration
def check_config(r):
    r.section('Configuration')
    import yaml
    try:
        ex = yaml.safe_load(open(os.path.join(share('rover_nav'), 'config', 'explorer.yaml')))
        e = ex['explorer']['ros__parameters']
        defaults = [k for k, v in (('lin_speed', 0.15), ('ang_speed', 0.6), ('robot_radius', 0.15))
                    if abs(float(e.get(k, v)) - v) < 1e-9]
        r.add(WARN if defaults else PASS, 'Explorer speeds / size',
              f"lin {e['lin_speed']} m/s, ang {e['ang_speed']} rad/s, radius {e['robot_radius']} m" +
              (' (defaults - not measured)' if defaults else ''),
              'measure with: field_test.sh --motion --apply  (and set robot_radius)')
        r.add(INFO, 'Explorer ToF use', f"use_tof: {e.get('use_tof')}")
    except Exception as ex_:
        r.add(FAIL, 'Explorer config', f'unreadable: {ex_}')
    try:
        b = yaml.safe_load(open(os.path.join(share('rover_control'), 'config', 'esp32_bridge.yaml')))
        b = b['esp32_bridge']['ros__parameters']
        r.add(PASS if b.get('line_ending') == '\n' else FAIL, 'Bridge line ending',
              repr(b.get('line_ending')), 'must be "\\n" for ESP32_Code.ino')
        r.add(INFO, 'Bridge drive speed', f"drive_pwm {b.get('drive_pwm', 0)} "
              f"({'firmware default 200' if not b.get('drive_pwm') else 'set'})")
    except Exception as ex_:
        r.add(FAIL, 'Bridge config', f'unreadable: {ex_}')
    cal = os.path.expanduser('~/.ros/vl53l5cx_floor.yaml')
    if os.path.exists(cal):
        c = yaml.safe_load(open(cal)) or {}
        r.add(PASS, 'ToF floor calibration', f"height {c.get('mount_height_m')} m, tilt "
              f"{c.get('tilt_down_deg')} deg ({c.get('calibrated', '?')})")
    else:
        r.add(WARN, 'ToF floor calibration', 'not done (using vl53l5cx.yaml numbers)',
              'with the stack running: ros2 service call /tof/calibrate_floor std_srvs/srv/Trigger')
    maps = sorted(glob.glob(os.path.expanduser('~/maps/*.posegraph')))
    r.add(INFO, 'Saved maps', ', '.join(os.path.basename(m)[:-10] for m in maps) or 'none yet')


# ====================================================================== serial ports
def find_ports(r):
    r.section('Serial devices')
    from .setup_serial_ports import probe
    devs = sorted(set(os.path.realpath(p) for p in glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')))
    r.add(PASS if len(devs) >= 2 else FAIL if not devs else WARN, 'USB serial devices',
          ', '.join(devs) or 'none', 'plug in the ESP32 and the lidar')
    esp = lidar = None
    named = {n: os.path.realpath(f'/dev/{n}') for n in ('rover_esp32', 'rover_lidar')
             if os.path.exists(f'/dev/{n}')}
    if len(named) == 2:
        r.add(PASS, 'Fixed port names', f"rover_esp32 -> {named['rover_esp32']}, "
              f"rover_lidar -> {named['rover_lidar']}")
        esp, lidar = '/dev/rover_esp32', '/dev/rover_lidar'
        role = probe(esp)
        if role != 'esp32':
            r.add(FAIL, 'rover_esp32 is the ESP32', f'device answered like a {role}',
                  'devices swapped USB sockets? re-run: ros2 run rover_bringup setup_serial_ports --install')
            esp = None
    else:
        r.add(WARN, 'Fixed port names', 'not set up - lidar and ESP32 can grab each other\'s port',
              'ros2 run rover_bringup setup_serial_ports --install  (both plugged in)')
        print('        probing devices (up to 8 s each)...')
        for d in devs:
            role = probe(d)
            if role == 'esp32' and not esp:
                esp = d
            elif role == 'lidar' and not lidar:
                lidar = d
    r.add(PASS if esp else FAIL, 'ESP32 found', esp or 'no device answers like ESP32_Code.ino',
          'is it flashed, powered and connected?')
    r.add(PASS if lidar else FAIL, 'Lidar port', lidar or 'not found', 'check the lidar USB cable')
    return esp, lidar


class Esp32Reader:
    """Reads the ESP32 stream in the background; keeps the latest IMU sample."""

    def __init__(self, port):
        import serial
        s = serial.Serial()
        s.port, s.baudrate, s.timeout = port, 115200, 0.05
        s.open()           # DTR/RTS untouched: changing them reboots the ESP32
        self.ser, self.lock = s, threading.Lock()
        self.samples, self.lines, self.run = [], [], True
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        buf = b''
        while self.run:
            try:
                buf += self.ser.read(512)
            except Exception:
                return
            *lines, buf = buf.split(b'\n')
            for ln in lines:
                t = ln.decode(errors='replace').strip()
                if not t:
                    continue
                with self.lock:
                    if t[0].isdigit() and t.count(',') == 8:
                        try:
                            self.samples.append((time.time(), [float(v) for v in t.split(',')]))
                        except ValueError:
                            pass
                    else:
                        self.lines.append(t)

    def send(self, text):
        self.ser.write((text + '\n').encode())

    def latest(self):
        with self.lock:
            return self.samples[-1] if self.samples else None

    def since(self, t0):
        with self.lock:
            return [s for s in self.samples if s[0] >= t0]

    def close(self):
        self.run = False
        time.sleep(0.1)
        try:
            self.send('x')
        except Exception:
            pass
        self.ser.close()


def check_esp32(r, port):
    r.section('ESP32 (motors + IMU)')
    try:
        esp = Esp32Reader(port)
    except Exception as e:
        r.add(FAIL, 'Open ESP32 port', str(e), 'is another program using it?')
        return None
    esp.send('x')
    # After a reboot the firmware waits 2 s and calibrates the gyro (2000 samples at ~100 Hz,
    # ~20 s) before streaming. Wait for that rather than poking it.
    end = time.time() + 4.0
    while time.time() < end and not esp.since(time.time() - 1.0):
        time.sleep(0.2)
        if any('calibrating' in ln or 'ESP32 +' in ln for ln in esp.lines):
            print('        ESP32 is calibrating its gyro (~20 s) - keep the rover still...')
            end = time.time() + 30.0
            while time.time() < end and not any('calibration complete' in ln or 'IMU init failed' in ln
                                                for ln in esp.lines):
                time.sleep(0.3)
            end = time.time() + 2.0
    if any('IMU init failed' in ln for ln in esp.lines):
        r.add(FAIL, 'IMU', 'firmware reports "IMU init failed"', 'check ICM-20948 wiring (SDA 21, SCL 22)')
    t0 = time.time()
    time.sleep(2.0)
    s = esp.since(t0)
    if not s:
        # the stream may have been switched off with 'q' (a toggle): ask and read the answer
        n = len(esp.lines)
        esp.send('q')
        time.sleep(0.8)
        reply = ' '.join(esp.lines[n:])
        if 'stream OFF' in reply:          # it was on (just silent) - restore it
            esp.send('q')
            time.sleep(0.5)
        t0 = time.time()
        time.sleep(2.0)
        s = esp.since(t0)
    lines = ' | '.join(esp.lines[-4:])
    if not s:
        r.add(FAIL, 'IMU stream', f'no CSV samples (last lines: {lines or "none"})',
              'check the firmware / IMU; type h in a serial monitor to see its state')
        return esp
    rate = len(s) / 2.0
    v = [x[1] for x in s]
    acc = sum(math.sqrt(a[1] ** 2 + a[2] ** 2 + a[3] ** 2) for a in v) / len(v) / 1000.0
    gz = sum(abs(a[6]) for a in v) / len(v)
    drift = (v[-1][8] - v[0][8]) / max(s[-1][0] - s[0][0], 1e-3)
    r.add(PASS if rate >= 20 else WARN, 'IMU stream', f'{rate:.0f} Hz')
    r.add(PASS if 0.85 < acc < 1.15 else WARN, 'IMU gravity', f'{acc:.2f} g', 'IMU reading looks off')
    r.add(PASS if gz < 0.5 and abs(drift) < 0.2 else WARN, 'Gyro at rest',
          f'|gz| {gz:.3f} deg/s, yaw drift {drift * 60:+.1f} deg/min',
          "keep the rover still and send 'cal' (recalibrate gyro)")
    r.add(PASS, 'Motor command link', 'stop command acknowledged' if any('STOP' in ln for ln in esp.lines)
          else 'port open (no reply seen)')
    return esp


# ====================================================================== ToF
def check_tof(r):
    r.section('VL53L5CX ToF')
    out = sh('i2cdetect -y 1 2>/dev/null')
    present = re.search(r'^20:.*\b29\b', out, re.M) is not None
    r.add(PASS if present else FAIL, 'I2C 0x29', 'detected' if present else 'not found on bus 1',
          'check wiring (SDA GPIO2, SCL GPIO3, 3.3 V)')
    if not present:
        return
    try:
        import vl53l5cx_ctypes as vl
        from smbus2 import SMBus
        t0 = time.time()
        dev = vl.VL53L5CX(i2c_dev=SMBus(1))
        dev.set_resolution(64)
        dev.set_ranging_frequency_hz(15)
        dev.start_ranging()
        init_s = time.time() - t0
        frames = []
        end = time.time() + 3.0
        while time.time() < end and len(frames) < 10:
            if dev.data_ready():
                d = dev.get_data()
                frames.append((list(d.distance_mm[0]), list(d.target_status[0])))
            time.sleep(0.01)
        dev.stop_ranging()
    except Exception as e:
        r.add(FAIL, 'ToF ranging', str(e))
        return
    r.add(PASS if len(frames) >= 8 else WARN, 'ToF ranging', f'{len(frames)} frames in 3 s '
          f'(init {init_s:.1f} s)')
    if not frames:
        return
    dist, stat = frames[-1]
    valid = [d for d, s_ in zip(dist, stat) if s_ in (5, 6, 9)]
    med = sorted(valid)[len(valid) // 2] if valid else 0
    if valid and med < 30:
        r.add(FAIL, 'ToF readings', f'median {med} mm - something is on the sensor',
              'remove the protective film / clean the cover')
    else:
        r.add(PASS if len(valid) >= 32 else WARN, 'ToF readings',
              f'{len(valid)}/64 valid zones, median {med} mm', 'aim the sensor at the floor ahead')


# ====================================================================== lidar
class LidarRun:
    """Starts the YDLidar driver as a child process and collects /scan."""

    def __init__(self, port):
        import rclpy
        import yaml
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
        cfg = yaml.safe_load(open(os.path.join(share('rover_nav'), 'config', 'ydlidar_x4.yaml')))
        cfg['ydlidar_ros2_driver_node']['ros__parameters']['port'] = port
        tmp = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False)
        yaml.safe_dump(cfg, tmp)
        tmp.close()
        self.log = tempfile.NamedTemporaryFile('w+', suffix='.log', delete=False)
        self.proc = subprocess.Popen(
            ['ros2', 'launch', 'ydlidar_ros2_driver', 'ydlidar_launch.py', f'params_file:={tmp.name}'],
            stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node('preflight')
        self.scans = []
        self.node.create_subscription(LaserScan, 'scan', self.scans.append, qos_profile_sensor_data)

    def spin(self, sec):
        import rclpy
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def wait_scans(self, n, timeout):
        end = time.time() + timeout
        while time.time() < end and len(self.scans) < n:
            self.spin(0.1)
        return len(self.scans) >= n

    def last_scan(self, settle=0.3):
        k = len(self.scans)
        self.spin(settle)
        end = time.time() + 2.0
        while len(self.scans) <= k and time.time() < end:
            self.spin(0.05)
        return self.scans[-1]

    def stop(self):
        try:
            os.killpg(self.proc.pid, signal.SIGINT)
            self.proc.wait(timeout=8)
        except Exception:
            try:
                os.killpg(self.proc.pid, signal.SIGKILL)
            except Exception:
                pass
        self.node.destroy_node()


def check_lidar(r, port):
    r.section('Lidar')
    try:
        lid = LidarRun(port)
    except Exception as e:
        r.add(FAIL, 'Start lidar driver', str(e))
        return None
    if not lid.wait_scans(5, 20.0):
        lid.log.seek(0)
        tail = ' | '.join(lid.log.read().strip().splitlines()[-2:])
        r.add(FAIL, 'Lidar scans', f'no /scan within 20 s ({tail[-160:]})',
              'check the port / power; the X4 needs 5 V with enough current')
        lid.stop()
        return None
    t0, k0 = time.time(), len(lid.scans)
    lid.spin(3.0)
    rate = (len(lid.scans) - k0) / (time.time() - t0)
    s = lid.scans[-1]
    import numpy as np
    rng = np.asarray(s.ranges, dtype=float)
    ok = np.isfinite(rng) & (rng > s.range_min) & (rng < s.range_max)
    r.add(PASS if rate >= 6 else WARN, 'Lidar scan rate', f'{rate:.1f} Hz', 'expected ~10 Hz')
    r.add(PASS if ok.mean() > 0.4 else WARN, 'Lidar valid points', f'{ok.sum()}/{len(rng)} '
          f'({ok.mean():.0%})', 'nothing in range? reflective/black surfaces? dirty window?')
    ang = s.angle_min + np.arange(len(rng)) * s.angle_increment

    def sector(deg):
        a = (np.degrees(ang) - deg + 180) % 360 - 180
        m = ok & (np.abs(a) < 10)
        return f'{np.median(rng[m]):.2f} m' if m.any() else 'no return'
    r.add(INFO, 'Lidar distances', f'front {sector(0)}, left {sector(90)}, back {sector(180)}, '
          f'right {sector(-90)}')
    print(f'        {"":28s} -> check these against reality (tape measure). If "front" is not what is in '
          'front of the rover, fix the lidar TF / mounting.')
    return lid


# ====================================================================== motion
def motion_test(r, esp, lid, apply, assume_yes):
    r.section('Motion test (rover will move!)')
    if esp is None or lid is None:
        r.add(WARN, 'Motion test', 'skipped (needs the ESP32 and the lidar)')
        return
    if not assume_yes:
        print('  The rover will drive FORWARD ~0.5 s, BACK ~0.5 s, turn LEFT ~0.5 s and RIGHT ~0.5 s.')
        print('  Put it on the floor with >= 1 m free in front and nothing within 0.5 m around it.')
        if input('  Type "go" to continue: ').strip().lower() != 'go':
            r.add(WARN, 'Motion test', 'skipped by user')
            return
    import yaml
    from rover_nav.grid_utils import scan_to_points, voxel_downsample
    from rover_nav.laser_odometry import Keyframe, icp
    try:
        bcfg = yaml.safe_load(open(os.path.join(share('rover_control'), 'config', 'esp32_bridge.yaml')))
        pwm = int(bcfg['esp32_bridge']['ros__parameters'].get('drive_pwm', 0))
    except Exception:
        pwm = 0
    if pwm > 0:
        esp.send(f'v {pwm}')
    pwm = pwm or 200

    def pts(scan):
        return voxel_downsample(scan_to_points(scan, 8.0, 0.12), 0.04)

    def move(cmd, ms):
        a = lid.last_scan()
        imu0 = esp.latest()
        esp.send(f'{cmd} {ms}')                 # timed move: firmware stops by itself
        lid.spin(ms / 1000.0 + 1.0)             # move + settle
        b = lid.last_scan(0.2)
        imu1 = esp.latest()
        dyaw_imu = math.radians(imu1[1][8] - imu0[1][8]) if imu0 and imu1 else None
        pa, pb = pts(a), pts(b)
        if len(pa) < 40 or len(pb) < 40:
            return None, dyaw_imu, {'ok': False}
        kf = Keyframe(pa)
        # try the IMU yaw, its negative (IMU may be upside down) and zero as initial guesses
        guesses = {0.0, dyaw_imu or 0.0, -(dyaw_imu or 0.0)}
        best = max((icp(pb, kf, (0.0, 0.0, g), iters=30, thr0=0.6) for g in guesses),
                   key=lambda res: res[1]['inliers'])
        return best[0], dyaw_imu, best[1]

    ms = 500
    fwd, _, i1 = move('w', ms)
    back, _, _ = move('s', ms)
    left, imu_left, i3 = move('a', ms)
    right, imu_right, _ = move('d', ms)
    esp.send('x')

    if fwd is None or not i1.get('ok'):
        r.add(WARN, 'Forward drive', 'could not measure (scan match failed - need walls within 8 m)')
    else:
        dx, dy = fwd[0], fwd[1]
        if abs(dx) < 0.02:
            r.add(FAIL, 'Forward drive', f'rover did not move ({dx * 100:.1f} cm)',
                  'motor power / L298N wiring / battery; try a higher drive_pwm')
        elif dx < 0:
            r.add(FAIL, 'Forward drive', f"'w' moved {dx * 100:.0f} cm BACKWARDS in the lidar frame",
                  'either the motors are reversed (swap IN pins) or the lidar faces backwards (fix its TF)')
        else:
            v = dx / (ms / 1000.0)
            r.add(PASS if abs(dy) < 0.4 * dx else WARN, 'Forward drive',
                  f'{dx * 100:.1f} cm in {ms} ms -> {v:.2f} m/s (sideways {dy * 100:+.1f} cm)',
                  'large sideways motion: lidar yaw offset or one side much weaker')
            r.lin_speed = round(v, 3)
            if v > 0.35:
                r.add(WARN, 'Forward speed', f'{v:.2f} m/s is fast for mapping',
                      'lower drive_pwm in rover_control/config/esp32_bridge.yaml (e.g. 150)')
    lid_rot = left[2] if (left is not None and i3.get('ok')) else None
    turn = lid_rot if lid_rot is not None else imu_left
    if turn is None:
        r.add(WARN, 'Turn left', 'could not measure (no IMU data and no scan match)')
    elif abs(turn) < math.radians(3):
        r.add(FAIL, 'Turn left', f'only {math.degrees(turn):.1f} deg in {ms} ms', 'motors / power')
    else:
        w = abs(turn) / (ms / 1000.0)
        r.ang_speed = round(w, 3)
        src = 'lidar' if lid_rot is not None else 'IMU'
        detail = (f"lidar {math.degrees(lid_rot):+.0f} deg" if lid_rot is not None else 'lidar n/a') + \
                 (f", IMU {math.degrees(imu_left):+.0f} deg" if imu_left is not None else ', IMU n/a')
        if lid_rot is not None and imu_left is not None:
            if lid_rot > 0 and imu_left > 0:
                r.add(PASS, 'Turn left', f"'a' turns counter-clockwise ({detail}) -> {w:.2f} rad/s")
            elif lid_rot > 0 > imu_left:
                r.add(WARN, 'IMU orientation', f'IMU yaw sign is reversed ({detail})',
                      'IMU mounted upside down; fine for now (navigation does not use the IMU yet)')
            elif lid_rot < 0 < imu_left:
                r.add(FAIL, 'Lidar orientation', f'lidar sees the rotation mirrored ({detail})',
                      "flip 'inverted' in src/rover_nav/config/ydlidar_x4.yaml")
            else:
                r.add(FAIL, 'Turn left', f"'a' turns clockwise ({detail})",
                      'left/right motors are swapped: swap the L298N outputs or the IN pin defines')
        else:
            r.add(PASS if turn > 0 else FAIL, 'Turn left', f"'a' turned {math.degrees(turn):+.0f} deg "
                  f"({src} only) -> {w:.2f} rad/s", "'a' must turn counter-clockwise")
        if w > 1.5:
            r.add(WARN, 'Turn rate', f'{w:.2f} rad/s is fast for clean lidar scans',
                  'lower drive_pwm in rover_control/config/esp32_bridge.yaml (e.g. 150)')
    if apply and getattr(r, 'lin_speed', None) and getattr(r, 'ang_speed', None):
        ws = ws_root()
        path = os.path.join(ws, 'src', 'rover_nav', 'config', 'explorer.yaml')
        txt = open(path).read()
        txt = re.sub(r'(lin_speed:\s*)[0-9.]+', rf'\g<1>{r.lin_speed}', txt)
        txt = re.sub(r'(ang_speed:\s*)[0-9.]+', rf'\g<1>{r.ang_speed}', txt)
        open(path, 'w').write(txt)
        res = subprocess.run(f'cd {ws} && colcon build --packages-select rover_nav',
                             shell=True, capture_output=True, text=True)
        r.add(PASS if res.returncode == 0 else FAIL, 'Apply speeds',
              f'explorer.yaml: lin_speed {r.lin_speed}, ang_speed {r.ang_speed}' +
              (' (rebuilt)' if res.returncode == 0 else ' (rebuild failed)'))
    elif getattr(r, 'lin_speed', None) or getattr(r, 'ang_speed', None):
        r.add(INFO, 'Measured speeds', f"lin_speed {getattr(r, 'lin_speed', '?')} m/s, "
              f"ang_speed {getattr(r, 'ang_speed', '?')} rad/s (use --apply to save)")


# ====================================================================== main
def main():
    args = sys.argv[1:]
    motion, apply, yes = '--motion' in args, '--apply' in args, '--yes' in args
    r = Report()
    print('AutoNav preflight' + (' + motion test' if motion else '') +
          f"  ({time.strftime('%Y-%m-%d %H:%M')})")
    check_software(r)
    free = check_system(r)
    check_config(r)
    esp = lid = None
    if free:
        esp_port, lidar_port = find_ports(r)
        if esp_port:
            esp = check_esp32(r, esp_port)
        check_tof(r)
        if lidar_port:
            lid = check_lidar(r, lidar_port)
        if motion:
            motion_test(r, esp, lid, apply, yes)
    else:
        r.add(WARN, 'Hardware checks', 'skipped because the stack is already running')
    if esp:
        esp.close()
    if lid:
        lid.stop()
    n = r.summary()
    try:
        log = os.path.expanduser('~/autonav_logs')
        os.makedirs(log, exist_ok=True)
        with open(os.path.join(log, time.strftime('preflight_%Y%m%d_%H%M%S.json')), 'w') as f:
            json.dump([dict(zip(('level', 'check', 'result', 'fix'), x)) for x in r.rows], f, indent=1)
    except OSError:
        pass
    sys.exit(1 if n[FAIL] else 0)


if __name__ == '__main__':
    main()
