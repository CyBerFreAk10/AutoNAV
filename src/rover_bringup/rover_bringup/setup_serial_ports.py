"""Give the ESP32 and the YDLidar their own fixed device names.

Both use a CP2102 USB-serial chip, so /dev/ttyUSB0/1 can swap and the YDLidar udev rule
names *both* /dev/ydlidar. This tool listens to each serial device to tell them apart
(the ESP32 prints '#' status lines and IMU CSV at 115200 baud) and writes udev rules
keyed on the physical USB port:

    /dev/rover_esp32   /dev/rover_lidar

  ros2 run rover_bringup setup_serial_ports            # detect and show the rules
  ros2 run rover_bringup setup_serial_ports --install  # also install them (asks for sudo)

Keep each device in the same USB socket afterwards. Plug in both devices first; if only
one is connected, its rule is written and the other one is kept from a previous run.
"""
import glob
import os
import re
import subprocess
import sys
import time

import serial

RULES = '/etc/udev/rules.d/99-autonav-serial.rules'
NAMES = {'esp32': 'rover_esp32', 'lidar': 'rover_lidar'}


def id_path(dev):
    out = subprocess.run(['udevadm', 'info', '-q', 'property', '-n', dev],
                         capture_output=True, text=True).stdout
    m = re.search(r'^ID_PATH=(.+)$', out, re.M)
    return m.group(1) if m else None


def looks_like_esp32(text):
    lines = [ln.strip() for ln in text.splitlines() if ln.strip()]
    csv = sum(1 for ln in lines if ln[0].isdigit() and ln.count(',') == 8)
    hashes = sum(1 for ln in lines if ln.startswith('#'))
    return csv >= 2 or hashes >= 1


def probe(dev):
    """'esp32', 'lidar' or None (busy / unreadable)."""
    try:
        s = serial.Serial()
        s.port, s.baudrate, s.timeout = dev, 115200, 0.1
        s.dtr = False      # don't reboot the ESP32
        s.rts = False
        s.open()
    except (serial.SerialException, OSError) as e:
        print(f'  {dev}: cannot open ({e}) - stop ROS nodes using it and retry')
        return None
    try:
        # Opening the port can reboot the ESP32; it is then silent for ~5 s while the IMU
        # calibrates. Listen for up to 8 s and ask for help text twice before calling it a lidar.
        data, asked = b'', 0
        start = time.time()
        while time.time() - start < 8.0:
            data += s.read(512)
            if looks_like_esp32(data.decode(errors='replace')):
                return 'esp32'
            if (asked == 0 and time.time() - start > 1.5) or (asked == 1 and time.time() - start > 6.0):
                s.write(b'h\n')
                asked += 1
        return 'lidar'
    finally:
        s.close()


def main():
    install = '--install' in sys.argv
    devs = sorted(set(os.path.realpath(p) for p in glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')))
    if not devs:
        sys.exit('No USB serial devices found. Plug in the ESP32 and the lidar.')
    found = {}
    print('Probing serial devices (up to 8 s each)...')
    for dev in devs:
        role = probe(dev)
        path = id_path(dev)
        if role is None:
            continue
        print(f'  {dev}: {role.upper():5s}  USB port {path}')
        if not path:
            print('    (no ID_PATH - cannot make a port-based rule)')
            continue
        if role in found:
            print(f'    WARNING: two devices look like the {role}; keeping the first')
            continue
        found[role] = path

    if 'esp32' not in found and len(devs) >= 2:
        print('No device answered like the ESP32 firmware (is it flashed and powered?).')

    rules = {}
    if os.path.exists(RULES):           # keep the other device's rule from a previous run
        for ln in open(RULES):
            for role, name in NAMES.items():
                if f'SYMLINK+="{name}"' in ln:
                    rules[role] = ln.rstrip('\n')
    for role, path in found.items():
        rules[role] = (f'SUBSYSTEM=="tty", ENV{{ID_PATH}}=="{path}", '
                       f'SYMLINK+="{NAMES[role]}", MODE="0666", GROUP="dialout"')
    if not rules:
        sys.exit('Nothing to write.')
    text = '# AutoNav: fixed names by physical USB port (rover_bringup setup_serial_ports)\n' + \
           '\n'.join(rules[r] for r in sorted(rules)) + '\n'
    print('\nudev rules:\n' + text)
    missing = [NAMES[r] for r in NAMES if r not in rules]
    if missing:
        print(f'Not configured yet: {", ".join(missing)} - plug it in and run this again.')
    if not install:
        print('Run again with --install to install these rules.')
        return
    subprocess.run(['sudo', 'tee', RULES], input=text, text=True, check=True,
                   stdout=subprocess.DEVNULL)
    subprocess.run(['sudo', 'udevadm', 'control', '--reload-rules'], check=True)
    subprocess.run(['sudo', 'udevadm', 'trigger', '--subsystem-match=tty'], check=True)
    time.sleep(1.0)
    for r in sorted(rules):
        p = f'/dev/{NAMES[r]}'
        print(f'{p} -> {os.path.realpath(p) if os.path.exists(p) else "not present"}')
    print('Done. Keep each device in the same USB socket.')


if __name__ == '__main__':
    main()
