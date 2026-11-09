#!/usr/bin/env bash
# Start the AutoNav stack automatically at boot (systemd).
#
#   bash ~/autonav_ws/src/rover_bringup/scripts/install_service.sh            # install + start
#   bash ~/autonav_ws/src/rover_bringup/scripts/install_service.sh --remove   # uninstall
#
# The rover does NOT drive at boot: the explorer waits for "Start" in the dashboard
# (http://<pi-ip>:8080). Logs: journalctl -u autonav -f
set -euo pipefail

SERVICE=/etc/systemd/system/autonav.service
if [[ "${1:-}" == "--remove" ]]; then
  sudo systemctl disable --now autonav.service || true
  sudo rm -f "$SERVICE"
  sudo systemctl daemon-reload
  echo "autonav service removed"
  exit 0
fi

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
if [[ ! -f "$WS/install/setup.bash" ]]; then
  echo "Build the workspace first: cd $WS && colcon build" >&2
  exit 1
fi
USER_NAME="$(id -un)"
DOMAIN="${ROS_DOMAIN_ID:-7}"

sudo tee "$SERVICE" > /dev/null <<EOF
[Unit]
Description=AutoNav rover (bridge, lidar, SLAM, explorer, ToF, dashboard)
After=network-online.target
Wants=network-online.target

[Service]
User=$USER_NAME
Environment=ROS_DOMAIN_ID=$DOMAIN
Environment=RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
Environment=ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ExecStart=/bin/bash -c 'source /opt/ros/jazzy/setup.bash && source $WS/install/setup.bash && exec ros2 launch rover_bringup autonomous_mapping.launch.py'
KillSignal=SIGINT
TimeoutStopSec=15
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable --now autonav.service
echo "Installed. Status: systemctl status autonav   Logs: journalctl -u autonav -f"
echo "Dashboard: http://$(hostname -I | awk '{print $1}'):8080"
