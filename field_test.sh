#!/usr/bin/env bash
# AutoNav field test: check everything, then start the full autonomous-mapping stack.
#
#   ./field_test.sh                    preflight checks, then launch (explorer waits for Start)
#   ./field_test.sh --motion           + short timed moves to measure speeds and check directions
#   ./field_test.sh --motion --apply   + save the measured speeds to explorer.yaml
#   ./field_test.sh --record           + record a rosbag of the run to ~/autonav_logs (for debugging)
#   ./field_test.sh --check-only       only run the checks
#   ./field_test.sh --force            launch even if a check FAILs
#   ./field_test.sh --skip-checks      launch straight away
# Anything else is passed to the launch file, e.g.:
#   ./field_test.sh explore:=true map_name:=warehouse_a      tof:=false      continue_map:=warehouse_a
#
# Then open the dashboard (URL printed below) -> "Autonomous mapping" -> Start.
# Ctrl-C stops everything (the rover is told to stop).
set -uo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CHECK_ARGS=()
LAUNCH_ARGS=()
CHECKS=1 FORCE=0 ONLY=0 RECORD=0
for a in "$@"; do
  case "$a" in
    --motion|--apply|--yes) CHECK_ARGS+=("$a") ;;
    --force) FORCE=1 ;;
    --check-only) ONLY=1 ;;
    --skip-checks) CHECKS=0 ;;
    --record) RECORD=1 ;;
    -h|--help) sed -n '2,17p' "$0"; exit 0 ;;
    *) LAUNCH_ARGS+=("$a") ;;
  esac
done

# --- environment (same as ~/.bashrc on the Pi; overridable)
set +u
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"

# --- build if needed (missing install, or sources newer than the build)
if [[ ! -f "$WS/install/setup.bash" ]] || \
   [[ -n "$(find "$WS/src" -path '*/rf2o_laser_odometry' -prune -o \
            \( -name '*.py' -o -name '*.yaml' -o -name '*.html' -o -name '*.xml' \) \
            -newer "$WS/install/setup.bash" -print -quit)" ]]; then
  echo "== Building workspace"
  (cd "$WS" && colcon build) || { echo "Build failed"; exit 1; }
fi
source "$WS/install/setup.bash"
set -u

if systemctl is-active --quiet autonav 2>/dev/null; then
  echo "The autonav boot service is running the stack already."
  echo "Stop it for a field test from here:  sudo systemctl stop autonav"
  exit 1
fi

# --- preflight
if (( CHECKS )); then
  ros2 run rover_bringup preflight "${CHECK_ARGS[@]}"
  rc=$?
  (( ONLY )) && exit $rc
  if (( rc != 0 && !FORCE )); then
    echo; echo "Fix the FAIL items (or use --force to launch anyway)."; exit $rc
  fi
fi

# --- optional recording
LOGDIR="$HOME/autonav_logs"; mkdir -p "$LOGDIR"
STAMP="$(date +%Y%m%d_%H%M%S)"
BAG_PID=""
if (( RECORD )); then
  ros2 bag record -o "$LOGDIR/bag_$STAMP" /scan /odom /tf /tf_static /imu /cmd_vel /rover/sent \
    /tof/distance_mm /tof/obstacle_detected /tof/cliff_detected /explorer/state /map \
    > "$LOGDIR/bag_$STAMP.log" 2>&1 &
  BAG_PID=$!
  echo "== Recording rosbag to $LOGDIR/bag_$STAMP"
fi
cleanup() { [[ -n "$BAG_PID" ]] && kill -INT "$BAG_PID" 2>/dev/null; wait 2>/dev/null; }
trap cleanup EXIT

IP="$(hostname -I | awk '{print $1}')"
echo
echo "== Starting AutoNav (log: $LOGDIR/run_$STAMP.log)"
echo "   Dashboard: http://${IP:-<pi-ip>}:8080  -> Autonomous mapping -> Start"
echo "   Ctrl-C to stop everything."
echo
ros2 launch rover_bringup autonomous_mapping.launch.py "${LAUNCH_ARGS[@]}" 2>&1 | tee "$LOGDIR/run_$STAMP.log"
