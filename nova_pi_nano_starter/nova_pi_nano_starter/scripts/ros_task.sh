#!/usr/bin/env bash
# Run on the Pi/Linux ROS host, normally through VS Code Remote-SSH.
set -eo pipefail
nova_distro="${1:-}"
nova_action="${2:-}"
nova_port="${3:-/dev/ttyACM0}"
if [[ ! "$nova_distro" =~ ^[a-z][a-z0-9_]*$ ]] || [[ ! -f "/opt/ros/$nova_distro/setup.bash" ]]; then
  echo 'Enter the ROS distribution installed on the Pi (list /opt/ros to check).' >&2
  exit 2
fi
nova_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "/opt/ros/$nova_distro/setup.bash"
cd "$nova_root/ros2_ws"
case "$nova_action" in
  build) exec colcon build --symlink-install --packages-select nova_hardware ;;
  run)
    if [[ ! -f install/setup.bash ]]; then
      echo 'Run the Pi: Build ROS2 package task first.' >&2
      exit 2
    fi
    source install/setup.bash
    exec ros2 run nova_hardware hardware_bridge --ros-args -p "port:=$nova_port"
    ;;
  *) echo 'Usage: bash ros_task.sh DISTRO build|run [SERIAL_PORT]' >&2; exit 2 ;;
esac
