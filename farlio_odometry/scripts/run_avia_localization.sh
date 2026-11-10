#!/usr/bin/env bash
# One-click FAR-LIO full-stack localization on the AVIA / CICV bag, with RViz.
#
# Pipeline: /livox/lidar (CustomMsg) -> converter -> FAR-LIO gicp_ext + 3D EKF,
# localizing against the FAST-LIVO2 prior map (data/maps/avia_cicv_local_map.pcd).
#
# Usage:
#   ./scripts/run_avia_localization.sh [bag] [roslaunch args...]
#   ./scripts/run_avia_localization.sh "" rviz:=false          # live AVIA
#
# Build the prior map first with ./scripts/run_avia_mapping.sh.
set -e

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${HERE}/env.sh"

BAG="${1:-/media/komasa/Data/Ubuntu文件备份/rosbags/avia_hk_cicv/cicv_avia.bag}"
shift || true

PORT="${ROS_PORT:-11321}"
export ROS_MASTER_URI="http://localhost:${PORT}"

CORE_PID=""
cleanup() {
  if [[ -n "${CORE_PID}" ]]; then
    kill "${CORE_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

if ! timeout 2 rosnode list >/dev/null 2>&1; then
  echo "[run_avia_localization] starting roscore on port ${PORT}"
  roscore -p "${PORT}" > /tmp/farlio_avia_roscore.log 2>&1 &
  CORE_PID=$!
  sleep 4
fi

echo "[run_avia_localization] bag: ${BAG:-<live sensor>}"
roslaunch farlio_odometry avia_localization.launch bag:="${BAG}" rviz:="${RVIZ:-true}" "$@"
