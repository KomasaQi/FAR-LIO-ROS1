#!/usr/bin/env bash
# One-click FAR-LIO full-stack localization on the HitchOpen PIX (CICV) bag,
# with RViz visualization of the prior map, live scan, trajectory and EKF pose.
#
# Usage:
#   ./scripts/run_pix_localization.sh [bag] [roslaunch args...]
#   ./scripts/run_pix_localization.sh "" rviz:=false          # no bag, live sensor
#
# The script starts its own roscore on ROS_PORT (default 11322) unless a master
# is already reachable at that URI.
set -e

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${HERE}/env.sh"

BAG="${1:-/home/komasa/HitchOpen-THICV-Stack/src/launch/simple_racing/maps/Cicv_Map_Outside/pix_moving.bag}"
shift || true

PORT="${ROS_PORT:-11322}"
export ROS_MASTER_URI="http://localhost:${PORT}"

CORE_PID=""
cleanup() {
  if [[ -n "${CORE_PID}" ]]; then
    kill "${CORE_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

if ! timeout 2 rosnode list >/dev/null 2>&1; then
  echo "[run_pix_localization] starting roscore on port ${PORT}"
  roscore -p "${PORT}" > /tmp/farlio_pix_roscore.log 2>&1 &
  CORE_PID=$!
  sleep 4
fi

echo "[run_pix_localization] bag: ${BAG:-<live sensor>}"
roslaunch farlio_odometry pix_localization.launch bag:="${BAG}" rviz:="${RVIZ:-true}" "$@"
