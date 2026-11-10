#!/usr/bin/env bash
# One-click prior-map build with FAST-LIVO2 on the AVIA / CICV bag, then crop and
# downsample the result for FAR-LIO localization.
#
# Output:
#   fast_livo/Log/PCD/all_downsampled_points.pcd   (FAST-LIVO2, on shutdown)
#   data/maps/avia_cicv_local_map.pcd              (FAR-LIO prior map)
#
# Usage: ./scripts/run_avia_mapping.sh [bag] [rate]
set -e

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${HERE}/env.sh"

BAG="${1:-/media/komasa/Data/Ubuntu文件备份/rosbags/avia_hk_cicv/cicv_avia.bag}"
RATE="${2:-1.0}"

PORT="${ROS_PORT:-11320}"
export ROS_MASTER_URI="http://localhost:${PORT}"

FASTLIVO_LOG="$(rospack find fast_livo)/Log/PCD"
MAP_OUT="/home/komasa/lio_ws/data/maps/avia_cicv_local_map.pcd"

CORE_PID=""
LAUNCH_PID=""
cleanup() {
  [[ -n "${LAUNCH_PID}" ]] && kill "${LAUNCH_PID}" 2>/dev/null || true
  [[ -n "${CORE_PID}" ]] && kill "${CORE_PID}" 2>/dev/null || true
}
trap cleanup EXIT

if ! timeout 2 rosnode list >/dev/null 2>&1; then
  roscore -p "${PORT}" > /tmp/farlio_avia_map_roscore.log 2>&1 &
  CORE_PID=$!
  sleep 4
fi

echo "[run_avia_mapping] launching FAST-LIVO2 (rviz disabled)"
roslaunch farlio_odometry avia_mapping_fastlivo.launch bag:="" rviz:=false \
  > /tmp/farlio_avia_mapping.launch.log 2>&1 &
LAUNCH_PID=$!
sleep 8

echo "[run_avia_mapping] playing ${BAG} at rate ${RATE}"
rosbag play -d 2 -r "${RATE}" "${BAG}" > /tmp/farlio_avia_mapping.play.log 2>&1

echo "[run_avia_mapping] bag finished, shutting FAST-LIVO2 down to write the PCD"
rosnode kill /laserMapping >/dev/null 2>&1 || true
for _ in $(seq 1 180); do
  if ls "${FASTLIVO_LOG}"/*.pcd >/dev/null 2>&1; then break; fi
  sleep 1
done
sleep 2

if [[ ! -f "${FASTLIVO_LOG}/all_downsampled_points.pcd" ]]; then
  echo "[run_avia_mapping] ERROR: FAST-LIVO2 did not write ${FASTLIVO_LOG}/all_downsampled_points.pcd"
  exit 1
fi

echo "[run_avia_mapping] cropping / downsampling prior map"
python3 "$(rospack find farlio_odometry)/scripts/prepare_prior_map.py" \
  --map "${FASTLIVO_LOG}/all_downsampled_points.pcd" \
  --out "${MAP_OUT}" \
  --voxel 0.5 --x-min -250 --x-max 80 --y-min -60 --y-max 320 --z-min -12 --z-max 20

echo "[run_avia_mapping] done: ${MAP_OUT}"
