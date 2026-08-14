#!/usr/bin/env bash
# Capture data while the robot is frozen in front of a dynamic obstacle.
#
# Usage:
#   bash scripts/record_freeze_incident.sh [duration_sec] [out_dir]
#
# Defaults: 20 seconds, ./freeze_logs/<timestamp>/
#
# Run this AFTER you have positioned yourself in the robot's path and it has
# stopped moving. Ctrl+C ends early.

set -u

DURATION="${1:-20}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="${2:-./freeze_logs/${STAMP}}"
mkdir -p "${OUT_DIR}"

LOG="${OUT_DIR}/runtime.log"
BAG_DIR="${OUT_DIR}/bag"
BAG_LOG="${OUT_DIR}/bag_record.log"
DIAG_BT_LOG="${OUT_DIR}/diag_bt_navigator.log"
DIAG_OBSTACLE_LOG="${OUT_DIR}/diag_obstacle.log"
DIAG_COSTMAP_LOG="${OUT_DIR}/diag_costmap_decay.log"
LIVE_CSV="${OUT_DIR}/live_sample.csv"

TOPICS=(
  /rosout
  /parameter_events
  /tf /tf_static
  /scan /livox/lidar
  /Odometry /odom_2d /odom
  /local_costmap/costmap /local_costmap/costmap_updates
  /local_costmap/costmap_raw
  /local_costmap/published_footprint
  /global_costmap/costmap /global_costmap/costmap_updates
  /global_costmap/published_footprint
  /plan /received_global_plan /local_plan
  /cmd_vel /cmd_vel_nav /cmd_vel_smoothed /cmd_vel_executed
  /motion_source
  /behavior_tree_log
  /navigate_to_pose/_action/feedback
  /navigate_to_pose/_action/status
  /navigate_through_poses/_action/status
  /goal_pose
  /diagnostics
  /navigation_manager/ready
  /navigation_manager/state /navigation_manager/detail
)

PIDS_TO_CLEAN=()

log_note() {
  echo "[record] $*" | tee -a "${LOG}"
}

append_section_header() {
  {
    echo
    echo "===== $* ====="
  } >> "${LOG}"
}

append_cmd() {
  local title="$1"
  shift
  append_section_header "${title}"
  {
    "$@"
    echo
  } >> "${LOG}" 2>&1 || true
}

append_process_snapshot() {
  local title="$1"
  append_section_header "${title}"
  {
    ps -eo pid,ppid,pgid,etime,cmd --sort=pid \
      | grep -E 'navigation_manager|multi_waypoint_nav|nav_script|bt_navigator|controller_server|planner_server|behavior_server|velocity_smoother|pc_to_laserscan|odom_to_tf|map_server|lifecycle_manager|super_lio|livox|record_freeze_incident' \
      | grep -v grep || true
    echo
  } >> "${LOG}" 2>&1
}

dump_param() {
  local node_name="$1"
  append_section_header "ros2 param dump ${node_name}"
  timeout 10 ros2 param dump "${node_name}" >> "${LOG}" 2>&1 || true
}

read_twist_once() {
  local topic="$1"
  timeout 0.6 ros2 topic echo --once "${topic}" 2>/dev/null | awk '
    /linear:/ {lin=1; next}
    lin && /^[[:space:]]+x:/ {vx=$2; lin=0}
    /angular:/ {ang=1; next}
    ang && /^[[:space:]]+z:/ {wz=$2; ang=0}
    END {
      if (vx == "" && wz == "") {
        exit 1
      }
      printf "%s,%s", vx + 0, wz + 0
    }
  '
}

read_string_once() {
  local topic="$1"
  timeout 0.6 ros2 topic echo --once "${topic}" 2>/dev/null | awk '
    /data:/ {
      sub(/^data: /, "")
      gsub(/"/, "")
      print
      exit
    }
  '
}

snapshot_environment() {
  append_section_header "record context"
  {
    date --iso-8601=seconds
    hostname
    pwd
    echo "duration_sec=${DURATION}"
    echo "out_dir=${OUT_DIR}"
    echo "topics=${TOPICS[*]}"
    echo
  } >> "${LOG}" 2>&1

  append_cmd "ros2 node list" timeout 10 ros2 node list
  append_cmd "ros2 topic list -t" timeout 10 ros2 topic list -t
  append_cmd "ros2 service list" timeout 10 ros2 service list
  append_cmd "ros2 action list -t" timeout 10 ros2 action list -t
  append_process_snapshot "candidate process snapshot (before)"

  dump_param /controller_server
  dump_param /planner_server
  dump_param /bt_navigator
  dump_param /behavior_server
  dump_param /velocity_smoother
  dump_param /local_costmap/local_costmap
  dump_param /global_costmap/global_costmap
  dump_param /navigation_manager

  timeout 15 ros2 run g1_base diag_bt_navigator > "${DIAG_BT_LOG}" 2>&1 || true
}

start_background_diag() {
  ros2 run g1_base diag_obstacle > "${DIAG_OBSTACLE_LOG}" 2>&1 &
  PIDS_TO_CLEAN+=("$!")

  ros2 run g1_base diag_costmap_decay > "${DIAG_COSTMAP_LOG}" 2>&1 &
  PIDS_TO_CLEAN+=("$!")
}

cleanup_pid() {
  local pid="$1"
  if [[ -n "${pid}" ]]; then
    kill -INT "${pid}" 2>/dev/null || true
    wait "${pid}" 2>/dev/null || true
  fi
}

cleanup() {
  trap - EXIT INT TERM
  log_note "stopping background recorders"

  cleanup_pid "${BAG_PID:-}"
  for pid in "${PIDS_TO_CLEAN[@]}"; do
    cleanup_pid "${pid}"
  done

  append_process_snapshot "candidate process snapshot (after)"
  log_note "done -> ${OUT_DIR}"
}
trap cleanup EXIT INT TERM

log_note "out=${OUT_DIR} duration=${DURATION}s"
snapshot_environment
start_background_diag

ros2 bag record -o "${BAG_DIR}" "${TOPICS[@]}" >> "${BAG_LOG}" 2>&1 &
BAG_PID=$!

END=$(( $(date +%s) + DURATION ))
{
  echo "ts,planner_vx,planner_wz,executed_vx,executed_wz,motion_source,nav_state,bt_status_tail"
  while [ "$(date +%s)" -lt "${END}" ]; do
    NOW=$(date +%s.%N)
    PLANNER_CMD=$(read_twist_once /cmd_vel || true)
    EXEC_CMD=$(read_twist_once /cmd_vel_executed || true)
    SOURCE=$(read_string_once /motion_source || true)
    NAV_STATE=$(read_string_once /navigation_manager/state || true)
    BT=$(timeout 0.5 ros2 topic echo --once /behavior_tree_log 2>/dev/null \
      | tr '\n' ' ' | tail -c 200 | tr '"' "'")
    echo "${NOW},${PLANNER_CMD:-,},${EXEC_CMD:-,},${SOURCE:-unknown},${NAV_STATE:-unknown},\"${BT}\""
    sleep 0.5
  done
} >> "${LIVE_CSV}" 2>&1

log_note "sampler finished"
