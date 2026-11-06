#!/usr/bin/env bash
#
# run_sim_test.sh
#
# End to end test of the vision pick and place pipeline in Gazebo.
# Sources the ROS2 and workspace overlays, builds the workspace, launches
# sim.launch.py, waits for move_group and the controllers to actually be
# ready (polling, not fixed sleeps), launches pick_place.launch.py, waits
# up to MAX_WAIT_SECONDS for the sequence to finish, checks success from
# both the node log and the final Gazebo pose of the cylinder relative to
# the cube, and always tears down every process this script started, even
# on failure or if interrupted.
#
# Success condition: the pick_place_node log contains "Pick-and-place
# complete" AND the final pose of blue_cylinder (read with
# "ign model -m blue_cylinder -p") is within POSE_TOLERANCE metres of
# red_box in x and y, and above red_box's top surface.
#
# Prints a single PASS or FAIL as the last line of stdout. Everything else
# goes to test_logs/<timestamp>/.
#
# Deliberately does NOT use "set -u": ROS2's own setup.bash (both the
# Humble install and the workspace install/setup.bash overlay) references
# variables such as AMENT_TRACE_SETUP_FILES that are legitimately unset on
# first source, and nounset mode turns that into a fatal error before the
# script even reaches the build step. Unset-variable bugs in this script's
# own logic are caught by code review instead.

WS_DIR="/home/aakash/ros2_ws"
REPO_DIR="/home/aakash/ros2_ws/src/ros2_ur5_vision_grasp"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${REPO_DIR}/test_logs/${TIMESTAMP}"
mkdir -p "${LOG_DIR}"

SIM_LOG="${LOG_DIR}/sim_launch.log"
PP_LOG="${LOG_DIR}/pick_place_launch.log"
BUILD_LOG="${LOG_DIR}/build.log"
SUMMARY_LOG="${LOG_DIR}/summary.log"

READY_TIMEOUT_MOVE_ACTION=180   # seconds to wait for /move_action to exist
READY_TIMEOUT_JOINT_STATES=60   # seconds to wait for /joint_states to be published
READY_POLL_INTERVAL=5

MAX_WAIT_SECONDS=420             # cap on waiting for the pick and place sequence
RESULT_POLL_INTERVAL=10

POSE_TOLERANCE=0.03
CUBE_HALF_HEIGHT=0.025           # red_box: 0.05 m cube, see pick_place.sdf

SIM_PID=""
PP_PID=""

log() {
  echo "[$(date +%H:%M:%S)] $*" | tee -a "${SUMMARY_LOG}"
}

cleanup() {
  # Killing by process group (kill -- -PID) is NOT enough here: gz-sim's own
  # launcher (ign gazebo -> "ign gazebo server" / "ign gazebo gui") puts
  # those two processes into their OWN process group, detached from the
  # group this script's setsid created, so a process-group kill of the
  # setsid'd leader never reaches them. Confirmed by hand: three earlier
  # test runs each left an orphaned gz-sim server+gui pair running
  # indefinitely (one for 36+ minutes) after this function reported
  # cleanup done, which went unnoticed until system load hit ~23 on an
  # 8-thread machine. What DOES stay constant is the session ID: setsid
  # makes SIM_PID/PP_PID both the PID and the SID of everything launched
  # under them, and gz-sim's server/gui change their process group but not
  # their session. So kill by session (pkill -s), not by process group.
  log "Cleanup: stopping everything this script started."
  for pid in "${PP_PID}" "${SIM_PID}"; do
    if [ -n "${pid}" ]; then
      log "Sending SIGTERM to session ${pid}"
      pkill -TERM -s "${pid}" 2>/dev/null
      kill -TERM -- "-${pid}" 2>/dev/null   # also try the process group, cheap and harmless
    fi
  done
  sleep 5
  for pid in "${PP_PID}" "${SIM_PID}"; do
    if [ -n "${pid}" ]; then
      log "Sending SIGKILL to session ${pid} (in case anything is still alive)"
      pkill -KILL -s "${pid}" 2>/dev/null
      kill -KILL -- "-${pid}" 2>/dev/null
    fi
  done
  sleep 1
  if [ -n "${SIM_PID}" ] || [ -n "${PP_PID}" ]; then
    local sids="${SIM_PID:-0},${PP_PID:-0}"
    local leftover
    leftover="$(pgrep -s "${sids}" 2>/dev/null)"
    if [ -n "${leftover}" ]; then
      log "WARNING: processes still alive after cleanup, check manually: ${leftover}"
    fi
  fi
  log "Cleanup done."
}
trap cleanup EXIT INT TERM

log "Log directory: ${LOG_DIR}"

# ---------------------------------------------------------------------------
# Environment
# ---------------------------------------------------------------------------
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [ -f "${WS_DIR}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${WS_DIR}/install/setup.bash"
fi

# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------
log "Building workspace (colcon build --symlink-install)..."
(
  cd "${WS_DIR}" || exit 1
  colcon build --symlink-install
) > "${BUILD_LOG}" 2>&1
BUILD_RC=$?
if [ "${BUILD_RC}" -ne 0 ]; then
  log "BUILD FAILED (exit ${BUILD_RC}). See ${BUILD_LOG}"
  echo "FAIL"
  exit 1
fi
log "Build OK."

# shellcheck disable=SC1091
source "${WS_DIR}/install/setup.bash"

# ---------------------------------------------------------------------------
# Readiness polling helper
# ---------------------------------------------------------------------------
wait_for_condition() {
  local description="$1"
  local timeout="$2"
  local check_cmd="$3"
  local waited=0
  log "Waiting for: ${description} (timeout ${timeout}s)"
  while [ "${waited}" -lt "${timeout}" ]; do
    if eval "${check_cmd}" >/dev/null 2>&1; then
      log "Ready: ${description} (after ${waited}s)"
      return 0
    fi
    sleep "${READY_POLL_INTERVAL}"
    waited=$((waited + READY_POLL_INTERVAL))
  done
  log "TIMEOUT waiting for: ${description}"
  return 1
}

# ---------------------------------------------------------------------------
# Launch simulation
# ---------------------------------------------------------------------------
log "Launching sim.launch.py..."
setsid ros2 launch ur5_pick_place_bringup sim.launch.py > "${SIM_LOG}" 2>&1 &
SIM_PID=$!
log "sim.launch.py started, PID/PGID=${SIM_PID}"

wait_for_condition "/move_action action server" "${READY_TIMEOUT_MOVE_ACTION}" \
  "ros2 action list 2>/dev/null | grep -qx /move_action"
MOVE_ACTION_READY=$?

wait_for_condition "/joint_states being published" "${READY_TIMEOUT_JOINT_STATES}" \
  "ros2 topic info /joint_states 2>/dev/null | grep -q 'Publisher count: [1-9]'"
JOINT_STATES_READY=$?

if [ "${MOVE_ACTION_READY}" -ne 0 ] || [ "${JOINT_STATES_READY}" -ne 0 ]; then
  log "Simulation stack did not become ready in time (move_action_ready=${MOVE_ACTION_READY}, joint_states_ready=${JOINT_STATES_READY})."
  echo "FAIL"
  exit 1
fi

# ---------------------------------------------------------------------------
# Launch perception + manipulation
# ---------------------------------------------------------------------------
log "Launching pick_place.launch.py..."
setsid ros2 launch ur5_pick_place_bringup pick_place.launch.py > "${PP_LOG}" 2>&1 &
PP_PID=$!
log "pick_place.launch.py started, PID/PGID=${PP_PID}"

# ---------------------------------------------------------------------------
# Wait for the sequence to finish (or time out)
# ---------------------------------------------------------------------------
waited=0
COMPLETE_SEEN=0
while [ "${waited}" -lt "${MAX_WAIT_SECONDS}" ]; do
  if grep -q "Pick-and-place complete" "${PP_LOG}" 2>/dev/null; then
    COMPLETE_SEEN=1
    log "Saw 'Pick-and-place complete' in pick_place_launch.log after ${waited}s"
    break
  fi
  sleep "${RESULT_POLL_INTERVAL}"
  waited=$((waited + RESULT_POLL_INTERVAL))
done

if [ "${COMPLETE_SEEN}" -eq 0 ]; then
  log "Did not see 'Pick-and-place complete' within ${MAX_WAIT_SECONDS}s."
fi

# Record perception health observations regardless of outcome. The README
# notes NaN depth values can appear even with the GUI, so this must not be
# hidden either way.
if grep -qi "nan" "${PP_LOG}" "${SIM_LOG}" 2>/dev/null; then
  log "NOTE: 'nan' appears in the logs, see ${PP_LOG} / ${SIM_LOG} for context."
fi
if grep -q "Received empty point cloud" "${PP_LOG}" "${SIM_LOG}" 2>/dev/null; then
  log "NOTE: perception reported at least one empty point cloud."
fi

# ---------------------------------------------------------------------------
# Pose check: read ground truth straight from Gazebo with the ign CLI, same
# parsing approach as pick_place_node.py's _get_model_pose, for consistency.
# ---------------------------------------------------------------------------
POSE_CHECK_OUTPUT="$(python3 - "${CUBE_HALF_HEIGHT}" "${POSE_TOLERANCE}" <<'PYEOF'
import subprocess
import sys

cube_half_height = float(sys.argv[1])
tolerance = float(sys.argv[2])


def get_model_pose(name):
    try:
        out = subprocess.run(
            ["ign", "model", "-m", name, "-p"],
            capture_output=True, text=True, timeout=5.0,
        )
    except (subprocess.SubprocessError, OSError) as exc:
        print(f"ERROR running ign for {name}: {exc}", file=sys.stderr)
        return None
    lines = out.stdout.splitlines()
    for i, line in enumerate(lines):
        if "Pose [" in line and i + 1 < len(lines):
            nums = lines[i + 1].strip().strip("[]").split()
            if len(nums) >= 3:
                try:
                    return tuple(float(n) for n in nums[:3])
                except ValueError:
                    return None
    return None


cyl = get_model_pose("blue_cylinder")
box = get_model_pose("red_box")

print(f"blue_cylinder pose: {cyl}")
print(f"red_box pose: {box}")

if cyl is None or box is None:
    print("POSE_RESULT: FAIL (could not read one or both model poses)")
    sys.exit(1)

dx = abs(cyl[0] - box[0])
dy = abs(cyl[1] - box[1])
box_top_z = box[2] + cube_half_height
above_top = cyl[2] > box_top_z

print(
    f"dx={dx:.4f} dy={dy:.4f} tolerance={tolerance} "
    f"box_top_z={box_top_z:.4f} cyl_z={cyl[2]:.4f} above_top={above_top}"
)

if dx <= tolerance and dy <= tolerance and above_top:
    print("POSE_RESULT: PASS")
    sys.exit(0)
else:
    print("POSE_RESULT: FAIL")
    sys.exit(1)
PYEOF
)"
POSE_CHECK_RC=$?
echo "${POSE_CHECK_OUTPUT}" | tee -a "${SUMMARY_LOG}"

# ---------------------------------------------------------------------------
# Verdict
# ---------------------------------------------------------------------------
if [ "${COMPLETE_SEEN}" -eq 1 ] && [ "${POSE_CHECK_RC}" -eq 0 ]; then
  log "RESULT: PASS (log complete=yes, pose check=pass)"
  echo "PASS"
  exit 0
else
  log "RESULT: FAIL (log complete=${COMPLETE_SEEN}, pose check rc=${POSE_CHECK_RC})"
  echo "FAIL"
  exit 1
fi
