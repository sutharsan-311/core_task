#!/usr/bin/env bash
# One-terminal launcher for the Omokai stack.
#
# Brings up the full stack (Gazebo + Nav2 + controller + LLM backend) in the
# background and hands THIS terminal to the natural-language mission prompt
# (nlm_cli). Type missions here; exit the prompt (/quit or Ctrl-C) and the whole
# stack is torn down. No second terminal.
#
#   ./run.sh                # single robot
#   ./run.sh squad:=true    # add robot2 + the squad coordinator
#   ./run.sh nlm:=false     # skip the LLM backend (then use ros2 topic pub)
#
# Needs AWS credentials + AWS_REGION for the Bedrock LLM front-end. Stack logs go
# to src/core_task/logs/ (path printed at startup) so they don't corrupt the prompt.
set -o pipefail

PKG="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$PKG/../.." && pwd)"
# In the Docker image the workspace is flattened (COPY . /omokai_ws puts
# run.sh straight at the ws root), so the host's <ws>/src/core_task depth
# assumption doesn't hold there. Fall back to PKG itself in that case.
[ ! -f "$WS/install/setup.bash" ] && [ -f "$PKG/install/setup.bash" ] && WS="$PKG"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$WS/install/setup.bash"

if [ -z "${AWS_REGION:-}" ] && [ -z "${AWS_DEFAULT_REGION:-}" ]; then
  echo "WARNING: AWS_REGION is not set - the Bedrock LLM front-end may not start."
  echo "         export AWS_REGION=us-east-1 (and ensure AWS credentials are"
  echo "         configured), or pass nlm:=false and drive with"
  echo "         'ros2 topic pub /submit_mission ...'."
fi

LOG_DIR="$PKG/logs"
mkdir -p "$LOG_DIR"
# Sequential run number (1, 2, 3, ...): first unused omokai-bringup.N.log.
N=1
while [ -e "$LOG_DIR/omokai-bringup.$N.log" ]; do N=$((N + 1)); done
LOG="$LOG_DIR/omokai-bringup.$N.log"
echo "Starting stack (logs -> $LOG). Give it a few seconds ..."
ros2 launch core_task_controller bringup.launch.py "$@" >"$LOG" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  echo
  echo "Shutting down the stack ..."
  kill -INT "$LAUNCH_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true
  # gzserver occasionally outlives ros2 launch's shutdown; make sure it's gone.
  pkill -f gzserver 2>/dev/null || true
}
trap cleanup EXIT

# Let Gazebo + Nav2 + the controller come up before the hand-off. nlm_cli also
# self-reports "nlm NOT RUNNING" in its status bar until the backend appears, so
# an early start is harmless - this sleep just avoids a confusing first second.
sleep 8
ros2 run core_task_controller nlm_cli
rc=$?

# nlm_cli exits 42 when the operator typed `kill`: do the aggressive teardown
# (kill.sh also resets the ROS 2 daemon). Run it from here - the process the
# shell is actually waiting on - and via exec, so kill.sh replaces run.sh and
# the terminal comes back cleanly when it finishes. The EXIT trap is dropped so
# the light cleanup above doesn't also fire.
if [ "$rc" = "42" ]; then
  trap - EXIT
  exec "$PKG/kill.sh"
fi
