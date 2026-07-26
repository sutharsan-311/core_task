#!/usr/bin/env bash
# Kill the whole Omokai stack and reset ROS 2 discovery.
#
# Stops Gazebo and every ROS 2 process (Nav2, the controllers, the LLM
# front-end, RViz, SLAM - anything bringup / run.sh started), then bounces the
# ROS 2 daemon so no stale discovery state leaks into the next run.
#
# Broad on purpose: it kills ALL ros2 processes on this machine (it matches the
# ROS install and this workspace's install dirs), not just ours. Run it when you
# want a clean slate.
#
#   ./kill.sh
set -o pipefail

PKG="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$PKG/../.." && pwd)"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash 2>/dev/null

# Process-cmdline patterns to match. The two paths sweep up every ROS node
# (the ROS install + this workspace's install); gazebo/rviz are separate
# binaries so they get their own patterns.
PATTERNS=(gzserver gzclient gazebo rviz2 'ros2 launch' 'ros2 run'
          "$WS/install" '/opt/ros/')

echo "Stopping Gazebo + all ROS 2 nodes ..."
for p in "${PATTERNS[@]}"; do pkill -INT  -f "$p" 2>/dev/null; done
sleep 2
# Anything that ignored SIGINT gets SIGKILL.
for p in "${PATTERNS[@]}"; do pkill -KILL -f "$p" 2>/dev/null; done

echo "Bouncing the ROS 2 daemon ..."
ros2 daemon stop  >/dev/null 2>&1
ros2 daemon start >/dev/null 2>&1

echo "Done - stack down, daemon fresh."
