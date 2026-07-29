#!/usr/bin/env bash
# Bare-metal setup for the Omokai workspace (alternative to Docker).
#
# Assumes Ubuntu 22.04 with ROS 2 Humble already installed and sourced.
# Installs the workspace's ROS + Python deps and builds it. Re-runnable.
#
#   ./install.sh
#   ./run.sh
set -euo pipefail

PKG="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$PKG/../.." && pwd)"
cd "$WS"

if [ -z "${ROS_DISTRO:-}" ]; then
  echo "ERROR: source ROS 2 Humble first:  source /opt/ros/humble/setup.bash" >&2
  exit 1
fi

echo ">> ROS package dependencies (rosdep)"
sudo apt-get update
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y
# Not a rosdep key of any package.xml here - it's pulled in explicitly to avoid
# an ABI mismatch between Nav2's lifecycle manager and diagnostic_updater
# (see Dockerfile).
sudo apt-get install -y ros-humble-diagnostic-updater

echo ">> Python dependencies (Bedrock/Claude/OpenAI + vision)"
python3 -m pip install boto3 anthropic openai \
  -r src/core_task/core_task_perception/requirements.txt

# ultralytics/torch pulls setuptools up to a version colcon-core rejects, and
# separately the system `packaging` package (apt, Ubuntu 22.04) predates a
# function setuptools' egg_info step needs - either one alone crashes
# colcon build with a canonicalize_version() TypeError. See Dockerfile for
# the same fix and how it was diagnosed.
python3 -m pip install "setuptools<80" "packaging>=22,<26"

echo ">> Build workspace"
if ! colcon build --symlink-install; then
  echo "ERROR: colcon build failed" >&2
  exit 1
fi

echo ">> Sourcing workspace"
# shellcheck disable=SC1091
source "$WS/install/setup.bash"

cat <<'DONE'

✅ Installation complete. To run:
  export AWS_REGION=us-east-1              # your Bedrock region
  export AWS_ACCESS_KEY_ID=...             # your AWS credentials (or configure ~/.aws)
  export AWS_SECRET_ACCESS_KEY=...
  ./run.sh                                 # two robots + squad (default)
  ./run.sh squad:=false                    # single robot (lighter)
DONE
