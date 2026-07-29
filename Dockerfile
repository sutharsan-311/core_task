# Omokai — reproducible ROS 2 Humble + Gazebo Classic + Nav2 image.
#
# Base already carries ROS 2 Humble, Gazebo Classic 11, and RViz2 (desktop-full).
# We add Nav2, SLAM Toolbox, TurtleBot3, the vision + Bedrock Python deps, then
# build the workspace. GUI (Gazebo/RViz) and AWS creds are passed at run time —
# see docker-run.sh.
FROM osrf/ros:humble-desktop-full

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV TURTLEBOT3_MODEL=waffle_pi

# ROS packages the workspace needs that aren't in desktop-full, plus pip.
# Note: diagnostic-updater is explicitly included to avoid ABI mismatches between
# Nav2 lifecycle manager and the diagnostic_updater library it links against.
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip \
      ros-humble-diagnostic-updater \
      ros-humble-navigation2 \
      ros-humble-nav2-bringup \
      ros-humble-slam-toolbox \
      ros-humble-turtlebot3-gazebo \
      ros-humble-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /omokai_ws

# Only the manifest first, so a source-code-only edit below doesn't bust this
# layer and re-trigger the ~2GB of downloads it pulls in.
COPY core_task_perception/requirements.txt core_task_perception/requirements.txt

# CPU-only torch: ultralytics' default pulls the CUDA build (cudnn, cublas,
# nccl, etc. - several GB this CPU-inference, no-GPU container never uses).
# Install the CPU wheel first so ultralytics' resolver finds it already
# satisfied. boto3/anthropic/openai cover the three LLM providers.
RUN pip3 install --no-cache-dir torch --index-url https://download.pytorch.org/whl/cpu \
    && pip3 install --no-cache-dir boto3 anthropic openai \
      -r core_task_perception/requirements.txt

# torch (an ultralytics dependency) drags setuptools up to a version colcon-core
# rejects (colcon-core requires <80; ultralytics/torch installs 83.0.0), which
# crashes ament_python's egg_info step with a canonicalize_version() TypeError.
# Pinning setuptools alone isn't enough: setuptools 78.x imports
# canonicalize_version from the *external* packaging package, but the base
# image's system packaging (21.3, apt-installed) predates that function, and
# `packaging<26` alone is satisfied by that already-installed 21.3 (pip skips
# the upgrade) - packaging must be forced above it explicitly. packaging>=26
# removed the function's strip_trailing_zero kwarg again, so the window is
# narrow. Verified against a real `colcon build --packages-select
# core_task_perception` before landing this pin.
RUN pip3 install --no-cache-dir "setuptools<80" "packaging>=22,<26"

# Pre-download the YOLO weights into ultralytics' fixed weights_dir cache
# (~/weights - checked regardless of a node's runtime cwd, unlike a plain
# relative path) so the container works fully offline after this build step.
RUN mkdir -p /root/weights && cd /root/weights \
    && python3 -c "from ultralytics import YOLO; YOLO('yolo26n.pt')"

# Now the rest of the workspace source, and the actual build. Everything
# above this line only re-runs when requirements.txt (or this Dockerfile)
# changes, not on ordinary source edits.
COPY . /omokai_ws

# Resolve any remaining rosdep keys from the package.xml files, then build.
RUN apt-get update \
    && rosdep update \
    && rosdep install --from-paths . --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/* \
    && source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install

# Auto-source ROS + the workspace in every shell.
RUN echo "source /opt/ros/humble/setup.bash"    >> /root/.bashrc \
    && echo "source /omokai_ws/install/setup.bash" >> /root/.bashrc

# Interactive shell by default; docker-run.sh launches ./run.sh.
CMD ["bash"]
