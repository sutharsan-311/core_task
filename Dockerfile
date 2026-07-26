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
COPY . /omokai_ws

# Python deps: boto3 for Bedrock, and the vision pins (numpy<2 + ultralytics)
# from the perception package's requirements.txt.
RUN pip3 install --no-cache-dir boto3 \
      -r src/core_task/core_task_perception/requirements.txt

# Resolve any remaining rosdep keys from the package.xml files, then build.
RUN apt-get update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/* \
    && source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install

# Auto-source ROS + the workspace in every shell.
RUN echo "source /opt/ros/humble/setup.bash"    >> /root/.bashrc \
    && echo "source /omokai_ws/install/setup.bash" >> /root/.bashrc

# Interactive shell by default; docker-run.sh launches ./run.sh.
CMD ["bash"]
