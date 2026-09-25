# ROS 2 Humble + Gazebo Fortress with the dependencies of every package in the repo.
# compose.yaml builds this with the repo root as the context.
FROM ros:humble-ros-base

# rosdep reads the package.xml files, the same way a bare-metal install does (README.md).
COPY camslab/package.xml /tmp/deps/camslab/package.xml
COPY camslab_webui/package.xml /tmp/deps/camslab_webui/package.xml
RUN apt-get update \
    && rosdep update --rosdistro humble \
    && rosdep install --from-paths /tmp/deps --ignore-src -y \
    && apt-get install -y --no-install-recommends ros-humble-rqt-graph \
    && rm -rf /var/lib/apt/lists/* /tmp/deps

ENV PYTHONDONTWRITEBYTECODE=1
# Interactive shells source the workspace overlay when it has been built.
RUN echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> /root/.bashrc
WORKDIR /ws
