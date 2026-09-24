FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-aiohttp \
      ros-humble-ros-gz \
      ros-humble-robot-state-publisher \
      ros-humble-joint-state-publisher \
      ros-humble-xacro \
      ros-humble-rviz2 \
      ros-humble-rqt-graph \
    && rm -rf /var/lib/apt/lists/*

ENV PYTHONDONTWRITEBYTECODE=1
# Interactive shells source the workspace overlay when it has been built.
RUN echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> /root/.bashrc
WORKDIR /ws
