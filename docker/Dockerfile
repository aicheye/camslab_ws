# ROS 2 Humble + Gazebo Fortress with the dependencies of every package in the repo, plus
# the lab hardware drivers: vrpn_mocap (motion capture) and robomaster_ros (DJI RoboMaster).
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

# robomaster_ros is not released as a Debian package. It runs on jeguzzi's fork of the
# RoboMaster SDK and is built from source into the underlay /opt/robomaster_ws.
# The SDK imports libmedia_codec (camera video and audio decoding) in robomaster.robot,
# so the driver needs it even with the camera off. It has no wheel for Python 3.10 and
# its CMake build expects vcpkg, so it is built against Ubuntu's ffmpeg and opus, with an
# OpusConfig.cmake that finds opus through pkg-config (libopus-dev has none).
# pip is upgraded first because the Ubuntu 22.04 pip cannot run its build backend.
# numpy stays below 2 for the Humble packages built against numpy 1 (cv_bridge).
RUN apt-get update \
    && apt-get install -y --no-install-recommends git python3-pip cmake g++ pkg-config \
        libavcodec-dev libavutil-dev libswscale-dev libopus-dev \
    && rm -rf /var/lib/apt/lists/* \
    && mkdir -p /opt/opus-cmake \
    && printf '%s\n' 'find_package(PkgConfig REQUIRED)' \
        'pkg_check_modules(OPUS REQUIRED IMPORTED_TARGET GLOBAL opus)' \
        'add_library(Opus::opus ALIAS PkgConfig::OPUS)' > /opt/opus-cmake/OpusConfig.cmake \
    && python3 -m pip install --no-cache-dir -U pip \
    && python3 -m pip install --no-cache-dir -Ccmake.args="" -Ccmake.define.Opus_DIR=/opt/opus-cmake \
        "rm_libmedia_codec @ git+https://github.com/jeguzzi/RoboMaster-SDK.git#subdirectory=lib/libmedia_codec" \
    && python3 -m pip install --no-cache-dir "numpy>=1.25,<2" numpy-quaternion \
        git+https://github.com/jeguzzi/RoboMaster-SDK.git \
    && python3 -c "import libmedia_codec, robomaster.robot"
RUN git clone --depth 1 https://github.com/jeguzzi/robomaster_ros.git \
        /opt/robomaster_ws/src/robomaster_ros \
    && apt-get update \
    && rosdep install --from-paths /opt/robomaster_ws/src --ignore-src -y \
    && rm -rf /var/lib/apt/lists/*
RUN . /opt/ros/humble/setup.sh \
    && cd /opt/robomaster_ws \
    && colcon build --event-handlers console_direct- \
    && rm -rf build log

ENV PYTHONDONTWRITEBYTECODE=1
# Interactive shells source the RoboMaster underlay, then the workspace overlay when it
# has been built.
RUN echo 'source /opt/robomaster_ws/install/setup.bash' >> /root/.bashrc \
    && echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> /root/.bashrc
WORKDIR /ws
