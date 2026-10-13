# Install

camslab needs Ubuntu 22.04 and ROS 2 Humble.

1. Install ROS 2 Humble Desktop by following the [ROS 2 install guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).
2. Install the build tools:

   ```bash
   sudo apt install python3-colcon-common-extensions python3-rosdep
   sudo rosdep init
   rosdep update
   ```

   If `rosdep init` says it is already initialized, carry on.
3. Download the code and its dependencies (Gazebo, RViz and the rest):

   ```bash
   cd ~
   git clone https://github.com/aicheye/camslab_ws.git
   cd camslab_ws
   source /opt/ros/humble/setup.bash
   rosdep install --from-paths . --ignore-src -y
   ```

4. Build:

   ```bash
   colcon build --symlink-install
   ```

Run `git pull` and step 4 again whenever the code changes.

```{tip}
Add `source /opt/ros/humble/setup.bash` to the end of `~/.bashrc` so every new terminal has ROS.
```
