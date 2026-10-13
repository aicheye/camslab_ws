# Install

Requires Ubuntu 22.04 and ROS 2 Humble.

Choose one setup below. Use the host install for development. Use Docker to keep the ROS
and Gazebo dependencies inside a container.

## Host install

1. Install ROS 2 Humble Desktop using the [ROS 2 install guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).
2. Install the build tools:

   ```bash
   sudo apt install python3-colcon-common-extensions python3-rosdep
   sudo rosdep init
   rosdep update
   ```

   Ignore the `rosdep init` error when it reports that rosdep is already initialized.
3. Clone the repository and install its dependencies:

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

After pulling changes, repeat step 4.

```{tip}
Add `source /opt/ros/humble/setup.bash` to `~/.bashrc` to source ROS automatically in new terminals.
```

Verify the build before launching:

```bash
source install/setup.bash
ros2 pkg prefix camslab
```

Confirm that the command prints a path containing `install/camslab`. If it does not,
rebuild from the repository root and source `install/setup.bash` again.

## Docker

Install Docker and grant the current user permission to run it. From the repository root:

```bash
docker compose -f docker/compose.yaml build
docker compose -f docker/compose.yaml run --rm ros
```

Inside the container, build and source the workspace once:

```bash
colcon build --symlink-install
source install/setup.bash
```

Keep that shell open while running the simulator ({doc}`running`). Repeat the build after
changing a `package.xml`, `CMakeLists.txt`, or Python package dependency. Rebuild the
image after changing the Dockerfile or adding a system dependency.
