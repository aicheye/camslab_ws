# Your own controller

## Change the control law

Edit `control()` in `camslab/src/controller.cpp`, run `colcon build --symlink-install`, and restart the simulator. The function gets the car's pose and the reference ($p^*$, $q^*$, $v^*$, $\omega^*$) and returns $v$ and $\omega$. The web UI, the command limits and the safe area keep working.

## Drive the car from another program

For a controller in Python, MATLAB's ROS Toolbox or anything else that speaks ROS 2:

1. Press **Stop** in the web UI.
2. Read the pose from `/angostura/pose`.
3. Publish `geometry_msgs/Twist` on `/angostura/cmd_vel`, with $v$ in `linear.x` and $\omega$ in `angular.z`.

The car holds the last command, so publish at a steady rate and send zero before exiting. Your program gets no limits and no safe-area checks.
