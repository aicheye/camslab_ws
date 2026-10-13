# Write a custom controller node

Write a custom ROS 2 node when the controller needs another language, additional sensor
data, or a different reference generator. Disable the built-in controller first.

## Topics

| Topic | Type | Direction | Contents |
|---|---|---|---|
| `/angostura/pose` | `geometry_msgs/PoseStamped` | subscribe | position in `pose.position.x`, `.y`; heading quaternion in `pose.orientation`, $\theta = 2\,\mathrm{atan2}(z, w)$ |
| `/angostura/cmd_vel` | `geometry_msgs/Twist` | publish | $v$ in `linear.x`, $\omega$ in `angular.z` |

The pose is published at 100 Hz. Compute the reference inside the custom node.

## Run the node

1. Start the simulator ({doc}`running`).
2. Press **Stop** in the web UI. The built-in controller publishes zero for 1 s, then
   stops publishing.
3. Run the node in another terminal with the Gazebo clock enabled:

   ```bash
   source ~/camslab_ws/install/setup.bash
   python3 my_controller.py --ros-args -p use_sim_time:=true
   ```

4. Stop the node with `Ctrl + C`. Publish a zero command on exit.

## Python example

Run the example controller below. It tracks a circle with a 1 m radius:

```{literalinclude} examples/my_controller.py
:language: python
```

## MATLAB example

Use the ROS Toolbox:

```matlab
node = ros2node("/my_controller");
poseSub = ros2subscriber(node, "/angostura/pose", "geometry_msgs/PoseStamped");
cmdPub = ros2publisher(node, "/angostura/cmd_vel", "geometry_msgs/Twist");
cmd = ros2message(cmdPub);

rate = ros2rate(node, 50);
for k = 1:3000                      % 60 s at 50 Hz
    msg = poseSub.LatestMessage;
    if isempty(msg), waitfor(rate); continue, end
    p = [msg.pose.position.x; msg.pose.position.y];
    theta = 2 * atan2(msg.pose.orientation.z, msg.pose.orientation.w);
    % ... compute v and w from p and theta ...
    cmd.linear.x = v;
    cmd.angular.z = w;
    send(cmdPub, cmd);
    waitfor(rate);
end
cmd.linear.x = 0; cmd.angular.z = 0; send(cmdPub, cmd);
```

## Safety

The custom node does not inherit the built-in controller's command saturation or safe
area checks. Before running on a real car:

- Saturate $|v| \le 0.6$ m/s and $|\omega| \le \min(2, 2.9 |v|)$ rad/s.
- Publish zero when the pose is older than 0.5 s.
- Publish zero before exiting.
- Publish at a fixed rate. The built-in controller uses 50 Hz.

The car keeps executing its last command until it receives a new command or the driver
stops it.

## Compare controllers

Use the same path, initial pose, and duration for every controller. Press **Reset** and
**Start**, download each CSV, and compare the `dist` column.
