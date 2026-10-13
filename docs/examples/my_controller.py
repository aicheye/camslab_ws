"""Track a circle with the camslab control law, as a standalone ROS 2 node.

Run with the built-in controller stopped:
    python3 my_controller.py --ros-args -p use_sim_time:=true
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

# Path: circle starting at (X0, Y0) heading THETA0, curvature KAPPA [1/m], speed SPEED [m/s].
X0, Y0, THETA0, KAPPA, SPEED = 2.0, 1.0, 0.0, 1.0, 0.3
# Gains and limits, as in params.yaml.
K_PAR, K_PERP, K_Q = 1.0, 4.0, 2.0
V_MAX, W_MAX, KAPPA_MAX = 0.6, 2.0, 2.9


def reference(t):
    """Return p*, q*, v*, w* of the circle at time t [s]."""
    phi = THETA0 + KAPPA * SPEED * t
    cx = X0 - math.sin(THETA0) / KAPPA  # circle centre, to the left of the start
    cy = Y0 + math.cos(THETA0) / KAPPA
    p_ref = (cx + math.sin(phi) / KAPPA, cy - math.cos(phi) / KAPPA)
    q_ref = (math.cos(phi), math.sin(phi))
    return p_ref, q_ref, SPEED, KAPPA * SPEED


def control(p, q, p_ref, q_ref, v_ref, w_ref):
    """Return (v, w) from the car pose (p, q) and the reference."""
    ex, ey = p_ref[0] - p[0], p_ref[1] - p[1]
    sq = (-q[1], q[0])  # S q
    v = (v_ref * q_ref[0] + K_PAR * ex) * q[0] + (v_ref * q_ref[1] + K_PAR * ey) * q[1]
    w = w_ref + (K_PERP * v_ref * ex + K_Q * q_ref[0]) * sq[0] \
              + (K_PERP * v_ref * ey + K_Q * q_ref[1]) * sq[1]
    return v, w


class MyController(Node):
    def __init__(self):
        super().__init__("my_controller")
        self.pose = None  # (p, q, time of the pose)
        self.t0 = None
        self.cmd_pub = self.create_publisher(Twist, "/angostura/cmd_vel", 10)
        self.create_subscription(PoseStamped, "/angostura/pose", self.on_pose, 10)
        self.create_timer(1.0 / 50.0, self.on_timer)  # 50 Hz

    def on_pose(self, msg):
        o = msg.pose.orientation
        theta = 2.0 * math.atan2(o.z, o.w)
        p = (msg.pose.position.x, msg.pose.position.y)
        self.pose = (p, (math.cos(theta), math.sin(theta)), self.get_clock().now())

    def on_timer(self):
        now = self.get_clock().now()
        cmd = Twist()
        if self.pose and (now - self.pose[2]).nanoseconds < 0.5e9:  # pose is fresh
            if self.t0 is None:
                self.t0 = now
            t = (now - self.t0).nanoseconds * 1e-9
            v, w = control(self.pose[0], self.pose[1], *reference(t))
            v = max(-V_MAX, min(V_MAX, v))
            w_limit = min(W_MAX, KAPPA_MAX * abs(v))
            cmd.linear.x = v
            cmd.angular.z = max(-w_limit, min(w_limit, w))
        self.cmd_pub.publish(cmd)

    def stop(self):
        self.cmd_pub.publish(Twist())


def main():
    # Handle Ctrl + C here instead of in rclpy, so the zero command below can still be sent.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = MyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
