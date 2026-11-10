"""The command relay of the fiducial survey. It runs on the car, next to the car's own
driver: `ros2 run camslab car drive` and survey.launch.py upload this file and start it
over ssh. The controller on the other computer publishes /<name>/cmd_vel over the
network; this node passes the newest command on to the driver's topic (--out) at a fixed
rate, and the driver hears nothing else.

It exists for the case the controller cannot handle: the network or the other computer
fails while the car drives. The controller's zero command then never arrives, and a
driver without a timeout keeps the last command. Here a command older than --timeout
seconds is replaced by zero, on the car, whatever happens to the network. The age is
measured from the arrival on the car's own monotonic clock, so it does not depend on
the two clocks agreeing.

- A command is limited to |v| <= --v-max and |w| <= --w-max; one that is not finite
  counts as zero.
- After the timeout the node publishes zero for ZERO_SECONDS and then nothing, so the
  car's own teleop works while no controller is driving.
- It publishes zero when it stops.

Self-contained like car_node.py: it needs rclpy and geometry_msgs and nothing of camslab.

    python3 car_drive.py --name angostura --out /cmd_vel --timeout 0.5 --v-max 0.3 --w-max 2
"""

import argparse
import math
import signal
import sys
import time

ZERO_SECONDS = 1.0          # zero is published this long after the last command expired
REPORT_PERIOD = 5.0


def parse_args(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if "--ros-args" in argv:                    # from a launch file's Node(arguments=...)
        argv = argv[:argv.index("--ros-args")]
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--name", required=True, help="robot name: commands arrive on /<name>/cmd_vel")
    p.add_argument("--out", default="/cmd_vel", help="the topic the car's driver listens on")
    p.add_argument("--timeout", type=float, default=0.5,
                   help="a command older than this is replaced by zero [s]")
    p.add_argument("--rate", type=float, default=20.0, help="commands per second to the driver")
    p.add_argument("--v-max", type=float, default=0.6, help="speed limit [m/s]")
    p.add_argument("--w-max", type=float, default=2.0, help="turning rate limit [rad/s]")
    return p.parse_args(argv)


class Relay:
    """The newest command and when it arrived. output() is what the driver gets."""

    def __init__(self, timeout, v_max, w_max):
        self.timeout, self.v_max, self.w_max = float(timeout), abs(float(v_max)), abs(float(w_max))
        self.v = self.w = 0.0
        self.time = None

    def receive(self, v, w, now):
        """A command (v [m/s], w [rad/s]) that arrived at monotonic time `now`."""
        if not (math.isfinite(v) and math.isfinite(w)):
            v = w = 0.0
        self.v = min(max(float(v), -self.v_max), self.v_max)
        self.w = min(max(float(w), -self.w_max), self.w_max)
        self.time = now

    def output(self, now):
        """(v, w) for the driver at time `now`: the newest command while it is younger
        than the timeout, zero for ZERO_SECONDS after that, then None for nothing."""
        if self.time is None:
            return None
        age = now - self.time
        if age <= self.timeout:
            return self.v, self.w
        if age <= self.timeout + ZERO_SECONDS:
            return 0.0, 0.0
        return None

    def driving(self, now):
        return self.time is not None and now - self.time <= self.timeout


def interrupt(signum, frame):
    raise KeyboardInterrupt


def run(args):
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node

    # This node's own handlers, so that it can still publish zero when it is stopped.
    try:
        from rclpy.signals import SignalHandlerOptions

        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init()
    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, interrupt)

    node = Node("drive", namespace=f"/{args.name}")
    log = node.get_logger()
    relay = Relay(args.timeout, args.v_max, args.w_max)
    publisher = node.create_publisher(Twist, args.out, 10)
    source = node.resolve_topic_name("cmd_vel")
    if publisher.topic_name == source:
        log.error(f"--out is {source}, the topic the commands arrive on; it must be the "
                  "driver's topic (driver_cmd_vel in survey.yaml)")
        return 2
    state = {"received": 0, "driving": False, "report": time.monotonic()}

    def on_command(msg):
        relay.receive(msg.linear.x, msg.angular.z, time.monotonic())
        state["received"] += 1

    def publish(v, w):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        publisher.publish(msg)

    def on_timer():
        now = time.monotonic()
        out = relay.output(now)
        if out is not None:
            publish(*out)
        driving = relay.driving(now)
        if state["driving"] and not driving:
            log.warning(f"no command for {args.timeout:g} s: zero to {publisher.topic_name}")
        elif driving and not state["driving"]:
            log.info("commands arrive: driving")
        state["driving"] = driving
        if now - state["report"] >= REPORT_PERIOD:
            rate = state["received"] / (now - state["report"])
            line = f"{rate:.0f} commands/s, " + (f"v {relay.v:.2f} m/s, w {relay.w:.2f} rad/s"
                                                 if driving else "not driving")
            if publisher.get_subscription_count() == 0:
                log.warning(f"{line}; nothing listens on {publisher.topic_name}: is the car's "
                            "driver running, and is driver_cmd_vel its topic?")
            else:
                log.info(line)
            state["received"], state["report"] = 0, now

    node.create_subscription(Twist, "cmd_vel", on_command, 10)
    node.create_timer(1.0 / args.rate, on_timer)
    log.info(f"{source} to {publisher.topic_name} at {args.rate:g} Hz, zero after "
             f"{args.timeout:g} s without a command, |v| <= {relay.v_max:g} m/s, "
             f"|w| <= {relay.w_max:g} rad/s")
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            for _ in range(5):
                publish(0.0, 0.0)
                time.sleep(0.02)
        except Exception:               # the context is already down: nothing left to send
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def main(argv=None):
    return run(parse_args(argv))


if __name__ == "__main__":
    sys.exit(main())
