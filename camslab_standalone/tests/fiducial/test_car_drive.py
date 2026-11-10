"""camslab.car_drive, the command relay that runs on the car, without ROS: its arguments
and what it gives the driver as the commands age."""

import math

import pytest

from camslab import car, car_drive

SETTINGS = {"name": "angostura", "camera_host": "jetson@192.168.1.85",
            "container": "yahboom_robot", "ros_distro": "humble",
            "driver_cmd_vel": "/cmd_vel", "command_timeout": 0.5,
            "controller": {"kappa_max": 2.9, "v_max": 0.4, "w_max": 2.0}}


def test_the_relay_parses_without_ros():
    """rclpy is imported inside run(): the module and its parser do not need it."""
    args = car_drive.parse_args(["--name", "a", "--ros-args", "-r", "__node:=drive"])
    assert (args.name, args.out, args.timeout) == ("a", "/cmd_vel", 0.5)
    with pytest.raises(SystemExit):
        car_drive.parse_args([])


def test_arguments_carry_the_settings():
    args = car_drive.parse_args(car.drive_arguments(SETTINGS))
    assert (args.name, args.out, args.timeout, args.v_max, args.w_max) == ("angostura", "/cmd_vel", 0.5, 0.4, 2.0)
    # without a controller block: the limits of params.yaml
    args = car_drive.parse_args(car.drive_arguments({"name": "bodies"}))
    assert (args.out, args.timeout, args.v_max, args.w_max) == ("/cmd_vel", 0.5, 0.6, 2.0)


def test_the_ssh_command_stops_an_earlier_relay_and_runs_in_the_container(monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", "32")
    command = car.drive_command(SETTINGS)
    assert command.startswith("pkill -INT -f '^python3 /tmp/camslab_car_drive.py' && sleep 1; exec "
                              "python3 /tmp/camslab_car_drive.py --name angostura --out /cmd_vel ")
    ssh = car.drive_ssh_command(SETTINGS, batch=True)
    assert ssh[0] == "ssh" and "-tt" in ssh and "BatchMode=yes" in ssh
    assert ssh[-2] == "jetson@192.168.1.85"
    assert "docker exec -it yahboom_robot" in ssh[-1] and "ROS_DOMAIN_ID=32" in ssh[-1]
    # the camera node's pattern does not match the relay, and the other way round
    assert car.REMOTE_NODE != car.REMOTE_DRIVE


def test_a_command_is_passed_on_while_it_is_fresh():
    relay = car_drive.Relay(timeout=0.5, v_max=0.4, w_max=2.0)
    assert relay.output(0.0) is None            # nothing before the first command
    relay.receive(0.2, -0.5, now=10.0)
    assert relay.output(10.0) == (0.2, -0.5) and relay.output(10.5) == (0.2, -0.5)
    assert relay.driving(10.5) and not relay.driving(10.51)


def test_a_stale_command_becomes_zero_and_then_nothing():
    """The network fails mid-drive: zero from the timeout on, for ZERO_SECONDS, and
    nothing after that, so the car's teleop is not overridden."""
    relay = car_drive.Relay(timeout=0.5, v_max=0.4, w_max=2.0)
    relay.receive(0.3, 1.0, now=10.0)
    assert relay.output(10.51) == (0.0, 0.0)
    assert relay.output(10.5 + car_drive.ZERO_SECONDS) == (0.0, 0.0)
    assert relay.output(10.51 + car_drive.ZERO_SECONDS) is None
    relay.receive(0.1, 0.0, now=20.0)           # a new command drives again
    assert relay.output(20.1) == (0.1, 0.0)


def test_commands_are_limited_and_must_be_finite():
    relay = car_drive.Relay(timeout=0.5, v_max=0.4, w_max=2.0)
    relay.receive(0.9, -5.0, now=1.0)
    assert relay.output(1.0) == (0.4, -2.0)
    relay.receive(-0.9, 5.0, now=1.0)
    assert relay.output(1.0) == (-0.4, 2.0)
    for v, w in ((math.nan, 0.0), (0.1, math.inf), (-math.inf, math.nan)):
        relay.receive(0.3, 0.3, now=2.0)
        relay.receive(v, w, now=2.1)
        assert relay.output(2.1) == (0.0, 0.0)
