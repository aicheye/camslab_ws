"""camslab.car without ROS: the settings, the commands it builds for the car, the rate
counter, the image decoding, and a calibration session fed from rendered board images."""

import io
import math
import os

import numpy as np
import pytest

from camslab import calibration, car

def board_renderer():
    """render_board and the Kalibr layout of test_calibration.py, loaded by path because
    the tests are collected in importlib mode."""
    import importlib.util

    path = os.path.join(os.path.dirname(__file__), "test_calibration.py")
    spec = importlib.util.spec_from_file_location("test_calibration_helpers", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.KALIBR, module.render_board


SETTINGS = {"name": "angostura", "video_device": "/dev/video0", "image_width": 640,
            "image_height": 480, "framerate": 30.0, "pixel_format": "yuyv",
            "camera_host": "jetson@192.168.1.85", "container": "yahboom_robot",
            "ros_distro": "humble"}


def test_settings_overrides_are_parsed_as_yaml(tmp_path):
    path = tmp_path / "survey.yaml"
    path.write_text("name: angostura\nframerate: 30.0\nrviz: true\n")
    settings = car.load_settings(str(path), ["name:=yahboom1", "framerate=15", "rviz:=false"])
    assert settings == {"name": "yahboom1", "framerate": 15, "rviz": False}
    with pytest.raises(SystemExit):
        car.load_settings(str(path), ["port:=1"])


def test_camera_node_command_carries_the_settings_and_the_calibration(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_DOMAIN_ID", "32")
    settings = {**SETTINGS, "dictionary": "DICT_4X4_50", "marker_size": 0.14, "preview_rate": 2.0,
                "marker_ids": [0, 1, 2, 3]}
    command = car.node_command(settings)
    assert "--ids 0 1 2 3 " in command
    assert command.startswith("pkill -INT -f '^python3 /tmp/camslab_car_node.py' && sleep 1; exec "
                              "python3 /tmp/camslab_car_node.py --name angostura --device /dev/video0 ")
    assert "--pixel-format yuyv" in command and "--fx" not in command and "--draw" in command
    cal = tmp_path / "cam.yaml"
    cal.write_text("image_width: 640\nimage_height: 480\ncamera_matrix: {rows: 3, cols: 3, data: "
                   "[500.5, 0, 320, 0, 501, 240, 0, 0, 1]}\ndistortion_coefficients: {rows: 1, "
                   "cols: 5, data: [-0.3, 0.1, 0.001, -0.002, 0.0]}\n")
    args = car.node_arguments(settings, str(cal))
    assert args[args.index("--fx") + 1] == "500.5" and args[args.index("--cy") + 1] == "240.0"
    assert args[args.index("--dist") + 1:args.index("--dist") + 6] == ["-0.3", "0.1", "0.001", "-0.002", "0.0"]
    args = car.node_arguments(settings, "", detect=False, draw=False, preview_rate=30.0, jpeg_quality=95)
    assert "--no-detect" in args and "--draw" not in args
    assert args[args.index("--preview-rate") + 1] == "30" and args[args.index("--jpeg-quality") + 1] == "95"
    ssh = car.camera_ssh_command(settings, batch=True)
    assert ssh[0] == "ssh" and "-tt" in ssh and "BatchMode=yes" in ssh
    assert ssh[-2] == "jetson@192.168.1.85"
    remote = ssh[-1]
    assert remote.startswith("docker exec -it yahboom_robot bash -c ")
    assert "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=32 && pkill" in remote
    plain = car.camera_ssh_command({**settings, "container": ""})
    assert plain[-1].startswith("bash -c ") and "BatchMode" not in plain
    check = car.remote_shell(settings, "echo $ROS_DISTRO")       # no terminal for the checks
    assert check.startswith("docker exec yahboom_robot bash -c ") and "-it" not in check
    upload = car.remote_shell(settings, "cat > /tmp/x", stdin=True)    # stdin for the upload
    assert upload.startswith("docker exec -i yahboom_robot bash -c ")


def test_camera_file_checks_the_image_size(tmp_path):
    cal = tmp_path / "cam.yaml"
    cal.write_text("image_width: 640\nimage_height: 480\ncamera_matrix: {data: [1, 0, 0, 0, 1, 0, 0, 0, 1]}\n"
                   "distortion_coefficients: {data: [0, 0, 0, 0, 0]}\n")
    assert car.camera_file({**SETTINGS, "camera": ""}) == ""
    assert car.camera_file({**SETTINGS, "camera": str(cal)}) == str(cal)
    with pytest.raises(RuntimeError, match="is for 640 x 480"):
        car.camera_file({**SETTINGS, "camera": str(cal), "image_width": 1280, "image_height": 720})
    with pytest.raises(RuntimeError, match="no camera calibration"):
        car.camera_file({**SETTINGS, "camera": str(tmp_path / "missing.yaml")})


def test_upload_sends_the_node_through_ssh_stdin(monkeypatch, tmp_path):
    calls = []

    class Result:
        returncode, stderr = 0, b""

    def fake_run(cmd, **kw):
        calls.append((cmd, kw["input"]))
        return Result()

    monkeypatch.setattr(car.subprocess, "run", fake_run)
    car.upload_node(SETTINGS)
    cmd, payload = calls[0]
    assert cmd[-2] == "jetson@192.168.1.85" and "cat > /tmp/camslab_car_node.py" in cmd[-1]
    assert cmd[-1].startswith("docker exec -i yahboom_robot")
    assert payload == open(car.NODE_FILE, "rb").read() and b"def run(args)" in payload
    car.upload_node({**SETTINGS, "camera_host": ""})              # the camera is here: no-op
    assert len(calls) == 1


def test_find_scans_the_cars_subnet_by_default():
    assert car.default_subnet("jetson@192.168.1.85") == "192.168.1.0/24"
    assert car.default_subnet("10.0.3.7") == "10.0.3.0/24"
    assert car.default_subnet("") == "192.168.1.0/24"


def test_check_ssh_explains_an_unknown_host(monkeypatch, capsys):
    """An unknown host key makes batch ssh fail; the camera stream could not answer the
    prompt, so the check stops with the setup hint instead."""
    monkeypatch.setattr(car, "ssh_run", lambda host, command, **kw: (255, "", "Host key verification failed."))
    assert not car.check_ssh(SETTINGS)
    out = capsys.readouterr().out
    assert "Host key verification failed" in out and "car setup" in out
    monkeypatch.setattr(car, "ssh_run", lambda host, command, **kw: (0, "", ""))
    assert car.check_ssh(SETTINGS)
    assert car.check_ssh({**SETTINGS, "camera_host": ""})    # the camera is here: no ssh


def test_rate_counts_events_in_its_window():
    rate = car.Rate(window=2.0)
    assert rate.hz(now=10.0) == 0.0 and rate.text(now=10.0).strip() == "--"
    for i in range(21):
        rate.tick(value=i, now=10.0 + 0.1 * i)
    assert rate.hz(now=12.0) == pytest.approx(10.0, rel=0.05)
    assert rate.value == 20
    assert rate.hz(now=15.0) == 0.0                       # everything fell out of the window


def test_screen_rewrites_in_place_only_on_a_terminal(monkeypatch):
    out = io.StringIO()
    monkeypatch.setattr(car.sys, "stdout", out)
    screen = car.Screen()
    screen.draw(["a", "b"])
    screen.draw(["c"])
    assert out.getvalue() == "a\nb\nc\n"                  # a pipe: plain lines


def test_the_preview_is_decoded():
    cv2 = pytest.importorskip("cv2")

    class Msg:
        def __init__(self, data):
            self.data = data

    bgr = np.zeros((4, 6, 3), np.uint8)
    bgr[..., 0] = 200                                     # blue
    ok, jpeg = cv2.imencode(".jpg", bgr)
    decoded = car.decode_compressed(Msg(jpeg.tobytes()))
    assert decoded.shape == (4, 6, 3) and decoded[..., 0].mean() > 150


def test_set_camera_setting_keeps_the_comment(tmp_path):
    path = tmp_path / "survey.yaml"
    path.write_text("name: a\ncamera: \"\"                       # e.g. yahboom_r2_640x480\nport: 8000\n")
    assert calibration.set_camera_setting(str(path), "yahboom_r2_640x480")
    lines = path.read_text().splitlines()
    assert lines[1].startswith("camera: yahboom_r2_640x480") and lines[1].endswith("# e.g. yahboom_r2_640x480")
    assert lines[0] == "name: a" and lines[2] == "port: 8000"
    path.write_text("name: a\n")
    assert not calibration.set_camera_setting(str(path), "x")


def test_session_calibrates_from_rendered_views_and_installs(tmp_path, monkeypatch):
    """A Session fed like the live client feeds it: every frame, views kept at most every
    `interval` seconds until the coverage is complete, then finish() writes the files and
    install() copies the calibration next to the settings file."""
    cv2 = pytest.importorskip("cv2")
    KALIBR, render_board = board_renderer()

    px_per_m = 2000.0
    texture, _ = render_board(cv2, "DICT_APRILTAG_36h11", KALIBR, px_per_m, 2)
    width, height = 640, 480
    K = np.array([[520.0, 0.0, 330.0], [0.0, 515.0, 236.0], [0.0, 0.0, 1.0]])
    half = texture.shape[0] / px_per_m / 2.0
    pixel = np.array([[1 / px_per_m, 0.0, -half], [0.0, 1 / px_per_m, -half], [0.0, 0.0, 1.0]])
    rng = np.random.default_rng(2)
    cameras = tmp_path / "cameras"
    cameras.mkdir()
    monkeypatch.setattr(calibration, "cameras_dir", lambda: str(cameras))
    settings = tmp_path / "survey.yaml"
    settings.write_text('camera: ""   # name\n')
    session = calibration.Session("test_640x480", views=12, interval=0.5,
                                  report=str(tmp_path / "report"))
    now = 0.0
    assert session.needed().startswith("12 more views")
    for _ in range(3000):
        R = cv2.Rodrigues(rng.uniform([-0.7, -0.7, -0.4], [0.7, 0.7, 0.4]))[0]
        t = rng.uniform([-0.5, -0.4, 0.9], [0.5, 0.4, 2.6])
        H = K @ np.column_stack([R[:, 0], R[:, 1], t]) @ pixel
        image = cv2.warpPerspective(texture, H, (width, height), borderValue=255)
        now += 0.2
        kept = session.feed(cv2.cvtColor(image, cv2.COLOR_GRAY2BGR), now)
        if kept:
            assert now - session.last_kept == 0.0
        if session.done():
            break
    assert session.done() and session.kept >= 12 and session.coverage.complete()
    assert session.needed() == "" and "areas 9/9" in session.status()
    assert session.picture(["line"]).shape == (height, width, 3)
    rms, K_hat, _, path = session.finish()
    assert rms < 0.5 and K_hat[0, 0] == pytest.approx(520.0, rel=0.02)
    assert os.path.isfile(path) and os.path.isfile(tmp_path / "report" / "before_after.png")
    installed = session.install(str(settings))
    assert installed == str(cameras / "test_640x480.yaml") and os.path.isfile(installed)
    assert settings.read_text().startswith("camera: test_640x480")
    assert any("installed" in line for line in session.log)


def test_session_refuses_too_few_views(tmp_path):
    pytest.importorskip("cv2")
    session = calibration.Session("x", report=str(tmp_path))
    with pytest.raises(RuntimeError, match="no image"):
        session.finish()
    session.feed(np.full((480, 640), 255, np.uint8), 0.0)
    with pytest.raises(RuntimeError, match="0 board views"):
        session.finish()
    assert math.isinf(session.last_kept) and session.status().startswith("no tags")


def test_chrony_configs_name_the_right_addresses():
    assert car.ntp_rich_rule("192.168.1.85") == (
        'rule family="ipv4" source address="192.168.1.85/32" service name="ntp" accept')
    assert car.chrony_server_lines("192.168.1.85") == ["allow 192.168.1.85", "local stratum 10"]
    client = car.chrony_client_conf("192.168.1.153")
    assert "server 192.168.1.153 iburst minpoll 2 maxpoll 4 prefer\n" in client
    assert "makestep 1 -1\n" in client and "pool" not in client
    assert car.timesyncd_conf("192.168.1.153") == (
        "[Time]\nNTP=192.168.1.153\nPollIntervalMinSec=16\nPollIntervalMaxSec=64\n")


def test_route_to_parses_ip_route(monkeypatch):
    class Result:
        def __init__(self, stdout):
            self.stdout = stdout

    monkeypatch.setattr(car.subprocess, "run", lambda *a, **k: Result(
        "192.168.1.85 dev wlp1s0 src 192.168.1.153 uid 1000 \n    cache \n"))
    assert car.route_to("192.168.1.85") == ("wlp1s0", "192.168.1.153")
    monkeypatch.setattr(car.subprocess, "run", lambda *a, **k: Result("RTNETLINK answers: Network is unreachable\n"))
    assert car.route_to("192.168.1.85") is None


def test_ros_deb_url_reads_the_package_index(monkeypatch):
    import gzip
    import io

    index = ("Package: ros-humble-aruco\nFilename: pool/main/r/ros-humble-aruco/x.deb\n\n"
             "Package: ros-humble-aruco-msgs\nVersion: 5.0.4-1jammy.20231218.234232\n"
             "Filename: pool/main/r/ros-humble-aruco-msgs/ros-humble-aruco-msgs_5.0.4-1jammy.20231218.234232_arm64.deb\n\n")
    seen = []

    class Response(io.BytesIO):
        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

    def fake_urlopen(url, timeout=0):
        seen.append(url)
        return Response(gzip.compress(index.encode()))

    monkeypatch.setattr(car.urllib.request, "urlopen", fake_urlopen)
    url = car.ros_deb_url("ros-humble-aruco-msgs", "jammy", "arm64")
    assert seen == ["http://packages.ros.org/ros2/ubuntu/dists/jammy/main/binary-arm64/Packages.gz"]
    assert url == ("http://packages.ros.org/ros2/ubuntu/pool/main/r/ros-humble-aruco-msgs/"
                   "ros-humble-aruco-msgs_5.0.4-1jammy.20231218.234232_arm64.deb")
    with pytest.raises(RuntimeError, match="not in"):
        car.ros_deb_url("ros-humble-nothing", "jammy", "arm64")
