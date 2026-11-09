"""A car of the lab from this computer: check it over ssh, run its camera node, calibrate
the camera and watch the survey topics live. The car runs one node, car_node.py, which
this tool uploads and starts over ssh: it reads the camera, detects the markers and
streams their poses (`observations`) and a low-rate JPEG preview. The motion capture
client, the estimator, RViz and the web UI run here (docs/hardware-survey.md).

    ros2 run camslab car find [--subnet 192.168.1.0/24]   the cars on the lab network
    ros2 run camslab car setup                        ssh key, container, ROS, OpenCV, camera, clock
    ros2 run camslab car clock                        set the car's clock from this computer
    ros2 run camslab car camera                       the camera node on the car, rates here
    ros2 run camslab car calibrate --name <model>_<w>x<h>
    ros2 run camslab car status [--show]              topic rates; --show: the preview

Every command reads config/survey.yaml, or --config <file>, with key=value overrides:
`car status name:=yahboom1`. The car is `camera_host` (user@address), its ROS container
`container` and ROS distribution `ros_distro`; `ROS_DOMAIN_ID` of this shell is passed to
the car so the topics meet. setup, camera and calibrate need ssh; status needs only the
topics. find needs no settings but the user of `camera_host`: it tries ssh on every
address of the subnet, and the cars are the hosts whose ssh host key is the one of the car
image (they are all flashed from it) and that answer the login.
"""

import argparse
import collections
import concurrent.futures
import ipaddress
import math
import os
import select
import shlex
import socket
import subprocess
import sys
import time

import numpy as np
import yaml

from . import calibration

SSH = ["ssh", "-o", "ConnectTimeout=10", "-o", "ServerAliveInterval=5"]
REMOTE_ENV = "source /opt/ros/{distro}/setup.bash && export ROS_DOMAIN_ID={domain}"
NODE_FILE = os.path.join(os.path.dirname(os.path.realpath(__file__)), "car_node.py")
REMOTE_NODE = "/tmp/camslab_car_node.py"        # where the car keeps its copy
CLOCK_OFFSET = 0.02                             # tolerated car clock offset [s]


# --- settings and the remote commands -------------------------------------------------------

def default_settings_file() -> str:
    """config/survey.yaml of the source tree when this file is there, else the installed
    share."""
    here = os.path.dirname(os.path.realpath(__file__))
    source = os.path.normpath(os.path.join(here, "..", "config", "survey.yaml"))
    if os.path.isfile(source):
        return source
    from ament_index_python.packages import get_package_share_directory

    return os.path.join(get_package_share_directory("camslab"), "config", "survey.yaml")


def load_settings(path: str, overrides=()) -> dict:
    """The settings file with `key:=value` (or key=value) overrides, parsed as YAML."""
    with open(path) as f:
        settings = yaml.safe_load(f)
    for item in overrides:
        key, sep, value = item.replace(":=", "=", 1).partition("=")
        if not sep or key not in settings:
            raise SystemExit(f"{item!r}: expected key=value with a key of {path}")
        settings[key] = yaml.safe_load(value)
    return settings


def domain_id() -> str:
    return os.environ.get("ROS_DOMAIN_ID", "0")


def remote_shell(settings: dict, command: str, tty: bool = False, stdin: bool = False) -> str:
    """`command` run on the car in a login shell with ROS sourced, inside the container
    when one is set. `tty` asks docker for a terminal, which only works through
    `ssh -tt` (the camera node, so that Ctrl-C reaches it); `stdin` passes this end's
    stdin through (the upload); the checks use neither."""
    env = REMOTE_ENV.format(distro=settings.get("ros_distro") or "humble", domain=domain_id())
    script = f"{env} && {command}"
    container = settings.get("container") or ""
    if container:
        flags = "-it " if tty else "-i " if stdin else ""
        return f"docker exec {flags}{shlex.quote(container)} bash -c {shlex.quote(script)}"
    return f"bash -c {shlex.quote(script)}"


def camera_file(settings: dict) -> str:
    """Path of the `camera` calibration: a name in config/cameras/ or a path. Empty when
    `camera` is empty; a RuntimeError when the file is missing or for another image
    size, because the intrinsics scale with the resolution."""
    camera = str(settings.get("camera") or "")
    if not camera:
        return ""
    if os.sep in camera or camera.endswith(".yaml"):
        path = os.path.expanduser(camera)
    else:
        path = os.path.join(calibration.cameras_dir(), f"{camera}.yaml")
    if not os.path.isfile(path):
        raise RuntimeError(f"no camera calibration {path}; run `ros2 run camslab car "
                           "calibrate` (docs/hardware-survey.md)")
    with open(path) as f:
        data = yaml.safe_load(f)
    size = (int(data["image_width"]), int(data["image_height"]))
    if size != (int(settings["image_width"]), int(settings["image_height"])):
        raise RuntimeError(f"{path} is for {size[0]} x {size[1]} images, the camera is set to "
                           f"{settings['image_width']} x {settings['image_height']}")
    return os.path.abspath(path)


def node_arguments(settings: dict, calibration_file: str = "", detect: bool = True,
                   draw: bool = True, preview_rate: float = None, jpeg_quality: int = 80) -> list:
    """The command line of car_node.py for these settings. The intrinsics of
    `calibration_file` go on the command line, so the car needs no file but the node."""
    args = ["--name", str(settings["name"]), "--device", str(settings["video_device"]),
            "--width", str(int(settings["image_width"])),
            "--height", str(int(settings["image_height"])),
            "--fps", f"{float(settings['framerate']):g}",
            "--pixel-format", str(settings["pixel_format"]),
            "--dictionary", str(settings["dictionary"]),
            "--marker-size", f"{float(settings['marker_size']):g}"]
    if calibration_file:
        from .aruco import intrinsics_of_file

        K, dist, _ = intrinsics_of_file(calibration_file)
        args += ["--fx", repr(float(K[0, 0])), "--fy", repr(float(K[1, 1])),
                 "--cx", repr(float(K[0, 2])), "--cy", repr(float(K[1, 2])),
                 "--dist", *[repr(float(d)) for d in dist]]
    elif float(settings.get("nominal_fov") or 0.0) > 0.0:
        args += ["--fov", f"{float(settings['nominal_fov']):g}"]
    if preview_rate is None:
        preview_rate = float(settings.get("preview_rate", 2.0))
    args += ["--preview-rate", f"{preview_rate:g}", "--jpeg-quality", str(int(jpeg_quality))]
    if not detect:
        args.append("--no-detect")
    if draw:
        args.append("--draw")
    return args


def node_command(settings: dict, calibration_file: str = "", **options) -> str:
    """car_node.py on the car as a shell command. A node left from an earlier run holds
    the camera, so it is stopped first; the anchored pattern matches the python process
    and not the shell that runs this command, whose own command line contains the path."""
    args = " ".join(shlex.quote(a) for a in node_arguments(settings, calibration_file, **options))
    return (f"pkill -INT -f '^python3 {REMOTE_NODE}' && sleep 1; "
            f"exec python3 {REMOTE_NODE} {args}")


def camera_ssh_command(settings: dict, calibration_file: str = "", batch: bool = False,
                       **options) -> list:
    """The ssh command that runs the camera node on the car in the foreground; Ctrl-C or
    SIGINT stops it there too. `batch` refuses password prompts, for a launch file."""
    ssh_options = ["-o", "BatchMode=yes"] if batch else []
    return [*SSH, *ssh_options, "-tt", str(settings["camera_host"]),
            remote_shell(settings, node_command(settings, calibration_file, **options), tty=True)]


def upload_node(settings: dict, source: str = NODE_FILE) -> None:
    """Copies car_node.py to REMOTE_NODE on the car, into the container when one is set,
    through ssh's stdin. A RuntimeError names the ssh error when it fails."""
    host = str(settings.get("camera_host") or "")
    if not host:
        return
    with open(source, "rb") as f:
        script = f.read()
    command = remote_shell(settings, f"cat > {REMOTE_NODE}", stdin=True)
    try:
        result = subprocess.run([*SSH, "-o", "BatchMode=yes", host, command], input=script,
                                capture_output=True, timeout=30)
    except subprocess.TimeoutExpired:
        raise RuntimeError(f"uploading {source} to {host} timed out")
    if result.returncode != 0:
        raise RuntimeError(f"uploading {source} to {host} failed: "
                           f"{result.stderr.decode(errors='replace').strip() or result.returncode}; "
                           "run `ros2 run camslab car setup`")


# --- rates and the terminal -------------------------------------------------------------------

class Rate:
    """Events per second over the last `window` seconds, plus the last value seen."""

    def __init__(self, window: float = 2.0):
        self.window = window
        self.times = collections.deque()
        self.value = None

    def tick(self, value=None, now: float = None) -> None:
        now = time.monotonic() if now is None else now
        self.times.append(now)
        self.value = value
        while self.times and self.times[0] < now - self.window:
            self.times.popleft()

    def hz(self, now: float = None) -> float:
        now = time.monotonic() if now is None else now
        while self.times and self.times[0] < now - self.window:
            self.times.popleft()
        if len(self.times) < 2:
            return 0.0
        return (len(self.times) - 1) / max(now - self.times[0], 1e-9)

    def text(self, now: float = None) -> str:
        hz = self.hz(now)
        return f"{hz:5.1f} Hz" if hz else "   --  "


class Screen:
    """Rewrites a block of lines in place on a terminal; on a pipe it prints them."""

    def __init__(self):
        self.tty = sys.stdout.isatty()
        self.drawn = 0

    def draw(self, lines) -> None:
        if self.tty and self.drawn:
            sys.stdout.write(f"\x1b[{self.drawn}F")       # cursor up to the block's start
        for line in lines:
            sys.stdout.write(("\x1b[2K" if self.tty else "") + line + "\n")
        self.drawn = len(lines) if self.tty else 0
        sys.stdout.flush()

    def end(self) -> None:
        self.drawn = 0


# --- images -----------------------------------------------------------------------------------

def decode_compressed(msg) -> np.ndarray:
    import cv2

    return cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)


# --- the ROS side -----------------------------------------------------------------------------

class Watch:
    """A node that follows the car's topics: preview/compressed (the car node's JPEG
    preview; image_raw/compressed too, for a usb_cam that already streams), camera_info,
    observations, pose and estimate. Rates and the latest values are read from the main
    thread between spins."""

    def __init__(self, settings: dict, images: bool = True):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from geometry_msgs.msg import PoseStamped
        from sensor_msgs.msg import CameraInfo, CompressedImage
        from std_msgs.msg import Float64MultiArray
        from tf2_msgs.msg import TFMessage

        rclpy.init()
        self.rclpy = rclpy
        self.name = str(settings["name"])
        self.node = Node("car_watch")
        ns = f"/{self.name}"
        self.rates = {k: Rate() for k in ("preview", "camera_info", "observations",
                                          "pose", "estimate")}
        self.image_msg = None
        self.info = None
        self.markers = {}                   # id -> distance [m] of the last observations
        self.pose = None
        self.estimate = None
        qos = qos_profile_sensor_data
        if images:
            for topic in ("preview/compressed", "image_raw/compressed"):
                self.node.create_subscription(CompressedImage, f"{ns}/{topic}",
                                              self.on_compressed, qos)
        self.node.create_subscription(CameraInfo, f"{ns}/camera_info", self.on_info, qos)
        self.node.create_subscription(TFMessage, f"{ns}/observations", self.on_observations, 10)
        self.node.create_subscription(PoseStamped, f"{ns}/pose", self.on_pose, qos)
        self.node.create_subscription(Float64MultiArray, f"{ns}/estimate", self.on_estimate, 10)

    def on_compressed(self, msg):
        self.rates["preview"].tick(msg.format)
        self.image_msg = msg

    def on_info(self, msg):
        self.rates["camera_info"].tick()
        self.info = msg

    def on_observations(self, msg):
        self.rates["observations"].tick()
        self.markers = {tf.child_frame_id: math.sqrt(tf.transform.translation.x ** 2
                                                     + tf.transform.translation.y ** 2
                                                     + tf.transform.translation.z ** 2)
                        for tf in msg.transforms}

    def on_pose(self, msg):
        self.rates["pose"].tick()
        self.pose = msg.pose

    def on_estimate(self, msg):
        self.rates["estimate"].tick()
        self.estimate = list(msg.data)

    def spin(self, seconds: float) -> None:
        self.rclpy.spin_once(self.node, timeout_sec=seconds)

    def frame(self):
        """The latest preview decoded, or None; the message is consumed."""
        msg, self.image_msg = self.image_msg, None
        return None if msg is None else decode_compressed(msg)

    def image_line(self) -> str:
        preview = self.rates["preview"]
        info = "no camera_info"
        if self.info is not None:
            info = (f"camera_info {self.info.width}x{self.info.height} fx {self.info.k[0]:.0f}"
                    if any(self.info.k) else f"camera_info {self.info.width}x{self.info.height} "
                    "uncalibrated")
        return (f"camera     preview {preview.text()} {preview.value or '':6s}   "
                f"observations {self.rates['observations'].text()}   {info}")

    def lines(self) -> list:
        seen = ", ".join(f"{k.replace('marker_', '')} at {d:.2f} m"
                         for k, d in sorted(self.markers.items())) or "none"
        pose = ""
        if self.pose is not None:
            q = self.pose.orientation
            yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                          1 - 2 * (q.y * q.y + q.z * q.z)))
            pose = f"x {self.pose.position.x:6.2f}  y {self.pose.position.y:6.2f}  yaw {yaw:4.0f} deg"
        estimate = ""
        if self.estimate and len(self.estimate) >= 6:
            t = self.estimate[3:6]
            estimate = f"camera in the robot frame t_X ({t[0]:.2f}, {t[1]:.2f}, {t[2]:.2f}) m"
        return [self.image_line(),
                f"markers    seen: {seen}",
                f"mocap      pose {self.rates['pose'].text()}           {pose}",
                f"estimator  estimate {self.rates['estimate'].text()}       {estimate}"]

    def close(self) -> None:
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()


def header(settings: dict) -> str:
    host = settings.get("camera_host") or "camera on this computer"
    return f"{settings['name']} @ {host}    ROS_DOMAIN_ID {domain_id()}"


# --- commands ---------------------------------------------------------------------------------

class CameraProcess:
    """The camera node on the car over ssh as a child process: car_node.py is uploaded
    first, then run in the foreground. Its output goes to the terminal, or to a log file
    when `log` is given. `options` are those of node_arguments()."""

    def __init__(self, settings: dict, calibration_file: str = "", log: str = "", **options):
        upload_node(settings)
        self.command = camera_ssh_command(settings, calibration_file, **options)
        self.log = open(log, "wb") if log else None
        self.process = subprocess.Popen(self.command, stdout=self.log, stderr=self.log,
                                        stdin=subprocess.PIPE)

    def alive(self) -> bool:
        return self.process.poll() is None

    def stop(self) -> None:
        """Ctrl-C through the remote terminal, which is what stops the node on the car: a
        signal to ssh itself only closes the connection and leaves the node running there."""
        if self.alive():
            try:
                self.process.stdin.write(b"\x03")
                self.process.stdin.flush()
            except (BrokenPipeError, OSError):
                pass
            try:
                self.process.wait(5)
            except subprocess.TimeoutExpired:
                self.process.send_signal(2)
                try:
                    self.process.wait(2)
                except subprocess.TimeoutExpired:
                    self.process.kill()
        if self.log:
            self.log.close()


def check(label: str, ok: bool, detail: str = "", hint: str = "") -> bool:
    print(f"  {'ok  ' if ok else 'FAIL'} {label:34s} {detail}")
    if not ok and hint:
        print(f"       {hint}")
    return ok


def ssh_run(host: str, command: str, batch: bool = True, timeout: float = 30.0,
            options: list = ()):
    """Runs `command` on `host` and returns (exit code, stdout, stderr). `batch` refuses
    prompts, for passwords and for unknown host keys alike."""
    options = [*(["-o", "BatchMode=yes"] if batch else []), *options]
    try:
        result = subprocess.run([*SSH, *options, host, command], capture_output=True, text=True,
                                timeout=timeout)
    except subprocess.TimeoutExpired:
        return 124, "", "timed out"
    return result.returncode, result.stdout.strip(), result.stderr.strip()


def check_ssh(settings: dict) -> bool:
    """Whether the car answers ssh without a prompt, which the camera node needs: its
    stdin is a pipe for Ctrl-C, so a password or host key prompt would hang unanswered.
    Prints what to do when it does not."""
    host = str(settings.get("camera_host") or "")
    if not host:
        return True
    code, _, err = ssh_run(host, "true")
    if code != 0:
        print(f"ssh {host} failed without a prompt: {err or code}\n"
              f"run `ros2 run camslab car setup` once for this car; it accepts the car's host "
              "key and installs this computer's ssh key", flush=True)
    return code == 0


def clock_offset(settings: dict, samples: int = 5, timeout: float = 10.0):
    """(offset, round trip) [s] of the car's clock against this computer's: the car echoes
    its time for each line sent over one ssh session, and the sample with the shortest
    round trip is taken, offset = remote - (sent + received) / 2. None when the car does
    not answer. The image stamps come from the car's clock and the mocap poses from this
    one, so an offset shifts every observation in time."""
    host = str(settings.get("camera_host") or "")
    echo = "python3 -u -c 'import sys,time\nfor _ in sys.stdin: print(time.time())'"
    process = subprocess.Popen([*SSH, "-o", "BatchMode=yes", host, echo], stdin=subprocess.PIPE,
                               stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    best = None
    try:
        for _ in range(samples):
            sent = time.time()
            process.stdin.write("\n")
            process.stdin.flush()
            if not select.select([process.stdout], [], [], timeout)[0]:
                break
            line = process.stdout.readline()
            received = time.time()
            try:
                remote = float(line)
            except ValueError:
                break
            trip = received - sent
            if best is None or trip < best[1]:
                best = (remote - (sent + received) / 2.0, trip)
    finally:
        process.stdin.close()
        try:
            process.wait(3)
        except subprocess.TimeoutExpired:
            process.kill()
    return best


def default_subnet(camera_host: str) -> str:
    """The /24 of `camera_host`'s address, or of the lab's 192.168.1.0 when it has none."""
    address = str(camera_host or "").rpartition("@")[2]
    try:
        return str(ipaddress.ip_network(f"{address}/24", strict=False))
    except ValueError:
        return "192.168.1.0/24"


def ssh_open(address: str, timeout: float = 0.5) -> bool:
    try:
        with socket.create_connection((address, 22), timeout=timeout):
            return True
    except OSError:
        return False


def host_key(address: str) -> str:
    """The ssh host key fingerprint of `address`, or '' when it has none in 3 s."""
    try:
        scan = subprocess.run(["ssh-keyscan", "-t", "ed25519", "-T", "3", address],
                              capture_output=True, text=True, timeout=6)
        if not scan.stdout.strip():
            return ""
        fp = subprocess.run(["ssh-keygen", "-lf", "-"], input=scan.stdout, capture_output=True,
                            text=True, timeout=6)
        return fp.stdout.split()[1] if fp.returncode == 0 and fp.stdout else ""
    except (subprocess.TimeoutExpired, OSError):
        return ""


def identify(address: str, user: str) -> dict:
    """Logs in to a car and reads its hostname and Jetson serial number."""
    code, text, err = ssh_run(f"{user}@{address}",
                              "hostname; cat /proc/device-tree/serial-number 2>/dev/null | tr -d '\\0'",
                              timeout=8, options=["-o", "StrictHostKeyChecking=accept-new"])
    if code != 0:
        login = "password needed: car setup" if "ermission denied" in err else (err or str(code))[:40]
        return {"login": login, "hostname": "", "serial": ""}
    lines = text.splitlines() + ["", ""]
    return {"login": "ok", "hostname": lines[0], "serial": lines[1]}


def cmd_find(settings: dict, args) -> int:
    """Scans the subnet for hosts with ssh and shows which ones are cars."""
    host = str(settings.get("camera_host") or "")
    user = host.rpartition("@")[0] or "jetson"
    subnet = args.subnet or default_subnet(host)
    addresses = [str(a) for a in ipaddress.ip_network(subnet, strict=False).hosts()]
    print(f"scanning {subnet} for ssh ({len(addresses)} addresses) ...", flush=True)
    with concurrent.futures.ThreadPoolExecutor(64) as pool:
        up = [a for a, ok in zip(addresses, pool.map(ssh_open, addresses)) if ok]
        up.sort(key=ipaddress.ip_address)
        keys = dict(zip(up, pool.map(host_key, up)))
        if not up:
            print("no host with ssh open; is this computer on the lab network?")
            return 1
        # The car image's key: the configured car's, else the key most hosts share. Only
        # the cars are logged in to; other hosts are left alone.
        configured = host.rpartition("@")[2]
        car_key = keys.get(configured) or collections.Counter(
            k for k in keys.values() if k).most_common(1)[0][0]
        cars = [a for a in up if keys[a] == car_key]
        found = dict(zip(cars, pool.map(lambda a: identify(a, user), cars)))
    print(f"{'address':16s} {'car key':7s} {'login as ' + user:30s} {'hostname':16s} serial")
    for a in up:
        f = found.get(a, {"login": "", "hostname": "", "serial": ""})
        print(f"{a:16s} {'yes' if a in found else 'no':7s} {f['login']:30s} {f['hostname']:16s} {f['serial']}")
    print(f"\n{len(cars)} host(s) with the car image's host key {car_key[:19]}... "
          "Tell identical cars apart by serial, or by unplugging one; then use the address "
          "as camera_host:=user@address or set it in config/survey.yaml.")
    return 0


def cmd_setup(settings: dict, args) -> int:
    """Checks the way to the car and what it needs, accepts its host key and installs an
    ssh key."""
    host = str(settings.get("camera_host") or "")
    container = settings.get("container") or ""
    distro = settings.get("ros_distro") or "humble"
    device = str(settings["video_device"])
    print(f"Car {settings['name']}: {host or 'no camera_host set'}, container "
          f"{container or 'none'}, ROS {distro}, camera {device}")
    ok = True
    ok &= check("ROS_DOMAIN_ID here", "ROS_DOMAIN_ID" in os.environ, domain_id(),
                "export ROS_DOMAIN_ID=<the lab's id> in every shell, or the topics stay apart")
    import importlib.util

    ok &= check("rclpy here", importlib.util.find_spec("rclpy") is not None, "",
                "source the ROS workspace: . install/setup.bash")
    if not host:
        check("camera_host", False, "", "set camera_host: user@address in config/survey.yaml")
        return 1
    # A car at a new address has a new host key; take it now, rather than in a prompt
    # the camera stream cannot answer. A changed key for a known address still fails.
    code, _, err = ssh_run(host, "true", options=["-o", "StrictHostKeyChecking=accept-new"])
    if code != 0 and "host key" in err.lower():
        check("host key", False, host, f"{err}; if the car was reinstalled, "
              f"ssh-keygen -R {host.rpartition('@')[2]} and run setup again")
        return 1
    if code != 0:
        print(f"  ...  ssh {host} without a password failed ({err or code}); installing "
              "this computer's key, enter the car's password:")
        if subprocess.run(["ssh-copy-id", host]).returncode != 0:
            check("ssh key", False, "", f"ssh-copy-id {host} failed")
            return 1
        code, _, err = ssh_run(host, "true")
    ok &= check("ssh without a password", code == 0, host, err)
    if code != 0:
        return 1
    if container:
        code, out, err = ssh_run(host, "docker ps --format '{{.Names}}'")
        running = container in out.split()
        ok &= check("container running", running, container,
                    f"ssh {host} docker start {container}" if code == 0 else err)
        if not running:
            return 1
    ls = "ls /opt/ros" if not container else f"docker exec {shlex.quote(container)} ls /opt/ros"
    installed = " ".join(ssh_run(host, ls)[1].split()) or "nothing in /opt/ros"
    code, out, err = ssh_run(host, remote_shell(settings, "echo $ROS_DISTRO"))
    ok &= check(f"ROS {distro} on the car", code == 0 and out.endswith(distro), out or err,
                f"set ros_distro in config/survey.yaml to one of: {installed}")
    probe = ("python3 -c \"import rclpy, numpy, cv2; print('OpenCV', cv2.__version__, "
             "'with aruco' if hasattr(cv2, 'aruco') else 'WITHOUT aruco')\"")
    code, out, err = ssh_run(host, remote_shell(settings, probe))
    ok &= check("python3 with rclpy, numpy, cv2.aruco", code == 0 and "with aruco" in out,
                out or err.splitlines()[-1] if err else out,
                "inside the container: apt install python3-opencv python3-numpy (Ubuntu's "
                "OpenCV has the aruco module); rclpy comes with ROS")
    try:
        upload_node(settings)
        code, out, err = ssh_run(host, remote_shell(settings, f"python3 {REMOTE_NODE} --help"))
        ok &= check("car_node.py uploaded and runs", code == 0, REMOTE_NODE, err)
    except RuntimeError as e:
        ok &= check("car_node.py uploaded", False, "", str(e))
    code, out, err = ssh_run(host, remote_shell(settings, f"test -e {device} && ls -1 /dev/video*"))
    ok &= check(f"camera {device}", code == 0, " ".join(out.split()) or err,
                "plug the camera in, or set video_device; /dev/video* must be visible in "
                "the container")
    code, out, _ = ssh_run(host, remote_shell(
        settings, f"v4l2-ctl --list-formats-ext -d {device} 2>/dev/null | grep -E \"'|[0-9]+x[0-9]+\""))
    if code == 0 and out:
        formats = {}
        current = ""
        for line in out.splitlines():
            line = line.strip()
            if "'" in line:
                current = line.split("'")[1]
                formats.setdefault(current, [])
            elif "x" in line and current:
                size = line.split()[-1]
                if size not in formats[current]:
                    formats[current].append(size)
        wanted = {"yuyv": "YUYV", "mjpeg": "MJPG", "uyvy": "UYVY"}.get(
            str(settings["pixel_format"]).lower(), str(settings["pixel_format"]).upper())
        size = f"{settings['image_width']}x{settings['image_height']}"
        ok &= check("format and size", size in formats.get(wanted, []),
                    f"{wanted} {size}; camera offers " + "; ".join(
                        f"{k}: {' '.join(v[:6])}" for k, v in formats.items()),
                    "set pixel_format, image_width and image_height to one of those")
    clock = clock_offset(settings)
    if clock is None:
        ok &= check("clock offset", False, "no answer", "python3 on the car did not echo the time")
    else:
        offset, trip = clock
        ok &= check("clock offset", abs(offset) <= CLOCK_OFFSET,
                    f"{1e3 * offset:+.0f} ms (round trip {1e3 * trip:.0f} ms)",
                    f"the car's clock is off by more than {1e3 * CLOCK_OFFSET:.0f} ms, so the "
                    "image stamps and the mocap poses disagree in time; `ros2 run camslab car "
                    "clock` sets it for this session (chrony/NTP against this computer for good)")
    code, out, _ = ssh_run(host, remote_shell(settings, f"pgrep -af '^python3 {REMOTE_NODE}' || true"))
    check("camera node now", True, "running" if out else "not running")
    print("All checks passed." if ok else "Fix the FAIL lines above, then run setup again.")
    print(f"Next: ros2 run camslab car calibrate --name <model>_{settings['image_width']}x"
          f"{settings['image_height']}, or ros2 launch camslab survey.launch.py")
    return 0 if ok else 1


def set_clock(host: str, lead: float) -> int:
    """`sudo date` on the car to this computer's time plus `lead`, the delay between
    reading the time here and the command running there. An epoch stamp with
    milliseconds, so the car's locale cannot misparse it; -t lets sudo prompt."""
    return subprocess.run([*SSH, "-t", host, f"sudo date -u -s @{time.time() + lead:.3f}"],
                          stdout=subprocess.DEVNULL).returncode


def cmd_clock(settings: dict, args) -> int:
    """Sets the car's clock to this computer's over ssh (sudo may ask for the car's
    password). The first pass lands behind by the ssh connection time; the next passes
    add the offset just measured, until it is within CLOCK_OFFSET."""
    host = str(settings.get("camera_host") or "")
    if not host:
        print("set camera_host in config/survey.yaml")
        return 1
    if not check_ssh(settings):
        return 1
    before = clock_offset(settings)
    if before is not None:
        print(f"car clock offset before: {before[0]:+.3f} s", flush=True)
    lead, after = 0.0, None
    for _ in range(4):
        code = set_clock(host, lead)
        if code != 0:
            print(f"setting the date on {host} failed ({code})")
            return 1
        after = clock_offset(settings)
        if after is None:
            print("the car did not answer the clock check")
            return 1
        print(f"car clock offset now: {1e3 * after[0]:+.0f} ms (round trip {1e3 * after[1]:.0f} ms)",
              flush=True)
        if abs(after[0]) <= CLOCK_OFFSET:
            break
        lead -= after[0]
    ok = abs(after[0]) <= CLOCK_OFFSET
    if not ok:
        print(f"still above {1e3 * CLOCK_OFFSET:.0f} ms; the ssh connection time varies too much, "
              "run it again")
    print("The Jetson loses the time at every boot without a battery-backed clock: run this "
          "after each power-up, or install chrony on the car against this computer.")
    return 0 if ok else 1


def cmd_camera(settings: dict, args) -> int:
    """The camera node on the car in the foreground, with the rates seen here."""
    if not check_ssh(settings):
        return 1
    try:
        calibration_file = camera_file(settings)
    except RuntimeError as e:
        print(e)
        return 1
    print(f"{header(settings)}\n$ {' '.join(camera_ssh_command(settings, calibration_file))}\n",
          flush=True)
    try:
        camera = CameraProcess(settings, calibration_file)
    except RuntimeError as e:
        print(e)
        return 1
    watch = None
    try:
        try:
            watch = Watch(settings)
        except ImportError:
            print("no rclpy here: the image rate is not shown", flush=True)
        last = time.monotonic()
        while camera.alive():
            if watch is None:
                time.sleep(0.5)
                continue
            watch.spin(0.1)
            if time.monotonic() - last > 2.0:
                print("  " + watch.image_line() + "\n  " + watch.lines()[1], flush=True)
                last = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        camera.stop()
        if watch:
            watch.close()
    return camera.process.returncode or 0


def wait_for_images(watch: Watch, camera, screen: Screen, settings: dict, timeout: float):
    """Spins until an image arrives; returns False when the camera died or `timeout`
    passed."""
    start = time.monotonic()
    while time.monotonic() - start < timeout:
        watch.spin(0.1)
        if watch.image_msg is not None:
            return True
        if camera is not None and not camera.alive():
            return False
        screen.draw([header(settings), f"waiting for /{settings['name']}/preview/compressed "
                     f"({time.monotonic() - start:.0f} s)"])
    return False


def show(window: str, image) -> str:
    """Shows `image` when a display is there; returns the key pressed, if any."""
    import cv2

    try:
        cv2.imshow(window, image)
        key = cv2.waitKey(1) & 0xFF
    except cv2.error:
        return ""
    return chr(key) if key < 128 else ""


def cmd_calibrate(settings: dict, args) -> int:
    """Calibrates the camera from its topic, installs the file and names it in the
    settings."""
    import cv2

    session = calibration.Session(
        args.name, calibration.Grid(args.cols, args.rows, args.tag, args.spacing), args.views,
        args.interval, args.report, args.save, border_bits=args.border_bits)
    if not args.no_camera and not check_ssh(settings):
        return 1
    print(f"{header(settings)}\n{calibration.INSTRUCTIONS} Press c in the image window to "
          "calibrate with the views so far, q to quit.\n", flush=True)
    camera = None
    if not args.no_camera:
        # Every frame, undrawn and at a high JPEG quality: the corners are measured on it.
        try:
            camera = CameraProcess(settings, "", os.path.join(session.report, "camera.log"),
                                   detect=False, draw=False,
                                   preview_rate=float(settings["framerate"]), jpeg_quality=95)
        except RuntimeError as e:
            print(e)
            return 1
    watch = Watch(settings)
    screen = Screen()
    display = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    code = 0
    try:
        if not wait_for_images(watch, camera, screen, settings, args.timeout):
            screen.end()
            if camera is not None and not camera.alive():
                print(f"the camera node on the car stopped; its output is in {session.report}/camera.log")
                print(open(os.path.join(session.report, "camera.log"), errors="replace").read()[-2000:])
            else:
                print(f"no image on /{settings['name']}/preview/compressed in {args.timeout:.0f} s; "
                      "is ROS_DOMAIN_ID the same on both sides?")
            return 1
        last_status = 0.0
        while not session.done():
            watch.spin(0.05)
            frame = watch.frame()
            if frame is None:
                if camera is not None and not camera.alive():
                    print("the camera node on the car stopped")
                    break
                continue
            now = time.monotonic()
            session.feed(frame, now)
            for line in session.log:
                screen.end()
                print(line, flush=True)
            session.log.clear()
            needed = session.needed()
            lines = [watch.image_line(), session.status(), *([f"need: {needed}"] if needed else [])]
            if display:
                key = show("calibrate", session.picture(lines[:1] + ["c: calibrate now, q: quit"]))
                if key == "q":
                    code = 1
                    break
                if key == "c":
                    break
            elif now - last_status > calibration.STATUS_PERIOD:
                session.write_latest(lines[:1])
                last_status = now
            screen.draw([header(settings), *lines])
    except KeyboardInterrupt:
        pass
    finally:
        screen.end()
        if camera is not None:
            camera.stop()
        watch.close()
        if display:
            cv2.destroyAllWindows()
    if code:
        return code
    try:
        session.finish()
    except RuntimeError as e:
        calibration.flush(session)
        print(e)
        return 1
    session.install(args.config)
    calibration.flush(session)
    print(f"\nPictures: {session.report}/before_after.png, distortion_map.png\n"
          f"Next: commit camslab/config/cameras/{args.name}.yaml, then "
          "ros2 launch camslab survey.launch.py")
    if display and args.open:
        for picture in ("before_after.png", "distortion_map.png"):
            subprocess.Popen(["xdg-open", os.path.join(session.report, picture)],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return 0


def cmd_status(settings: dict, args) -> int:
    """The topic rates, redrawn twice a second; with --show, the car's preview (the
    detections drawn there) with the marker distances of `observations`."""
    import cv2

    watch = Watch(settings, images=args.show)
    screen = Screen()
    last = 0.0
    try:
        while True:
            watch.spin(0.05)
            now = time.monotonic()
            if args.show:
                frame = watch.frame()
                if frame is not None:
                    text = [watch.image_line().strip()] + [
                        f"{k}: {d:.2f} m" for k, d in sorted(watch.markers.items())]
                    if show("status", calibration.annotate(frame, [], text)) == "q":
                        break
            if now - last > 0.5:
                screen.draw([header(settings), *watch.lines()])
                last = now
    except KeyboardInterrupt:
        pass
    finally:
        screen.end()
        watch.close()
        if args.show:
            cv2.destroyAllWindows()
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="car", description=__doc__.split("\n\n")[0],
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default=default_settings_file(), help="settings file")
    sub = parser.add_subparsers(dest="command", required=True)
    find = sub.add_parser("find", help="scan the lab network for the cars")
    find.add_argument("--subnet", default="", help="network to scan; default: the /24 of camera_host")
    sub.add_parser("setup", help="check ssh, the container, ROS, OpenCV, the camera and the clock")
    sub.add_parser("clock", help="set the car's clock to this computer's")
    sub.add_parser("camera", help="run the camera node on the car and show the rates here")
    cal = sub.add_parser("calibrate", help="calibrate the camera from its image topic")
    cal.add_argument("--name", required=True,
                     help="calibration name, <camera model>_<width>x<height>")
    cal.add_argument("--no-camera", action="store_true",
                     help="the camera node already runs; do not start it over ssh")
    cal.add_argument("--views", type=int, default=calibration.VIEWS, help="views to keep at least")
    cal.add_argument("--interval", type=float, default=calibration.INTERVAL,
                     help="minimum time between kept views [s]")
    cal.add_argument("--timeout", type=float, default=30.0, help="seconds to wait for images")
    cal.add_argument("--report", default="", help="output directory; default "
                                                 "~/camslab_calibration/<name>")
    cal.add_argument("--save", default="", help="directory to save the kept views in")
    cal.add_argument("--no-open", dest="open", action="store_false",
                     help="do not open the pictures at the end")
    cal.add_argument("--cols", type=int, default=calibration.COLS, help="tags across")
    cal.add_argument("--rows", type=int, default=calibration.ROWS, help="tags down")
    cal.add_argument("--tag", type=float, default=calibration.TAG, help="tag side [m]")
    cal.add_argument("--spacing", type=float, default=calibration.SPACING,
                     help="gap between tags [m]")
    cal.add_argument("--border-bits", type=int, default=0,
                     help="black border of the tags in bits, 2 for a Kalibr AprilGrid; "
                          "0: found from the images")
    status = sub.add_parser("status", help="topic rates, and the image with markers")
    status.add_argument("--show", action="store_true", help="window with the car's preview")
    for p in sub.choices.values():
        p.add_argument("overrides", nargs="*", metavar="key:=value",
                       help="override a value of the settings file")
    args = parser.parse_args(argv)
    settings = load_settings(args.config, args.overrides)
    return {"find": cmd_find, "setup": cmd_setup, "clock": cmd_clock, "camera": cmd_camera,
            "calibrate": cmd_calibrate, "status": cmd_status}[args.command](settings, args)


if __name__ == "__main__":
    sys.exit(main())
