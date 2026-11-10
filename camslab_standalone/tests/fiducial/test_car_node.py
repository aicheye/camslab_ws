"""camslab.car_node, the camera node that runs on the car, without ROS or a camera: its
arguments, the frame convention shared with camslab.aruco, and the detection on a
rendered marker image."""

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from camslab import aruco, car, car_node

SETTINGS = {"name": "angostura", "video_device": "/dev/video0", "image_width": 640,
            "image_height": 480, "framerate": 30.0, "pixel_format": "yuyv",
            "camera_host": "", "dictionary": "DICT_4X4_50", "marker_size": 0.14,
            "nominal_fov": 0.0}


def test_the_node_parses_without_ros():
    """rclpy and cv2 are imported inside run(): the module and its parser need neither."""
    assert car_node.parse_args(["--name", "a"]).name == "a"
    with pytest.raises(SystemExit):
        car_node.parse_args([])


def test_arguments_round_trip_through_the_parser(tmp_path):
    cal = tmp_path / "cam.yaml"
    cal.write_text("image_width: 640\nimage_height: 480\ncamera_matrix: {data: "
                   "[528.1, 0, 325.2, 0, 527.9, 239.4, 0, 0, 1]}\ndistortion_coefficients: {data: "
                   "[-0.41, 0.19, 0.0012, -0.0007, -0.04]}\n")
    args = car_node.parse_args(car.node_arguments(SETTINGS, str(cal)))
    assert (args.name, args.device, args.width, args.height, args.fps) == ("angostura", "/dev/video0", 640, 480, 30.0)
    assert (args.dictionary, args.marker_size, args.pixel_format) == ("DICT_4X4_50", 0.14, "yuyv")
    K, dist = car_node.intrinsics(args)
    expected_K, expected_dist, _ = aruco.intrinsics_of_file(str(cal))
    assert K == pytest.approx(expected_K) and dist == pytest.approx(expected_dist)
    assert args.detect and args.draw and args.preview_rate == 2.0
    assert (args.min_side, args.ambiguity_ratio) == (30.0, 2.0)
    # a launch file appends --ros-args; they are not the node's
    args = car_node.parse_args(["--name", "b", "--ros-args", "-r", "__node:=camera"])
    assert args.name == "b"


def test_nominal_intrinsics_match_the_aruco_module():
    args = car_node.parse_args(car.node_arguments({**SETTINGS, "nominal_fov": 60.0}))
    K, dist = car_node.intrinsics(args)
    assert K == pytest.approx(aruco.nominal_intrinsics(640, 480, 60.0)) and not dist.any()
    assert car_node.intrinsics(car_node.parse_args(["--name", "a"])) is None


def test_frame_convention_is_that_of_the_aruco_module():
    assert np.array_equal(car_node.OPTICAL_TO_BODY, aruco.OPTICAL_TO_BODY)
    assert np.array_equal(car_node.marker_corners(0.14), aruco.marker_corners(0.14))
    pytest.importorskip("cv2")
    rng = np.random.default_rng(1)
    for _ in range(5):
        rvec, tvec = rng.normal(size=3), rng.normal(size=3)
        R, t = car_node.observation(rvec, tvec)
        T = aruco.marker_pose(rvec, tvec)
        assert R == pytest.approx(T[:3, :3]) and t == pytest.approx(T[:3, 3])
        q = car_node.quaternion_of(R)
        assert np.abs(Rotation.from_quat(q).as_matrix() - R).max() < 1e-9


def render(cv2, K, dist, marker_id, size, R_o, t_o, shape=(480, 640)):
    """A camera image of one marker at pose (R_o, t_o) in the optical frame: the marker
    bitmap warped onto its projected corners, white elsewhere."""
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    draw = getattr(cv2.aruco, "generateImageMarker", None) or cv2.aruco.drawMarker
    bitmap = draw(aruco_dict, marker_id, 300)
    rvec, _ = cv2.Rodrigues(R_o)
    corners, _ = cv2.projectPoints(car_node.marker_corners(size), rvec, t_o, K, dist)
    src = np.array([[0, 0], [300, 0], [300, 300], [0, 300]], np.float32)
    H = cv2.getPerspectiveTransform(src, corners.reshape(4, 2).astype(np.float32))
    image = cv2.warpPerspective(bitmap, H, (shape[1], shape[0]), flags=cv2.INTER_LINEAR,
                                borderMode=cv2.BORDER_CONSTANT, borderValue=255)
    return image


def test_detector_recovers_a_rendered_marker_pose():
    cv2 = pytest.importorskip("cv2")
    K = aruco.nominal_intrinsics(640, 480, 60.0)
    dist = np.zeros(5)
    detector = car_node.Detector("DICT_4X4_50", 0.14, K, dist)
    # the marker 1.2 m ahead, 0.2 m to the right, facing the camera, turned 20 deg about
    # its vertical axis
    R_o = Rotation.from_euler("xyz", [np.pi, 0.0, 0.0]).as_matrix() @ Rotation.from_euler("y", 0.35).as_matrix()
    t_o = np.array([0.2, 0.05, 1.2])
    image = render(cv2, K, dist, 3, 0.14, R_o, t_o)
    found = detector.detect(image)
    assert list(found) == [3]
    R, t = found[3]
    expected_R, expected_t = aruco.marker_pose(cv2.Rodrigues(R_o)[0], t_o)[:3, :3], car_node.OPTICAL_TO_BODY @ t_o
    assert t == pytest.approx(expected_t, abs=0.02)
    assert t[0] == pytest.approx(1.2, abs=0.02) and t[1] == pytest.approx(-0.2, abs=0.02)
    assert aruco.rotation_angle(R.T @ expected_R) < 0.1
    drawn = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    detector.draw(drawn, found)
    assert not np.array_equal(drawn, cv2.cvtColor(image, cv2.COLOR_GRAY2BGR))


def camera_image(cv2, K, marker_id, R_o, t_o, rng, blur=0.8, noise=3.0, scale=4):
    """render() as a camera would see it: drawn at `scale` times the resolution with a
    white margin around the marker, reduced, blurred and with pixel noise. Returns the
    image and the true corners."""
    margin = 0.25 * 0.14
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    draw = getattr(cv2.aruco, "generateImageMarker", None) or cv2.aruco.drawMarker
    paper = np.full((900, 900), 255, np.uint8)
    paper[150:750, 150:750] = draw(aruco_dict, marker_id, 600)
    h = 0.07 + margin
    sheet = np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]])
    rvec, _ = cv2.Rodrigues(R_o)
    projected, _ = cv2.projectPoints(sheet, rvec, t_o, K, np.zeros(5))
    src = np.array([[0, 0], [900, 0], [900, 900], [0, 900]], np.float32) - 0.5
    dst = ((projected.reshape(4, 2) + 0.5) * scale - 0.5).astype(np.float32)
    image = cv2.warpPerspective(paper, cv2.getPerspectiveTransform(src, dst),
                                (640 * scale, 480 * scale), borderMode=cv2.BORDER_CONSTANT,
                                borderValue=140)
    image = cv2.resize(image, (640, 480), interpolation=cv2.INTER_AREA).astype(float)
    image = cv2.GaussianBlur(image, (0, 0), blur) + rng.normal(0.0, noise, image.shape)
    corners, _ = cv2.projectPoints(car_node.marker_corners(0.14), rvec, t_o, K, np.zeros(5))
    return np.clip(image, 0, 255).astype(np.uint8), corners.reshape(4, 2)


def facing(yaw_deg):
    """Optical-frame rotation of a marker that faces the camera, turned about its
    vertical axis."""
    return np.diag([1.0, -1.0, -1.0]) @ Rotation.from_euler("y", yaw_deg, degrees=True).as_matrix()


def test_corners_are_refined_below_half_a_pixel():
    """The contour corners are about 0.8 px off; the survey needs the refined ones, also
    on a marker of 25 px, where OpenCV's default window no longer refines."""
    cv2 = pytest.importorskip("cv2")
    K = aruco.nominal_intrinsics(640, 480, 60.0)
    rng = np.random.default_rng(0)
    detector = car_node.Detector("DICT_4X4_50", 0.14, K, np.zeros(5), min_side=0.0)
    for distance in (1.0, 2.0, 3.0):
        errors = []
        for _ in range(10):
            t_o = np.array([rng.uniform(-0.3, 0.3) * distance, 0.02, distance])
            image, truth = camera_image(cv2, K, 1, facing(rng.uniform(20.0, 40.0)), t_o, rng)
            detector.detect(image)
            assert detector.found_ids is not None
            quad = detector.found_corners[0].reshape(4, 2)
            errors.append(np.sqrt(((quad - truth) ** 2).sum(axis=1).mean()))
        assert np.mean(errors) < 0.5


def test_a_small_marker_is_dropped():
    cv2 = pytest.importorskip("cv2")
    K = aruco.nominal_intrinsics(640, 480, 60.0)
    rng = np.random.default_rng(0)
    # 0.14 m at 3.5 m with f = 554 px: a side of 22 px
    image, _ = camera_image(cv2, K, 2, facing(30.0), np.array([0.3, 0.0, 3.5]), rng)
    detector = car_node.Detector("DICT_4X4_50", 0.14, K, np.zeros(5), min_side=30.0)
    assert detector.detect(image) == {} and detector.rejected == {2: car_node.SMALL}
    assert car_node.shortest_side(detector.found_corners[0]) == pytest.approx(19.0, abs=3.0)
    drawn = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    detector.draw(drawn, {})
    assert not np.array_equal(drawn, cv2.cvtColor(image, cv2.COLOR_GRAY2BGR))
    detector = car_node.Detector("DICT_4X4_50", 0.14, K, np.zeros(5), min_side=0.0, ambiguity_ratio=0.0)
    assert list(detector.detect(image)) == [2] and detector.rejected == {}


def test_an_ambiguous_pose_is_dropped():
    """Head-on at 2.5 m the two planar solutions fit almost equally well and the better
    one is often the wrong one; turned by 40 deg at 1 m they do not."""
    cv2 = pytest.importorskip("cv2")
    K = aruco.nominal_intrinsics(640, 480, 60.0)
    rng = np.random.default_rng(3)
    checked = car_node.Detector("DICT_4X4_50", 0.14, K, np.zeros(5), min_side=0.0, ambiguity_ratio=2.0)
    unchecked = car_node.Detector("DICT_4X4_50", 0.14, K, np.zeros(5), min_side=0.0, ambiguity_ratio=0.0)

    def errors(detector, distance, yaw):
        """Orientation errors [deg] of the published poses over 40 images, and the
        number dropped as ambiguous."""
        out, dropped = [], 0
        for _ in range(40):
            R_o = facing(yaw + rng.uniform(-3.0, 3.0))
            image, _ = camera_image(cv2, K, 0, R_o, np.array([0.1, 0.0, distance]), rng)
            found = detector.detect(image)
            dropped += detector.rejected.get(0) == car_node.AMBIGUOUS
            if 0 in found:
                expected = aruco.marker_pose(cv2.Rodrigues(R_o)[0], np.zeros(3))[:3, :3]
                out.append(np.degrees(aruco.rotation_angle(found[0][0].T @ expected)))
        return np.array(out), dropped

    near, dropped = errors(checked, 1.0, 40.0)
    assert dropped == 0 and len(near) == 40 and near.max() < 2.0
    far, dropped = errors(checked, 2.5, 0.0)
    everything, _ = errors(unchecked, 2.5, 0.0)
    assert dropped > 0 and len(far) + dropped == 40
    assert np.sqrt((far ** 2).mean()) < np.sqrt((everything ** 2).mean())


def test_only_the_configured_ids_are_kept():
    cv2 = pytest.importorskip("cv2")
    K = aruco.nominal_intrinsics(640, 480, 60.0)
    image = render(cv2, K, np.zeros(5), 7, 0.14, np.diag([1.0, -1.0, -1.0]), np.array([0.0, 0.0, 1.0]))
    assert list(car_node.Detector("DICT_4X4_50", 0.14, K, np.zeros(5)).detect(image)) == [7]
    kept = car_node.Detector("DICT_4X4_50", 0.14, K, np.zeros(5), ids=[0, 1, 2]).detect(image)
    assert kept == {}
    assert car_node.parse_args(["--name", "a", "--ids", "0", "1"]).ids == [0, 1]


def test_no_calibration_gives_ids_only():
    cv2 = pytest.importorskip("cv2")
    K = aruco.nominal_intrinsics(640, 480, 60.0)
    detector = car_node.Detector("DICT_4X4_50", 0.14)
    image = render(cv2, K, np.zeros(5), 0, 0.14, np.diag([1.0, -1.0, -1.0]), np.array([0.0, 0.0, 1.0]))
    assert detector.detect(image) == {}
    assert detector.found_ids.reshape(-1).tolist() == [0]
