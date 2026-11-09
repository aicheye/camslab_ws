"""camslab.calibration: the calibration file, the board geometry and the choice of board
views without OpenCV, and a calibration on rendered board images when OpenCV is
installed."""

import ast
import math

import numpy as np
import pytest

from camslab.calibration import (LAYOUTS, Coverage, Grid, camera_info_yaml, is_new_view,
                                 view_feature)

KALIBR = (False, False, True)         # Kalibr AprilGrid: row by row from the bottom left


def test_camera_info_yaml_is_the_camera_info_manager_format():
    K = np.array([[520.0, 0.0, 330.0], [0.0, 515.0, 236.0], [0.0, 0.0, 1.0]])
    text = camera_info_yaml("camera", 640, 480, K, [0.1, -0.2, 0.0, 0.0], "two\nlines")
    assert text.startswith("# two\n# lines\n")
    lines = text.splitlines()
    assert "image_width: 640" in lines and "image_height: 480" in lines
    assert "distortion_model: plumb_bob" in lines
    data = {lines[i - 3]: np.array(ast.literal_eval(line.split(": ", 1)[1]))
            for i, line in enumerate(lines) if line.startswith("  data: ")}
    assert data["camera_matrix:"].reshape(3, 3) == pytest.approx(K)
    assert data["distortion_coefficients:"] == pytest.approx([0.1, -0.2, 0, 0, 0])
    P = data["projection_matrix:"].reshape(3, 4)
    assert P[:, :3] == pytest.approx(K) and np.all(P[:, 3] == 0.0)


def test_views_are_kept_only_when_they_differ():
    corners = np.array([[100.0, 100.0], [300.0, 100.0], [300.0, 260.0], [100.0, 260.0]])
    flat = view_feature(corners, [0.0, 0.0, -1.0], 640, 480)
    assert flat[:2] == pytest.approx([200 / 640, 180 / 480])
    assert flat[2] == pytest.approx(math.sqrt(200 * 160 / (640 * 480)))
    assert not is_new_view(view_feature(corners + 5.0, [0.0, 0.0, -1.0], 640, 480), [flat])
    assert is_new_view(view_feature(corners + 150.0, [0.0, 0.0, -1.0], 640, 480), [flat])
    tilted = [math.sin(0.5), 0.0, -math.cos(0.5)]
    assert is_new_view(view_feature(corners, tilted, 640, 480), [flat])
    assert is_new_view(flat, [])


def test_coverage_is_complete_only_with_every_area_size_and_tilt():
    """Features are (centre x, centre y, size, normal x, normal y). Nine areas, two views
    each of far, mid and near, and two each of flat and the four tilt directions."""
    c = Coverage()
    assert c.bins([0.1, 0.1, 0.2, 0.0, 0.0]) == ((0, 0), "far", "flat")
    assert c.bins([0.99, 0.5, 0.4, 0.5, 0.1]) == ((1, 2), "mid", "right")
    assert c.bins([0.5, 0.9, 0.7, -0.1, -0.6]) == ((2, 1), "near", "up")
    assert c.missing() == ["9 of 9 image areas", "far 2", "mid 2", "near 2", "flat 2", "left 2",
                           "right 2", "up 2", "down 2"]
    for row in range(3):
        for column in range(3):
            c.add([(column + 0.5) / 3, (row + 0.5) / 3, 0.2, 0.0, 0.0])    # far and flat, 9 times
    assert c.missing() == ["mid 2", "near 2", "left 2", "right 2", "up 2", "down 2"]
    for size in (0.35, 0.35, 0.6, 0.6):
        for nx, ny in ((-0.5, 0.0), (0.5, 0.0), (0.0, -0.5), (0.0, 0.5)):
            c.add([0.5, 0.5, size, nx, ny])
    assert c.complete() and c.text() == "areas 9/9, far 9 mid 8 near 8, flat 9 left 4 right 4 up 4 down 4"


def test_grid_places_the_tags_of_the_lab_board():
    grid = Grid(layout=KALIBR)
    corners = grid.object_points([0, 5, 30])
    pitch = 0.088 + 0.0264
    assert corners[:4, :2].mean(axis=0) == pytest.approx([0.0, 5 * pitch])      # bottom left
    assert corners[4:8, :2].mean(axis=0) == pytest.approx([5 * pitch, 5 * pitch])
    assert corners[8:, :2].mean(axis=0) == pytest.approx([0.0, 0.0])             # top left
    assert np.linalg.norm(corners[0] - corners[1]) == pytest.approx(0.088)
    assert corners[1, 0] > corners[0, 0] and corners[2, 1] > corners[1, 1]       # clockwise


def test_layout_is_found_from_one_view():
    """Every layout, seen through a perspective map, is told apart from the other seven
    when enough tags are visible."""
    grid = Grid()
    H = np.array([[600.0, 40.0, 120.0], [-30.0, 560.0, 80.0], [0.15, -0.2, 1.0]])
    ids = [0, 1, 2, 7, 9, 14, 20, 23, 29, 35]
    for layout in LAYOUTS:
        plane = grid.object_points(ids, layout)[:, :2]
        image = (H @ np.column_stack([plane, np.ones(len(plane))]).T).T
        image = image[:, :2] / image[:, 2:]
        found, residuals = grid.find_layout(ids, image)
        assert found == layout and residuals[layout] < 1e-6


def render_board(cv2, dictionary, layout, px_per_m, border_bits=1):
    """The 6 x 6 tag board as an image, white margin of one spacing. With a 2 bit border
    it is drawn as Kalibr prints an AprilGrid, with a black square of the spacing's side
    at every tag corner."""
    grid = Grid(layout=layout)
    tag_px = round(grid.tag * px_per_m)
    side = round((5 * grid.pitch + grid.tag + 2 * 0.0264) * px_per_m)
    board = np.full((side, side), 255, np.uint8)
    aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
    draw = getattr(cv2.aruco, "generateImageMarker", None) or cv2.aruco.drawMarker
    for i in range(36):
        c, r = grid.cell(i, layout)
        x = round((0.0264 + c * grid.pitch) * px_per_m)
        y = round((0.0264 + r * grid.pitch) * px_per_m)
        board[y:y + tag_px, x:x + tag_px] = draw(aruco_dict, i, tag_px, borderBits=border_bits)
    if border_bits == 2:
        gap = round(0.0264 * px_per_m)
        for c in range(7):
            for r in range(7):
                x = round(c * grid.pitch * px_per_m)
                y = round(r * grid.pitch * px_per_m)
                board[y:y + gap, x:x + gap] = 0
    origin = 0.0264 + grid.tag / 2.0        # board pixel of the grid origin, in metres
    return board, origin


@pytest.mark.parametrize("dictionary, layout, border_bits",
                         [("DICT_APRILTAG_36h11", KALIBR, 2),
                          ("DICT_APRILTAG_36h11", (False, False, False), 1)])
def test_calibration_recovers_a_rendered_camera(dictionary, layout, border_bits):
    """The board rendered through a known camera at board poses until 30 views are kept:
    the border width and the layout are found, the focal lengths are within 1 percent and
    the principal point within 3 px."""
    cv2 = pytest.importorskip("cv2")
    from camslab.calibration import Calibrator

    px_per_m = 2000.0
    texture, origin = render_board(cv2, dictionary, layout, px_per_m, border_bits)
    width, height = 640, 480
    K = np.array([[520.0, 0.0, 330.0], [0.0, 515.0, 236.0], [0.0, 0.0, 1.0]])
    half = texture.shape[0] / px_per_m / 2.0
    pixel = np.array([[1 / px_per_m, 0.0, -half], [0.0, 1 / px_per_m, -half], [0.0, 0.0, 1.0]])
    rng = np.random.default_rng(1)
    calibrator = Calibrator(Grid(), width, height, dictionary)
    for _ in range(300):
        R = cv2.Rodrigues(rng.uniform([-0.7, -0.7, -0.4], [0.7, 0.7, 0.4]))[0]
        t = rng.uniform([-0.4, -0.3, 1.0], [0.4, 0.3, 2.2])
        H = K @ np.column_stack([R[:, 0], R[:, 1], t]) @ pixel
        image = cv2.warpPerspective(texture, H, (width, height), borderValue=255)
        calibrator.add(cv2.GaussianBlur(image, (3, 3), 0.6))
        if len(calibrator.features) >= 30:
            break
    assert calibrator.detector.name == dictionary and calibrator.grid.layout == layout
    assert calibrator.detector.border_bits == border_bits
    assert len(calibrator.features) == 30 and calibrator.tilted_views() >= 5
    rms, K_hat, _ = calibrator.calibrate()
    assert rms < 0.5
    assert np.diag(K_hat)[:2] == pytest.approx(np.diag(K)[:2], rel=0.01)
    assert K_hat[:2, 2] == pytest.approx(K[:2, 2], abs=3.0)


def test_kalibr_tags_need_the_2_bit_border():
    """A Kalibr AprilGrid tag has a 2 bit black border, which OpenCV's default of 1 bit
    does not decode."""
    cv2 = pytest.importorskip("cv2")
    from camslab.calibration import TagDetector

    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    draw = getattr(cv2.aruco, "generateImageMarker", None) or cv2.aruco.drawMarker
    image = np.full((400, 400), 255, np.uint8)
    image[100:300, 100:300] = draw(d, 7, 200, borderBits=2)
    assert len(TagDetector("DICT_APRILTAG_36h11", 1).detect(image, 36)[0]) == 0
    assert list(TagDetector("DICT_APRILTAG_36h11", 2).detect(image, 36)[0]) == [7]


def test_nice_step_is_one_two_or_five():
    from camslab.calibration import nice_step

    assert [nice_step(v) for v in (149.3, 10.0, 3.0, 0.4)] == pytest.approx([50.0, 2.0, 1.0, 0.1])


def test_report_pictures_show_barrel_distortion():
    """With barrel distortion (k1 < 0) undistortion moves the image corners outward and
    leaves the principal point in place; the pictures have the expected sizes."""
    cv2 = pytest.importorskip("cv2")
    from camslab.calibration import before_after, distortion_displacement, distortion_map

    K = np.array([[520.0, 0.0, 330.0], [0.0, 515.0, 236.0], [0.0, 0.0, 1.0]])
    dist = np.array([-0.3, 0.1, 0.0, 0.0, 0.0])
    d = distortion_displacement(K, dist, 640, 480)
    assert np.linalg.norm(d[236, 330]) < 0.01
    assert d[0, 0, 0] < -10.0 and d[0, 0, 1] < -10.0           # top left corner moves up left
    assert d[479, 639, 0] > 10.0 and d[479, 639, 1] > 10.0
    assert distortion_map(K, dist, 640, 480).shape == (480 + 70, 640, 3)
    frame = np.full((480, 640, 3), 128, np.uint8)
    assert before_after(frame, K, dist).shape == (480, 2 * 640 + 8, 3)
