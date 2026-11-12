"""camslab_standalone.fiducial.rehearsal: the simulated camera with the acceptance rule
of the camera node on the car, the stands, and the survey of config/survey.yaml."""

import math
import os

import numpy as np
import pytest

from camslab import car, car_node, fiducial
from camslab.aruco import nominal_intrinsics
from camslab_standalone.fiducial import rehearsal

K = nominal_intrinsics(640, 480, 60.0)


def scene(markers=(), **kwargs):
    return rehearsal.CameraScene(markers=list(markers), K=K, **kwargs)


def camera_pose(R_marker, t):
    """C_T_M of a marker at t in the camera frame (x forward, z up)."""
    return fiducial.pose(R_marker, t)


def facing_camera(turn_deg=0.0):
    """Orientation in the camera frame of a marker that faces the camera, turned about
    its vertical axis."""
    return rehearsal.stand_rotation("upright", math.pi + math.radians(turn_deg))


def test_stands_face_where_they_are_told():
    for stand, (up, _) in rehearsal.STANDS.items():
        for facing in (0.0, 1.0, -2.5):
            normal = rehearsal.stand_rotation(stand, facing)[:, 2]
            assert math.atan2(normal[1], normal[0]) == pytest.approx(facing)
            assert math.degrees(math.asin(normal[2])) == pytest.approx(up)
    marker = rehearsal.stand_marker("faceup", 1.0, 2.0, 0.7, 0.13, False)
    assert marker.pose()[:3, :3] == pytest.approx(rehearsal.stand_rotation("faceup", 0.7))
    assert marker.pose()[:3, 3] == pytest.approx([1.0, 2.0, 0.13])


def test_the_side_limit_is_the_camera_nodes():
    """The shortest side of the projected corners is what car_node.shortest_side
    measures, so a marker at f s / z = 30 px is the limit: 2.58 m with f = 554 px."""
    s = scene()
    f = K[0, 0]
    limit = f * 0.14 / 30.0
    assert s.accepted(camera_pose(facing_camera(), [limit - 0.02, 0.0, 0.0]))
    assert not s.accepted(camera_pose(facing_camera(), [limit + 0.02, 0.0, 0.0]))
    T = camera_pose(facing_camera(40.0), [1.5, 0.0, 0.0])
    corners = T[:3, :3] @ car_node.marker_corners(0.14).T + T[:3, 3:4]
    quad = np.stack([K[0, 2] - f * corners[1] / corners[0], K[1, 2] - K[1, 1] * corners[2] / corners[0]], axis=1)
    assert car_node.shortest_side(quad) == pytest.approx(f * 0.14 * math.cos(math.radians(40.0)) / 1.5, rel=0.02)
    # turned by 40 deg the horizontal sides are shorter by cos(40 deg): out at 2.1 m, in at 1.9 m
    assert s.accepted(camera_pose(facing_camera(40.0), [1.9, 0.0, 0.0]))
    assert not s.accepted(camera_pose(facing_camera(40.0), [2.1, 0.0, 0.0]))


def test_a_marker_outside_the_image_or_seen_from_behind_is_not_accepted():
    s = scene()
    assert s.accepted(camera_pose(facing_camera(), [1.5, 0.0, 0.0]))
    assert not s.accepted(camera_pose(facing_camera(), [1.5, 1.0, 0.0]))        # 34 deg to the left
    assert not s.accepted(camera_pose(facing_camera(), [-1.5, 0.0, 0.0]))       # behind the camera
    assert not s.accepted(camera_pose(facing_camera(180.0), [1.5, 0.0, 0.0]))   # its back
    assert not s.accepted(camera_pose(facing_camera(75.0), [1.0, 0.0, 0.0]))    # too oblique
    s.blind = True
    assert s.observe(np.eye(4)) == {}


def survey_settings():
    return car.load_settings(os.path.join(rehearsal.config_dir(), "survey.yaml"))


def test_the_planned_stands_are_away_from_the_cayley_singularity_and_the_path():
    settings = survey_settings()
    built = rehearsal.survey_scene(settings)
    assert built.n == len(settings["marker_ids"]) == 4
    assert built.K[0, 0] == pytest.approx(523.24, abs=0.01)         # the calibration, not the nominal
    for marker in built.markers:
        assert fiducial.rotation_angle(marker.pose()[:3, :3]) < rehearsal.CAYLEY_LIMIT
    params = rehearsal.survey_params(settings)
    shape = rehearsal.make_shape(params)
    path = np.array([shape.position(t) for t in np.linspace(0.0, params.period, 801)])
    for stand in rehearsal.placed_stands(settings):
        distance = np.linalg.norm(path - [stand["x"], stand["y"]], axis=1).min()
        assert distance >= 0.5
    with pytest.raises(ValueError, match="one planned stand per marker"):
        rehearsal.survey_scene({**settings, "stands": rehearsal.corner_stands(params)[:2]})


def test_the_stands_are_past_the_corners_and_move_with_the_path():
    """Without a `stands` setting the stands are where the guide puts them: past each
    corner of the bowtie on the line the car arrives on, facing the car, in the frame
    of the path."""
    settings = survey_settings()
    settings["controller"] = {**settings["controller"], "x0": 2.0, "y0": 1.0, "theta0": 0.0}
    home = rehearsal.placed_stands(settings)
    # past the corner (4, 0), which the car reaches driving in the -y direction
    assert [home[3]["x"], home[3]["y"], home[3]["facing"]] == pytest.approx([4.0, -0.6, 90.0])
    # past the corner (4, 2) on the diagonal from (0, 0)
    d = math.atan2(1.0, 2.0)
    assert [home[1]["x"], home[1]["y"]] == pytest.approx([4.0 + 0.6 * math.cos(d), 2.0 + 0.6 * math.sin(d)])
    assert home[1]["facing"] == pytest.approx(math.degrees(d) - 180.0)
    settings["controller"] = {**settings["controller"], "x0": -1.0, "y0": 3.0, "theta0": math.pi / 2}
    moved = rehearsal.placed_stands(settings)
    assert [moved[3]["x"], moved[3]["y"], moved[3]["facing"]] == pytest.approx([0.6, 5.0, 180.0])
    planned = {**settings, "stands": [{"stand": "upright", "x": 1.0, "y": 0.0, "facing": 0.0}] * 4}
    assert [rehearsal.placed_stands(planned)[0][k] for k in ("x", "y", "facing")] == pytest.approx([-1.0, 4.0, 90.0])


def test_the_bowtie_is_fitted_into_the_safe_area():
    """Setting the safe area sets the path: the largest bowtie inside it, centred, along
    its longest edge, with the whole lap farther than the margin from every edge."""
    from camslab.safety import Polytope, check_inside
    from camslab.types import Params
    from camslab_webui.polytope import convex_hull, fit_bowtie

    def lap(corners, kappa_max=2.9):
        hull = convex_hull(corners)
        fitted = fit_bowtie(hull, 0.2, kappa_max)
        params = Params(kappa_max=kappa_max, **fitted)
        shape = rehearsal.make_shape(params)
        assert check_inside(shape, Polytope([c for p in hull for c in p]), 0.2, params.period) == ""
        assert shape.perimeter / params.period == pytest.approx(0.14, abs=0.002)
        return fitted

    big = lap([(-4, -3), (4, -3), (4, 3), (-4, 3)])
    assert (big["x0"], big["y0"], big["a"], big["b"]) == (0.0, 0.0, 2.0, 1.0)
    # along x: the direction in which no stand that faces the arriving car faces +y
    assert abs(big["theta0"]) == pytest.approx(math.pi, abs=1e-3)
    small = lap([(0, 0), (3.5, 0), (3.5, 2.5), (0, 2.5)])
    assert (small["x0"], small["y0"]) == (1.75, 1.25) and 1.4 < small["a"] < 1.5
    tilted = lap([(0, 0), (5, 0.3), (4.6, 3.1), (0.4, 2.6)], kappa_max=2.0)
    assert 1.6 < tilted["a"] < 2.0
    assert lap([(0, 0), (3, 0), (3, 6), (0, 6)])["theta0"] == pytest.approx(math.pi / 2, abs=1e-3)
    with pytest.raises(ValueError, match="too small"):
        fit_bowtie(convex_hull([(0, 0), (2, 0), (2, 1.5), (0, 1.5)]), 0.2, 2.9)


def test_the_survey_of_the_settings_file_identifies_everything():
    """Three laps with the camera's limits and the assumed noise: every marker is used,
    both laws switch on, and the errors are centimetres."""
    result = rehearsal.rehearse(survey_settings(), laps=3, dt=0.1)
    assert result["used"].sum(axis=0).min() > 100
    assert result["ready"] == (True, True)
    assert result["turning"] == 0 and result["yaw_rate"] < 0.5
    lap, e = result["laps"][-1]
    assert lap == 3 and e[0] < 0.03 and e[1] < 0.08 and e[3] < 0.12
    assert e[3] < result["laps"][0][1][3]
    rehearsal.report(result)


def test_a_gerono_gives_no_pure_translation_pair():
    """Why the survey path is a bowtie: on a gerono the heading never repeats while a
    marker is in view, the orientation law stays off, and the camera's orientation is
    not identified."""
    settings = survey_settings()
    settings["controller"] = {**settings["controller"], "shape": "gerono", "period": 60.0,
                              "x0": 2.0, "y0": 1.0, "theta0": 0.0}
    settings["stands"] = [{"stand": "upright", "x": -2.75, "y": 0.95, "facing": -30},
                          {"stand": "overhang", "x": 2.75, "y": 0.70, "facing": -150},
                          {"stand": "diamond", "x": -2.50, "y": -1.80, "facing": 120},
                          {"stand": "faceup", "x": 2.50, "y": -2.05, "facing": 120}]
    settings["straight_tol"] = 5e-4         # the simulation's: the heading must repeat
    result = rehearsal.rehearse(settings, laps=2, dt=0.1, noise=None)
    assert result["ready"][0] is False
