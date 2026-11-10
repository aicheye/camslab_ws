"""camslab_standalone.fiducial.rehearsal: the simulated camera with the acceptance rule
of the camera node on the car, the stands, and the survey of config/survey.yaml."""

import math
import os

import numpy as np
import pytest
import yaml

from camslab import car_node, fiducial
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
    with open(os.path.join(rehearsal.config_dir(), "survey.yaml")) as f:
        return yaml.safe_load(f)


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
    for stand in settings["stands"]:
        distance = np.linalg.norm(path - [stand["x"], stand["y"]], axis=1).min()
        assert distance >= 0.75
    with pytest.raises(ValueError, match="one planned stand per marker"):
        rehearsal.survey_scene({**settings, "stands": settings["stands"][:2]})


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
    settings["controller"] = {**settings["controller"], "shape": "gerono", "period": 60.0}
    result = rehearsal.rehearse(settings, laps=2, dt=0.1, noise=None)
    assert result["ready"][0] is False
