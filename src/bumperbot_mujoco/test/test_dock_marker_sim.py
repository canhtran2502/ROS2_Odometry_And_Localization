"""AC22: emulate_detection (range, field of view, exact pose) and the noise of dock_marker_sim."""
import math
import os
import random
import statistics

import pytest
import yaml

from bumperbot_mujoco.dock_marker_sim import Pose2D, add_noise, emulate_detection

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, "..", "config", "dock_marker_sim.yaml")
DOCK = (-0.5, 0.0, 0.0)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def reference(gt, dock):
    """Independent reference: dock pose expressed in the robot frame."""
    dx, dy = dock[0] - gt[0], dock[1] - gt[1]
    c, s = math.cos(gt[2]), math.sin(gt[2])
    return (c * dx + s * dy, -s * dx + c * dy, wrap(dock[2] - gt[2]))


def config():
    with open(CONFIG) as f:
        return yaml.safe_load(f)["dock_marker_sim"]["ros__parameters"]


def as_tuple(pose):
    return (pose.x, pose.y, pose.yaw)


@pytest.mark.parametrize("gt", [
    (0.0, 0.0, 0.0), (0.5, 0.0, 0.0), (0.5, 0.3, 0.2), (1.0, -0.5, -0.4), (-0.2, 0.1, 0.1),
    (0.0, 0.0, 0.5)])
def test_exact_relative_pose_without_noise(gt):
    got = emulate_detection(gt, DOCK, 2.0, 2.1, "rear")
    assert got is not None
    assert as_tuple(got) == pytest.approx(reference(gt, DOCK), abs=1e-9)


def test_dock_in_front_of_a_rear_detector_is_not_seen():
    assert emulate_detection((0.3, 0.1, 3.0), DOCK, 2.0, 2.1, "rear") is None


def test_accepts_pose_objects_and_tuples():
    a = emulate_detection(Pose2D(0.5, 0.1, 0.1), Pose2D(*DOCK), 2.0, 2.1, "rear")
    b = emulate_detection((0.5, 0.1, 0.1), DOCK, 2.0, 2.1, "rear")
    assert as_tuple(a) == pytest.approx(as_tuple(b), abs=1e-12)


def test_none_beyond_max_range():
    # the dock is 2.0 m behind the robot: visible exactly at the limit, invisible beyond
    assert emulate_detection((1.5, 0.0, 0.0), DOCK, 2.0, 2.1, "rear") is not None
    assert emulate_detection((1.51, 0.0, 0.0), DOCK, 2.0, 2.1, "rear") is None
    assert emulate_detection((5.0, 3.0, 0.0), DOCK, 2.0, 2.1, "rear") is None
    assert emulate_detection((1.2, 0.0, 0.0), DOCK, 1.0, 2.1, "rear") is None
    assert emulate_detection((1.2, 0.0, 0.0), DOCK, 3.0, 2.1, "rear") is not None


@pytest.mark.parametrize("bearing, visible", [
    (0.0, True), (0.5, True), (-0.5, True), (1.0, True), (-1.0, True), (1.1, False), (-1.1, False),
    (2.0, False), (math.pi, False)])
def test_field_of_view_around_the_rear_axis(bearing, visible):
    """The dock is 1 m away at the given angle from the rear axis of a robot at the origin."""
    angle_world = math.pi - bearing
    dock = (math.cos(angle_world), math.sin(angle_world), 0.0)
    got = emulate_detection((0.0, 0.0, 0.0), dock, 2.0, 2.1, "rear")
    assert (got is not None) == visible
    if visible:
        assert as_tuple(got) == pytest.approx(reference((0.0, 0.0, 0.0), dock), abs=1e-9)


def test_front_facing_detector_looks_forward():
    ahead = (1.0, 0.0, 0.0)
    assert emulate_detection((0.0, 0.0, 0.0), ahead, 2.0, 2.1, "front") is not None
    assert emulate_detection((0.0, 0.0, 0.0), ahead, 2.0, 2.1, "rear") is None
    behind = (-1.0, 0.0, 0.0)
    assert emulate_detection((0.0, 0.0, 0.0), behind, 2.0, 2.1, "front") is None
    assert emulate_detection((0.0, 0.0, 0.0), behind, 2.0, 2.1, "rear") is not None


def test_noise_standard_deviation_matches_the_configuration():
    cfg = config()
    rng = random.Random(cfg["seed"])
    clean = emulate_detection((0.5, 0.0, 0.0), DOCK, cfg["max_range"], cfg["fov"], cfg["facing"])
    samples = [add_noise(clean, rng, cfg["position_noise"], cfg["yaw_noise"])
               for _ in range(2000)]
    errors_x = [s.x - clean.x for s in samples]
    errors_y = [s.y - clean.y for s in samples]
    errors_yaw = [wrap(s.yaw - clean.yaw) for s in samples]
    for errors, expected in ((errors_x, cfg["position_noise"]), (errors_y, cfg["position_noise"]),
                             (errors_yaw, cfg["yaw_noise"])):
        assert statistics.pstdev(errors) == pytest.approx(expected, rel=0.30)
        assert abs(statistics.mean(errors)) < 0.2 * expected
    # the same seed gives the same sequence
    again = random.Random(cfg["seed"])
    first = add_noise(clean, again, cfg["position_noise"], cfg["yaw_noise"])
    assert as_tuple(first) == pytest.approx(as_tuple(samples[0]), abs=1e-15)


def test_zero_noise_returns_the_clean_pose():
    clean = emulate_detection((0.5, 0.0, 0.0), DOCK, 2.0, 2.1, "rear")
    noisy = add_noise(clean, random.Random(1), 0.0, 0.0)
    assert as_tuple(noisy) == pytest.approx(as_tuple(clean), abs=1e-15)


def test_config_values():
    cfg = config()
    assert (cfg["dock_x"], cfg["dock_y"], cfg["dock_yaw"]) == (-0.5, 0.0, 0.0)
    assert cfg["max_range"] == 2.0 and cfg["fov"] == 2.1 and cfg["facing"] == "rear"
    assert cfg["position_noise"] == 0.005 and cfg["yaw_noise"] == 0.01
    assert cfg["seed"] == 7 and cfg["rate"] == 10.0
