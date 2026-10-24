"""
AC21: closed-loop docking precision on headless MuJoCo (no ROS).

BumperbotSim on small_house.xml is driven by the Python dock_logic controller: wheel commands come
from the diff-drive inverse kinematics (r = 0.033, b = 0.17), the odometry from the wheel encoders
and the detections from emulate_detection with seeded noise. The error is measured against the
simulator ground truth.
"""
import math
import os
import random
import sys

import mujoco  # noqa: F401  (a missing mujoco is a test failure, not a skip)
import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
try:
    import bumperbot_navigation.dock_logic  # noqa: F401
except ImportError:  # not installed: use the source tree of the sibling package
    sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..", "..", "bumperbot_navigation")))

from bumperbot_mujoco.dock_marker_sim import add_noise, emulate_detection  # noqa: E402
from bumperbot_mujoco.sim_core import BumperbotSim  # noqa: E402
from bumperbot_navigation import dock_logic as dl  # noqa: E402

WORLD = os.path.join(HERE, "..", "mjcf", "small_house.xml")
R = 0.033
B = 0.17
DT = 0.05                    # control period (sim time)
DETECTION_PERIOD = 0.1       # dock_marker_sim rate 10 Hz
STAGING = dl.Pose2D(0.0, 0.0, 0.0)
CHARGER = dl.Pose2D(-0.5, 0.0, 0.0)
SIM_LIMIT = 60.0
POS_LIMIT, YAW_LIMIT = 0.05, 0.1
POSITION_NOISE, YAW_NOISE = 0.005, 0.01
MAP_BIAS = 0.05


def perturbed_starts():
    rng = np.random.default_rng(42)
    return [(float(rng.uniform(-0.1, 0.1)), float(rng.uniform(-0.1, 0.1)),
             float(rng.uniform(-0.25, 0.25))) for _ in range(6)]


STARTS = perturbed_starts()


class Wheels:
    """Diff-drive inverse kinematics and encoder odometry on a BumperbotSim."""

    def __init__(self, sim):
        self.sim = sim
        self.last = sim.wheel_positions()
        x, y, yaw = sim.base_pose()
        self.odom = dl.Pose2D(x, y, yaw)    # map == odom at the start
        self.cmd = (0.0, 0.0)

    def command(self, v, w):
        self.cmd = (v, w)
        self.sim.set_wheel_velocities((v - w * B / 2.0) / R, (v + w * B / 2.0) / R)

    def update_odometry(self):
        now = self.sim.wheel_positions()
        dl_, dr_ = now[0] - self.last[0], now[1] - self.last[1]
        self.last = now
        ds = R * (dl_ + dr_) / 2.0
        dth = R * (dr_ - dl_) / B
        mid = self.odom.yaw + dth / 2.0
        self.odom = dl.Pose2D(self.odom.x + ds * math.cos(mid), self.odom.y + ds * math.sin(mid),
                              dl.normalize_angle(self.odom.yaw + dth))


class Docker:
    """The closed loop of one run: detections, alignment, insertion, verification, undock."""

    def __init__(self, start, detect, seed):
        self.detect = detect
        self.sim = BumperbotSim(WORLD, x=STAGING.x + start[0], y=STAGING.y + start[1],
                                yaw=start[2])
        self.sim.step(0.5)
        self.wheels = Wheels(self.sim)
        self.rng = random.Random(seed)
        self.params = dl.ControlParams()
        self.controller = dl.DockController(self.params)
        self.filter = dl.DetectionFilter(0.3, 0.3)
        # charger in odom as the map says; with detections the map estimate is biased by 5 cm
        angle = self.rng.uniform(0.0, 2.0 * math.pi)
        bias = MAP_BIAS if detect else 0.0
        self.estimate = dl.Pose2D(CHARGER.x + bias * math.cos(angle),
                                  CHARGER.y + bias * math.sin(angle), CHARGER.yaw)
        self.detections = 0
        self.next_detection = 0.0
        self.t0 = self.sim.time

    @property
    def elapsed(self):
        return self.sim.time - self.t0

    def truth_error(self, pose):
        x, y, yaw = self.sim.base_pose()
        rel = dl.relative(pose, dl.Pose2D(x, y, yaw))
        return math.hypot(rel.x, rel.y), abs(dl.normalize_angle(rel.yaw))

    def detect_step(self):
        if not self.detect or self.sim.time < self.next_detection:
            return
        self.next_detection = self.sim.time + DETECTION_PERIOD
        rel = emulate_detection(self.sim.base_pose(), (CHARGER.x, CHARGER.y, CHARGER.yaw),
                                2.0, 2.1, "rear")
        if rel is None:
            return
        rel = add_noise(rel, self.rng, POSITION_NOISE, YAW_NOISE)
        detection = dl.compose(self.wheels.odom, dl.Pose2D(rel.x, rel.y, rel.yaw))
        accepted, self.estimate = self.filter.update(self.estimate, detection)
        self.detections += int(accepted)

    def tick(self, v, w):
        self.wheels.command(v, w)
        self.sim.step(DT)
        self.wheels.update_odometry()
        self.detect_step()

    def pose_in_dock(self):
        return dl.relative(self.estimate, self.wheels.odom)

    def stop(self, seconds=0.5):
        end = self.sim.time + seconds
        while self.sim.time < end:
            self.tick(0.0, 0.0)

    def wait_detections(self):
        """Collect detections at the pre-dock pose (at most 2 s), as the node does."""
        if not self.detect:
            return
        end = self.sim.time + 2.0
        while self.detections < 5 and self.sim.time < end:
            self.tick(0.0, 0.0)
        assert self.detections >= 5, "only {} detections".format(self.detections)

    def run_phase(self, command, limit):
        while self.elapsed < limit:
            cmd = command(self.pose_in_dock())
            assert not cmd.failed, "controller reported a failure at {:.1f} s".format(self.elapsed)
            if cmd.done:
                self.wheels.command(0.0, 0.0)
                return True
            self.tick(cmd.v, cmd.w)
        return False

    def dock(self):
        self.wait_detections()
        self.controller.reset()
        assert self.run_phase(self.controller.align_command, SIM_LIMIT), "alignment timed out"
        assert self.run_phase(self.controller.insert_command, SIM_LIMIT), "insertion timed out"
        self.stop(0.5)

    def undock(self):
        assert self.run_phase(self.controller.undock_command, self.elapsed + 30.0), \
            "undocking timed out"
        self.stop(0.3)


@pytest.mark.parametrize("detect", [True, False], ids=["detections_biased_map", "map_only"])
@pytest.mark.parametrize("index", range(len(STARTS)))
def test_docks_within_tolerance_and_undocks(index, detect):
    docker = Docker(STARTS[index], detect, seed=100 + index)
    docker.dock()
    assert docker.elapsed <= SIM_LIMIT, "docking took {:.1f} s".format(docker.elapsed)
    pos, yaw = docker.truth_error(CHARGER)
    assert pos <= POS_LIMIT, "start {} detect {}: position error {:.4f} m".format(
        STARTS[index], detect, pos)
    assert yaw <= YAW_LIMIT, "start {} detect {}: yaw error {:.4f} rad".format(
        STARTS[index], detect, yaw)
    assert docker.wheels.cmd == (0.0, 0.0), "the final wheel command is not zero"
    assert max(abs(v) for v in docker.sim.wheel_velocities()) < 0.05, "the wheels still turn"
    if detect:
        assert docker.detections >= 5
    docker.undock()
    pos, yaw = docker.truth_error(STAGING)
    assert pos <= POS_LIMIT, "undock position error {:.4f} m".format(pos)
    assert yaw <= YAW_LIMIT, "undock yaw error {:.4f} rad".format(yaw)
    assert docker.wheels.cmd == (0.0, 0.0)


def test_detections_remove_the_map_bias():
    """Control: the same biased charger estimate without detections misses by about the bias."""
    biased = Docker(STARTS[0], False, seed=5)
    angle_rng = random.Random(5)
    angle = angle_rng.uniform(0.0, 2.0 * math.pi)
    biased.estimate = dl.Pose2D(CHARGER.x + MAP_BIAS * math.cos(angle),
                                CHARGER.y + MAP_BIAS * math.sin(angle), CHARGER.yaw)
    biased.dock()
    pos, _ = biased.truth_error(CHARGER)
    assert pos >= 0.03, "the bias should show without detections, got {:.4f} m".format(pos)
