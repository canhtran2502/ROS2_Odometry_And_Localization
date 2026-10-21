import math
import os

import numpy as np
import pytest

pytest.importorskip("mujoco")

from bumperbot_mujoco.sim_core import BumperbotSim  # noqa: E402

MJCF_DIR = os.path.join(os.path.dirname(__file__), "..", "mjcf")
WHEEL_RADIUS = 0.033
WHEEL_SEPARATION = 0.17


def world(name):
    return os.path.join(MJCF_DIR, name + ".xml")


def test_robot_rests_on_ground_and_imu_measures_gravity():
    sim = BumperbotSim(world("empty"))
    sim.step(1.0)
    _, _, accel = sim.imu(noisy=False)
    assert accel[2] == pytest.approx(9.81, abs=0.1)
    x, y, _ = sim.base_pose()
    assert math.hypot(x, y) < 1e-3


def test_drives_straight_forward():
    sim = BumperbotSim(world("empty"))
    sim.step(1.0)
    speed = 0.2 / WHEEL_RADIUS
    sim.set_wheel_velocities(speed, speed)
    sim.step(3.0)
    x, y, yaw = sim.base_pose()
    assert 0.5 < x < 0.65
    assert abs(y) < 0.01 and abs(yaw) < 0.01


def test_rotates_in_place():
    sim = BumperbotSim(world("empty"))
    sim.step(1.0)
    speed = 0.5 * WHEEL_SEPARATION / WHEEL_RADIUS
    sim.set_wheel_velocities(-speed, speed)
    sim.step(2.0)
    x, y, yaw = sim.base_pose()
    assert 1.6 < yaw < 2.1
    assert math.hypot(x, y) < 0.01


@pytest.mark.parametrize("name", ["small_house", "small_warehouse"])
def test_lidar_sees_the_world_but_not_the_robot(name):
    sim = BumperbotSim(world(name))
    sim.step(0.5)
    ranges = sim.scan(0.0, 6.28 / 359, 360, 0.12, 12.0)
    assert np.isfinite(ranges).sum() > 100
    assert np.nanmin(ranges[np.isfinite(ranges)]) > 0.12


def test_lidar_sees_nothing_in_empty_world():
    sim = BumperbotSim(world("empty"))
    sim.step(0.5)
    assert not np.isfinite(sim.scan(0.0, 6.28 / 359, 360, 0.12, 12.0)).any()
