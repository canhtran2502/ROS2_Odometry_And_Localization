"""AC10: MuJoCo wheel odometry must agree with the ground truth (caster/wheel contact model pin)."""
import math
import os

import pytest

pytest.importorskip("mujoco")
import mujoco  # noqa: E402

from bumperbot_mujoco.sim_core import BumperbotSim  # noqa: E402

MJCF = os.path.join(os.path.dirname(__file__), "..", "mjcf", "empty.xml")
R = 0.033
L = 0.17
DT = 0.01          # 100 Hz control / encoder rate
A_LIN = 0.7        # m/s^2, controller limit
A_ANG = 5.0        # rad/s^2, controller limit


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Rig:
    """Drives the sim with ramped (v, w) commands and integrates encoder odometry (midpoint)."""

    def __init__(self):
        self.sim = BumperbotSim(MJCF)
        self.sim.step(1.0)
        self.x0, self.y0, self.yaw0 = self.sim.base_pose()
        self.ox = self.oy = self.oyaw = 0.0
        self.prev = self.sim.wheel_positions()
        self.v = 0.0
        self.w = 0.0

    def tick(self, v_cmd, w_cmd):
        self.v += max(-A_LIN * DT, min(A_LIN * DT, v_cmd - self.v))
        self.w += max(-A_ANG * DT, min(A_ANG * DT, w_cmd - self.w))
        self.sim.set_wheel_velocities((self.v - self.w * L / 2) / R, (self.v + self.w * L / 2) / R)
        self.sim.step(DT)
        cur = self.sim.wheel_positions()
        dl, dr = (cur[0] - self.prev[0]) * R, (cur[1] - self.prev[1]) * R
        self.prev = cur
        ds, dth = (dl + dr) / 2.0, (dr - dl) / L
        self.ox += ds * math.cos(self.oyaw + dth / 2)
        self.oy += ds * math.sin(self.oyaw + dth / 2)
        self.oyaw += dth

    def run(self, v, w, duration):
        for _ in range(int(round(duration / DT))):
            self.tick(v, w)

    def stop(self, duration=1.0):
        # Ramp to zero, then hold for the pause
        self.run(0.0, 0.0, duration)

    def errors(self):
        x, y, yaw = self.sim.base_pose()
        tx, ty = x - self.x0, y - self.y0
        c, s = math.cos(-self.yaw0), math.sin(-self.yaw0)
        tx, ty = c * tx - s * ty, s * tx + c * ty
        heading = math.degrees(abs(wrap(self.oyaw - (yaw - self.yaw0))))
        return heading, math.hypot(self.ox - tx, self.oy - ty)


def test_square_2m_left_turns():
    rig = Rig()
    for _ in range(4):
        rig.run(0.15, 0.0, 2.0 / 0.15)
        rig.stop(1.0)
        rig.run(0.0, 0.8, (math.pi / 2) / 0.8)
        rig.stop(1.0)
    heading, pos = rig.errors()
    print(f"\nsquare: heading error {heading:.2f} deg, position error {pos:.3f} m")
    assert heading <= 1.5
    assert pos <= 0.05


@pytest.mark.parametrize("w", [0.8, -0.8])
def test_four_turns_in_place(w):
    rig = Rig()
    rig.run(0.0, w, 4 * 2 * math.pi / abs(w))
    rig.stop(1.0)
    heading, _ = rig.errors()
    print(f"\n4 turns at w={w}: heading error {heading:.2f} deg")
    assert heading <= 5.0


def test_straight_3m():
    rig = Rig()
    rig.run(0.15, 0.0, 3.0 / 0.15)
    rig.stop(1.0)
    _, pos = rig.errors()
    print(f"\nstraight 3 m: position error {pos:.4f} m")
    assert pos <= 0.03


def test_caster_contacts_are_frictionless():
    sim = BumperbotSim(MJCF)
    sim.step(2.0)
    model, data = sim.model, sim.data
    ids = {mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n)
           for n in ("caster_front_link_collision", "caster_rear_link_collision")}
    assert -1 not in ids
    found = 0
    for i in range(data.ncon):
        c = data.contact[i]
        if c.geom1 in ids or c.geom2 in ids:
            found += 1
            assert c.dim == 1, f"caster contact {i} has dim {c.dim}, expected 1"
    assert found >= 1, "no caster contact found after settling"
