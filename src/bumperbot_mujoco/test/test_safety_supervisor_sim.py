"""
AC4 and AC16: the safety logic in closed loop with the headless MuJoCo simulation (no ROS).

BumperbotSim on small_house.xml runs the Python SafetyCore. Scans run at 5 Hz through
scan_to_points with the MuJoCo laser pose, the tick runs at 20 Hz, the odometry comes from the
wheel velocities and the diff-drive inverse kinematics uses r = 0.033, b = 0.17. Virtual
obstacles are injected into the point list of a scan (the real LiDAR cannot see below 0.12 m
from the laser, so a 0.01 m contact point cannot be injected into the ranges).
"""
import math
import os
import re
import sys

import mujoco
import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
try:
    import bumperbot_utils.safety_logic  # noqa: F401
except ImportError:  # not installed: use the source tree of the sibling package
    sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..", "..", "bumperbot_utils")))

from bumperbot_mujoco.sim_core import BumperbotSim  # noqa: E402
from bumperbot_utils import safety_logic as sl  # noqa: E402

WORLD = os.path.join(HERE, "..", "mjcf", "small_house.xml")
URDF = os.path.join(HERE, "..", "..", "bumperbot_description", "urdf", "bumperbot.urdf.xacro")
R = 0.033
B = 0.17
TICK = 0.05                  # 20 Hz
SCAN_EVERY = 4               # 5 Hz
BEAMS = 360
CFG = sl.Config()
POLY = sl.footprint_polygon(CFG.footprint)
X_FRONT, X_REAR, HALF_WIDTH = sl.footprint_extents(POLY)


# ------------------------------------------------------------------ helpers
def place(sim, x, y, yaw):
    sim._set_base_pose(x, y, 0.001, yaw)
    mujoco.mj_forward(sim.model, sim.data)


def laser_pose(sim):
    """Pose of laser_site in base_footprint: (x, y, yaw)."""
    body = sim._id(mujoco.mjtObj.mjOBJ_BODY, "base_footprint")
    origin = np.array(sim.data.xpos[body])
    rot = sim.data.xmat[body].reshape(3, 3)
    site = rot.T @ (np.array(sim.data.site_xpos[sim._laser_site]) - origin)
    site_rot = rot.T @ sim.data.site_xmat[sim._laser_site].reshape(3, 3)
    return float(site[0]), float(site[1]), math.atan2(site_rot[1, 0], site_rot[0, 0])


def ground_truth_clearance(sim, direction):
    """Front (+1) or rear (-1) clearance from the footprint edge, noise-free dense scan."""
    n = 1440
    ranges = sim.scan(0.0, 2.0 * math.pi / n, n, 0.0, 12.0)
    x, y, yaw = laser_pose(sim)
    points, _ = sl.scan_to_points(list(ranges), 0.0, 2.0 * math.pi / n, 0.0, 12.0, x, y, yaw)
    best = math.inf
    for px, py in points:
        if abs(py) <= HALF_WIDTH + CFG.lateral_margin:
            if direction > 0 and px >= X_FRONT:
                best = min(best, px - X_FRONT)
            if direction < 0 and px <= X_REAR:
                best = min(best, X_REAR - px)
    return best


class Loop:
    """BumperbotSim plus SafetyCore at 20 Hz, scans at 5 Hz, wheel odometry."""

    def __init__(self, x, y, yaw, cfg=None):
        self.cfg = cfg or sl.Config()
        self.sim = BumperbotSim(WORLD, x=x, y=y, yaw=yaw)
        self.sim.step(0.5)
        self.core = sl.SafetyCore(self.cfg)
        self.tf = laser_pose(self.sim)
        self.tick_count = 0
        self.t_scan = None
        self.evaluation = sl.Evaluation()
        self.scan_ok = False
        self.feed_scans = True
        self.cmd = (0.0, 0.0)
        self.virtual = None            # ("robot", dx) or ("world", (wx, wy)) point to inject
        self.out = sl.Outputs()
        self.applied = (0.0, 0.0)
        self.history = []              # (t, applied v, locked, state, true front clearance)
        self.lock_seen = False
        self.speed = 0.0

    @property
    def now(self):
        return self.sim.time

    def pose(self):
        return self.sim.base_pose()

    def odom_speed(self):
        left, right = self.sim.wheel_velocities()
        return R * (left + right) / 2.0

    def virtual_point(self):
        if self.virtual is None:
            return None
        kind, value = self.virtual
        if kind == "robot":
            return (X_FRONT + value, 0.0)
        wx, wy = value
        x, y, yaw = self.pose()
        dx, dy = wx - x, wy - y
        return (math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy)

    def scan(self):
        ranges = self.sim.scan(0.0, 2.0 * math.pi / BEAMS, BEAMS, 0.12, 12.0)
        points, ratio = sl.scan_to_points(
            list(ranges), 0.0, 2.0 * math.pi / BEAMS, 0.12, 12.0, *self.tf)
        extra = self.virtual_point()
        if extra is not None:
            points.append(extra)
        self.evaluation = sl.evaluate_points(points, self.cfg)
        self.scan_ok = ratio <= self.cfg.max_invalid_beam_ratio
        self.t_scan = self.now

    def tick(self):
        if self.feed_scans and self.tick_count % SCAN_EVERY == 0:
            self.scan()
        inp = sl.Inputs()
        inp.scan_ok = self.scan_ok
        inp.scan_age = -1.0 if self.t_scan is None else self.now - self.t_scan
        inp.tf_ok = True
        inp.tf_description = "base_footprint <- laser_link"
        inp.evaluation = self.evaluation
        inp.odom_age = 0.0
        inp.v_odom = self.odom_speed()
        inp.cmd_age = 0.0
        inp.v_cmd = self.cmd[0]
        self.out = self.core.update(self.now, inp)
        v, w = self.core.limit_command(self.cmd[0], self.cmd[1], self.out)
        self.applied = (v, w)
        self.lock_seen = self.lock_seen or self.out.locked
        self.sim.set_wheel_velocities((v - w * B / 2.0) / R, (v + w * B / 2.0) / R)
        self.sim.step(TICK)
        self.tick_count += 1
        self.speed = self.odom_speed()
        self.history.append((self.now, v, self.out.locked, self.out.state))

    def run(self, seconds, stop=None):
        end = self.now + seconds
        while self.now < end - 1e-9:
            self.tick()
            if stop is not None and stop(self):
                return True
        return False

    def travelled_since(self, start):
        x, y, _ = self.pose()
        return math.hypot(x - start[0], y - start[1])


# ------------------------------------------------------------------ AC4
def collision_geoms(model):
    root = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_footprint")
    geoms = []
    for g in range(model.ngeom):
        body = model.geom_bodyid[g]
        while body not in (0, root):
            body = model.body_parentid[body]
        if body != root:
            continue
        if model.geom_contype[g] == 0 and model.geom_conaffinity[g] == 0:
            continue
        geoms.append(g)
    return geoms


def geom_aabb_xy_in_base(sim, g):
    """
    Axis-aligned box (in base_footprint) of a collision geom as its four xy corners.

    Meshes use their real vertices (the MuJoCo geom_aabb is expressed in the rotated principal
    axis frame of the mesh and would overstate the extent), spheres use their radius and other
    primitives the corners of the geom box.
    """
    model, data = sim.model, sim.data
    body = sim._id(mujoco.mjtObj.mjOBJ_BODY, "base_footprint")
    origin = np.array(data.xpos[body])
    rot = data.xmat[body].reshape(3, 3)
    geom_rot = data.geom_xmat[g].reshape(3, 3)
    kind = model.geom_type[g]
    if kind == mujoco.mjtGeom.mjGEOM_MESH:
        mid = model.geom_dataid[g]
        start = model.mesh_vertadr[mid]
        verts = model.mesh_vert[start:start + model.mesh_vertnum[mid]]
        world = data.geom_xpos[g] + verts @ geom_rot.T
    elif kind == mujoco.mjtGeom.mjGEOM_SPHERE:
        radius = model.geom_size[g][0]
        world = np.array([data.geom_xpos[g] + radius * np.array([sx, sy, 0.0])
                          for sx in (-1, 1) for sy in (-1, 1)])
    else:
        center, half = model.geom_aabb[g][:3], model.geom_aabb[g][3:]
        world = np.array([data.geom_xpos[g] + geom_rot @ (center + np.array([sx, sy, sz]) * half)
                          for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    local = (world - origin) @ rot
    lo, hi = local[:, :2].min(axis=0), local[:, :2].max(axis=0)
    return [np.array([x, y]) for x in (lo[0], hi[0]) for y in (lo[1], hi[1])]


_SIM = BumperbotSim(WORLD)
GEOMS = [(_SIM.model.geom(g).name or str(g), g) for g in collision_geoms(_SIM.model)]


def test_collision_geoms_were_found():
    names = [n for n, _ in GEOMS]
    assert "base_link_collision" in names and "wheel_left_link_collision" in names, names


@pytest.mark.parametrize("name,geom", GEOMS, ids=[n for n, _ in GEOMS])
def test_footprint_contains_the_geom_aabb(name, geom):
    """AC4: the default footprint contains the xy projection of every collision geom AABB."""
    sim = BumperbotSim(WORLD)
    sim.step(0.2)
    place(sim, 0.0, 0.0, 0.0)
    corners = geom_aabb_xy_in_base(sim, geom)
    xs = [c[0] for c in corners]
    ys = [c[1] for c in corners]
    outside = [(round(float(c[0]), 4), round(float(c[1]), 4)) for c in corners
               if not sl.point_in_polygon(float(c[0]), float(c[1]), POLY)]
    assert not outside, (
        "{}: AABB x [{:.4f}, {:.4f}] y [{:.4f}, {:.4f}] exceeds the footprint "
        "x [{:.2f}, {:.2f}] y +-{:.2f}; corners outside {}".format(
            name, min(xs), max(xs), min(ys), max(ys), X_REAR, X_FRONT, HALF_WIDTH, outside))


def urdf_laser_joint():
    with open(URDF) as f:
        text = f.read()
    block = re.search(r'<joint name="laser_joint".*?</joint>', text, re.S).group(0)
    xyz = [float(v) for v in re.search(r'xyz="([^"]+)"', block).group(1).split()]
    rpy = [float(v) for v in re.search(r'rpy="([^"]+)"', block).group(1).split()]
    return xyz, rpy


def test_laser_site_matches_the_urdf_laser_joint():
    """AC4: MuJoCo laser pose equals the URDF laser_joint (xy 0.01 m, yaw 0.01 rad)."""
    sim = BumperbotSim(WORLD)
    place(sim, 0.0, 0.0, 0.0)
    x, y, yaw = laser_pose(sim)
    xyz, rpy = urdf_laser_joint()
    assert abs(x - xyz[0]) <= 0.01 and abs(y - xyz[1]) <= 0.01, (x, y, xyz)
    diff = math.atan2(math.sin(yaw - rpy[2]), math.cos(yaw - rpy[2]))
    assert abs(diff) <= 0.01, (yaw, rpy)


def test_mock_and_sim_scan_to_points_agree():
    """AC4: scan_to_points with the URDF pose and with the MuJoCo pose give the same points."""
    sim = BumperbotSim(WORLD)
    place(sim, 1.0, 1.0, 3.14)
    n = 360
    ranges = sim.scan(0.0, 2.0 * math.pi / n, n, 0.12, 12.0)
    near = [r if r < 3.0 else math.inf for r in ranges]
    assert sum(1 for r in near if math.isfinite(r)) > 20
    xyz, rpy = urdf_laser_joint()
    a, _ = sl.scan_to_points(near, 0.0, 2.0 * math.pi / n, 0.12, 12.0, *laser_pose(sim))
    b, _ = sl.scan_to_points(near, 0.0, 2.0 * math.pi / n, 0.12, 12.0, xyz[0], xyz[1], rpy[2])
    assert len(a) == len(b) > 20
    worst = max(math.hypot(p[0] - q[0], p[1] - q[1]) for p, q in zip(a, b))
    assert worst < 0.01, worst


# ------------------------------------------------------------------ AC16
def test_a_approach_a_wall_halts_without_contact():
    """AC16 (a): 0.3 m/s toward a wall: halt, clearance >= stop_margin - 0.02, monotonic speed."""
    loop = Loop(1.0, 1.0, 3.14)          # wall about 3.3 m ahead
    loop.cmd = (0.3, 0.0)
    clearances = []
    speeds = []
    previous = math.inf
    min_clearance = math.inf
    while loop.now < 40.0:
        loop.tick()
        clearance = ground_truth_clearance(loop.sim, +1)
        clearances.append(clearance)
        speeds.append(loop.applied[0])
        min_clearance = min(min_clearance, clearance)
        if clearance < previous - 1e-9:
            pass
        previous = clearance
        if loop.now > 2.0 and abs(loop.speed) < 0.005 and loop.applied[0] == 0.0:
            break
    loop.run(1.0)
    assert abs(loop.speed) < 0.01, "robot still moving: {}".format(loop.speed)
    assert loop.applied[0] == 0.0
    assert min_clearance > 0.0, "contact with the wall"
    assert min_clearance >= CFG.stop_margin - 0.02, "min clearance {:.4f}".format(min_clearance)
    # output speed is non-increasing while the clearance decreases
    last_speed = speeds[0]
    last_clear = clearances[0]
    for speed, clear in zip(speeds, clearances):
        if clear < last_clear - 1e-6:
            assert speed <= last_speed + 1e-6, "speed rose from {} to {} at clearance {:.3f}".format(
                last_speed, speed, clear)
        last_speed, last_clear = speed, clear
    assert min(clearances) < 0.2
    print("minimum ground-truth clearance {:.4f} m".format(min_clearance))


def test_b_virtual_point_ahead_stops_the_robot():
    """AC16 (b): a point 0.04 m ahead at 0.3 m/s: speed < 0.02 within 1 s, travel <= 0.20 m."""
    loop = Loop(0.0, 0.0, 0.0)           # open space ahead
    loop.cmd = (0.3, 0.0)
    loop.run(3.0)
    assert abs(loop.speed - 0.3) < 0.05, loop.speed
    assert not loop.lock_seen, "locked in free space"
    start = loop.pose()
    t0 = loop.now
    x, y, yaw = start
    ahead = X_FRONT + 0.04
    loop.virtual = ("world", (x + ahead * math.cos(yaw), y + ahead * math.sin(yaw)))
    loop.run(1.0)
    assert abs(loop.speed) < 0.02, "speed {} 1 s after the injection".format(loop.speed)
    assert loop.travelled_since(start) <= 0.20, "travel {:.3f} m".format(
        loop.travelled_since(start))
    assert any(locked for t, _, locked, _ in loop.history if t >= t0), "no brake lock"
    assert loop.out.front_zone in ("STOP", "CONTACT")


def test_c_reverse_into_a_wall_then_forward_is_free():
    """AC16 (c): rear STOP does not block forward motion (>= 0.10 m within 1.5 s)."""
    loop = Loop(-2.0, -1.0, 3.14)        # wall about 1.0 m behind
    loop.cmd = (-0.3, 0.0)
    reached = loop.run(25.0, stop=lambda l: l.out.rear_zone in ("STOP", "CONTACT") and
                       abs(l.speed) < 0.01 and not l.out.braking)
    assert reached, "rear STOP not reached: {} {}".format(loop.out, loop.pose())
    assert loop.out.state == "ACTIVE", loop.out
    assert ground_truth_clearance(loop.sim, -1) > 0.0
    loop.cmd = (0.2, 0.0)
    start = loop.pose()
    loop.run(1.5)
    assert loop.travelled_since(start) >= 0.10, "moved {:.3f} m".format(
        loop.travelled_since(start))
    assert loop.out.state == "ACTIVE"


def test_d_scan_dropout_while_moving_faults_and_stops():
    """AC16 (d): scans stop at 0.3 m/s: FAULT and zero command within scan_timeout + 0.1 s."""
    loop = Loop(0.0, 0.0, 0.0)
    loop.cmd = (0.3, 0.0)
    loop.run(3.0)
    assert abs(loop.speed - 0.3) < 0.05, loop.speed
    loop.feed_scans = False
    t_last_scan = loop.t_scan
    ok = loop.run(2.0, stop=lambda l: l.out.state == "FAULT")
    assert ok, "no FAULT"
    assert loop.now - t_last_scan <= CFG.scan_timeout + 0.1 + 1e-9, loop.now - t_last_scan
    assert loop.out.locked and loop.applied == (0.0, 0.0)
    t_fault = loop.now
    loop.run(1.0)
    assert abs(loop.speed) < 0.02, "speed {} 1 s after the FAULT".format(loop.speed)
    assert loop.now - t_fault >= 1.0 - 1e-9


def test_e_contact_latch_and_reset():
    """AC16 (e): a point at 0.01 m while moving latches ESTOP; reset refused until removed."""
    loop = Loop(0.0, 0.0, 0.0)
    loop.cmd = (0.3, 0.0)
    loop.run(3.0)
    loop.virtual = ("robot", 0.01)
    ok = loop.run(1.0, stop=lambda l: l.out.state == "ESTOP")
    assert ok, "no ESTOP: {}".format(loop.out)
    assert loop.out.estop_latched and loop.out.locked
    loop.cmd = (0.0, 0.0)
    loop.run(1.0)
    assert abs(loop.speed) < 0.02
    accepted, message = loop.core.request_reset()
    assert (accepted, message) == (False, "obstacle in contact zone (front)")
    loop.virtual = None
    loop.run(0.5)
    assert loop.core.request_reset() == (True, "reset")
    loop.run(0.5)
    assert loop.out.state == "ACTIVE" and not loop.out.estop_latched
