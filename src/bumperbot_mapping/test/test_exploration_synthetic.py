"""AC4: closed-loop kinematic test of ExplorerCore on a synthetic 2-room + corridor world."""
import math

import numpy as np
import pytest
from scipy import ndimage

from bumperbot_mapping import frontier_exploration as fe

RES = 0.05
W, H = 170, 130            # 8.5 m x 6.5 m
LASER_OFFSET = math.pi
N_BEAMS = 360
RANGE_MAX = 12.0
RANGE_MIN = 0.12
DT = 0.1
MAP_PERIOD = 1.0
SIM_LIMIT = 900.0


def build_world():
    """True grid: 100 occupied, 0 free. Rooms joined by a 0.8 m wide corridor."""
    world = np.full((H, W), 100, dtype=np.int8)
    world[10:110, 10:65] = 0      # room A  (2.75 m x 5.0 m)
    world[10:110, 105:160] = 0    # room B
    world[52:68, 65:105] = 0      # corridor, 16 cells = 0.8 m wide doorways
    # a pillar in room B so that exploration is not trivial
    world[55:63, 130:138] = 100
    return world


def raycast(world, x, y, angle, max_range):
    """Return range to the first occupied cell along the ray (DDA-like fixed-step march)."""
    step = RES * 0.25
    n = int(max_range / step)
    c, s = math.cos(angle), math.sin(angle)
    for i in range(1, n + 1):
        d = i * step
        cx = int((x + c * d) / RES)
        cy = int((y + s * d) / RES)
        if cx < 0 or cy < 0 or cx >= W or cy >= H or world[cy, cx] >= 50:
            return d
    return math.inf


class Mapper:
    """Accumulates scans into an occupancy grid (-1 unknown, 0 free, 100 occupied)."""

    def __init__(self):
        self.grid = np.full((H, W), -1, dtype=np.int8)

    def integrate(self, pose, ranges):
        x, y, yaw = pose
        inc = 2 * math.pi / N_BEAMS
        for i, r in enumerate(ranges):
            a = yaw + LASER_OFFSET + i * inc
            hit = math.isfinite(r)
            d_end = r if hit else RANGE_MAX
            step = RES * 0.5
            n = int(d_end / step)
            c, s = math.cos(a), math.sin(a)
            for k in range(n):
                d = k * step
                cx, cy = int((x + c * d) / RES), int((y + s * d) / RES)
                if 0 <= cx < W and 0 <= cy < H and self.grid[cy, cx] != 100:
                    self.grid[cy, cx] = 0
            if hit:
                cx, cy = int((x + c * r) / RES), int((y + s * r) / RES)
                if 0 <= cx < W and 0 <= cy < H:
                    self.grid[cy, cx] = 100


def lidar(world, pose):
    x, y, yaw = pose
    inc = 2 * math.pi / N_BEAMS
    out = np.full(N_BEAMS, math.inf)
    for i in range(N_BEAMS):
        r = raycast(world, x, y, yaw + LASER_OFFSET + i * inc, RANGE_MAX)
        if RANGE_MIN <= r <= RANGE_MAX:
            out[i] = r
    return out


def test_ac4_closed_loop_exploration():
    world = build_world()
    info = fe.GridInfo(RES, 0.0, 0.0, W, H)
    start = (1.0, 3.0, 0.0)
    x, y, yaw = start
    mapper = Mapper()
    core = fe.ExplorerCore({}, LASER_OFFSET)
    inc = 2 * math.pi / N_BEAMS

    t = 0.0
    next_map = 0.0
    cmd = None
    min_range = math.inf
    collided = False
    while t < SIM_LIMIT:
        ranges = lidar(world, (x, y, yaw))
        fin = ranges[np.isfinite(ranges)]
        if fin.size:
            min_range = min(min_range, float(fin.min()))
        if t >= next_map:
            mapper.integrate((x, y, yaw), ranges)
            core.update_map(mapper.grid.copy(), info, t)
            next_map += MAP_PERIOD
        cmd = core.step((x, y, yaw), fe.ScanData(ranges, 0.0, inc), t)
        if cmd.state == fe.COMPLETE:
            break
        x += cmd.v * math.cos(yaw) * DT
        y += cmd.v * math.sin(yaw) * DT
        yaw = fe.wrap_angle(yaw + cmd.w * DT)
        cx, cy = int(x / RES), int(y / RES)
        if world[cy, cx] >= 50:
            collided = True
            break
        t += DT

    # Coverage of the free cells reachable from the start
    free = world == 0
    labels, _ = ndimage.label(free)
    reach = labels == labels[int(start[1] / RES), int(start[0] / RES)]
    # final map snapshot
    mapper.integrate((x, y, yaw), lidar(world, (x, y, yaw)))
    coverage = float(((mapper.grid == 0) & reach).sum()) / float(reach.sum())
    print(f"\nAC4: state={cmd.state} sim_time={t:.1f}s coverage={coverage:.3f} "
          f"collided={collided} min_range={min_range:.3f} final_cmd=({cmd.v}, {cmd.w})")

    assert not collided, "robot entered an occupied cell"
    assert cmd.state == fe.COMPLETE, f"not COMPLETE after {t:.0f} s (state {cmd.state})"
    assert t <= SIM_LIMIT
    assert coverage >= 0.95, f"coverage {coverage:.3f}"
    assert (cmd.v, cmd.w) == (0.0, 0.0)
    # Hold: further steps stay exactly zero
    again = core.step((x, y, yaw), fe.ScanData(lidar(world, (x, y, yaw)), 0.0, inc), t + 1.0)
    assert (again.v, again.w) == (0.0, 0.0) and again.state == fe.COMPLETE
