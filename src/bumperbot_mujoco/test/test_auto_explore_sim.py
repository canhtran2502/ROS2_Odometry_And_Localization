"""AC5: headless MuJoCo end-to-end test of the exploration core on small_house."""
import math
import os
import time

import numpy as np
import pytest
from scipy import ndimage

pytest.importorskip("mujoco")

from bumperbot_mujoco.sim_core import BumperbotSim  # noqa: E402
from bumperbot_mapping import frontier_exploration as fe  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
MJCF = os.path.join(HERE, "..", "mjcf", "small_house.xml")
WHEEL_R = 0.033
WHEEL_L = 0.17
RES = 0.05
SIZE = 500
ORIGIN = -12.5
SIM_LIMIT = 1800.0
WALL_LIMIT = 180.0
LOCK_DISTANCE = 0.2   # safety_stop_node danger_distance (beam <= this locks twist_mux)
CTRL_DT = 0.1
SCAN_PERIOD = 0.2     # mujoco_sim_node publishes /scan at 5 Hz
MAP_PERIOD = 1.0
N_BEAMS = 360
RANGE_MIN, RANGE_MAX, NOISE = 0.12, 12.0, 0.01
INC = (6.28 - 0.0) / (N_BEAMS - 1)


def find_reference_map():
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory("bumperbot_mapping"),
                            "maps", "small_house", "map.pgm")
        if os.path.exists(path):
            return path
    except Exception:  # noqa: BLE001
        pass
    path = os.path.join(HERE, "..", "..", "bumperbot_mapping", "maps", "small_house", "map.pgm")
    return path if os.path.exists(path) else None


def read_pgm(path):
    with open(path, "rb") as f:
        data = f.read()
    tokens = []
    pos = 0
    while len(tokens) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b"#":
            while data[pos:pos + 1] != b"\n":
                pos += 1
            continue
        start = pos
        while not data[pos:pos + 1].isspace():
            pos += 1
        tokens.append(data[start:pos])
    pos += 1
    assert tokens[0] == b"P5"
    w, h, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    img = np.frombuffer(data, dtype=np.uint8, count=w * h, offset=pos).reshape(h, w)
    return img


class LogOddsMapper:
    def __init__(self):
        self.l = np.zeros((SIZE, SIZE), dtype=np.float32)

    def integrate(self, pose, ranges, laser_offset):
        x, y, yaw = pose
        for i, r in enumerate(ranges):
            hit = math.isfinite(r)
            d_end = r if hit else RANGE_MAX
            a = yaw + laser_offset + i * INC
            c, s = math.cos(a), math.sin(a)
            n = int(d_end / (RES * 0.5))
            seen = set()
            for k in range(n):
                d = k * RES * 0.5
                cx = int(math.floor((x + c * d - ORIGIN) / RES))
                cy = int(math.floor((y + s * d - ORIGIN) / RES))
                if (cx, cy) in seen or not (0 <= cx < SIZE and 0 <= cy < SIZE):
                    continue
                seen.add((cx, cy))
                self.l[cy, cx] = max(self.l[cy, cx] - 0.4, -5.0)
            if hit:
                cx = int(math.floor((x + c * r - ORIGIN) / RES))
                cy = int(math.floor((y + s * r - ORIGIN) / RES))
                if 0 <= cx < SIZE and 0 <= cy < SIZE:
                    self.l[cy, cx] = min(self.l[cy, cx] + 0.85, 5.0)

    def grid(self):
        g = np.full((SIZE, SIZE), -1, dtype=np.int8)
        g[self.l < -0.3] = 0
        g[self.l > 0.5] = 100
        return g


def laser_yaw_offset(sim):
    import mujoco
    site = sim.data.site_xmat[sim._laser_site].reshape(3, 3)
    body = sim.model.body("base_footprint").id
    rel = sim.data.xmat[body].reshape(3, 3).T @ site
    return math.atan2(rel[1, 0], rel[0, 0])


def test_ac5_mujoco_exploration_small_house():
    ref_path = find_reference_map()
    assert ref_path is not None, "reference map small_house/map.pgm not found"
    img = read_pgm(ref_path)
    assert img.shape == (SIZE, SIZE)
    # PGM row 0 is the top of the map (max y); flip to grid layout (row 0 = min y)
    ref_free = np.flipud(img > 205)  # trinary map: free = 254

    sim = BumperbotSim(MJCF, seed=1)
    sim.step(0.5)
    offset = laser_yaw_offset(sim)
    print(f"\nAC5: laser yaw offset from model = {offset:.4f}")
    assert abs(abs(offset) - math.pi) < 0.05

    info = fe.GridInfo(RES, ORIGIN, ORIGIN, SIZE, SIZE)
    core = fe.ExplorerCore({}, offset)
    mapper = LogOddsMapper()

    wall_start = time.time()
    scan = None
    next_scan = 0.0
    next_map = 0.0
    lock = False
    lock_since = None
    max_lock = 0.0
    lock_events = 0
    lock_log = []
    min_range = math.inf
    cmd = None
    max_step_wall = 0.0

    while sim.time < SIM_LIMIT:
        t = sim.time
        if t >= next_scan:
            scan = sim.scan(0.0, INC, N_BEAMS, RANGE_MIN, RANGE_MAX, NOISE)
            next_scan += SCAN_PERIOD
            fin = scan[np.isfinite(scan)]
            if fin.size:
                min_range = min(min_range, float(fin.min()))
            now_lock = bool(fin.size and fin.min() <= LOCK_DISTANCE)
            if now_lock and not lock:
                lock_events += 1
                lock_since = t
                k = int(np.nanargmin(np.where(np.isfinite(scan), scan, np.nan)))
                base_ang = fe.wrap_angle(k * INC + offset)
                lock_log.append((round(t, 1), tuple(round(v, 2) for v in sim.base_pose()),
                                 round(float(scan[k]), 3), round(float(base_ang), 2)))
            if lock and not now_lock:
                max_lock = max(max_lock, t - lock_since)
            lock = now_lock
        pose = sim.base_pose()
        if t >= next_map and scan is not None:
            mapper.integrate(pose, scan, offset)
            core.update_map(mapper.grid(), info, t)
            next_map += MAP_PERIOD
        t0 = time.time()
        cmd = core.step(pose, fe.ScanData(scan, 0.0, INC) if scan is not None else None, t)
        max_step_wall = max(max_step_wall, time.time() - t0)
        if cmd.state == fe.COMPLETE:
            break
        v, w = (0.0, 0.0) if lock else (cmd.v, cmd.w)
        if lock:
            max_lock = max(max_lock, t - lock_since)
        sim.set_wheel_velocities((v - w * WHEEL_L / 2) / WHEEL_R, (v + w * WHEEL_L / 2) / WHEEL_R)
        sim.step(CTRL_DT)
        if time.time() - wall_start > WALL_LIMIT:
            break

    wall = time.time() - wall_start
    # Coverage against the reference map
    start_cell = (int((0.0 - ORIGIN) / RES), int((0.0 - ORIGIN) / RES))
    labels, _ = ndimage.label(ref_free)
    assert labels[start_cell[1], start_cell[0]] > 0, "start cell is not free in the reference"
    ref_reach = labels == labels[start_cell[1], start_cell[0]]
    mine_free = mapper.grid() == 0
    tol = ndimage.binary_dilation(mine_free, structure=np.ones((3, 3), bool), iterations=2)
    coverage = float((tol & ref_reach).sum()) / float(ref_reach.sum())
    print(f"AC5: state={cmd.state} sim_time={sim.time:.1f}s wall={wall:.1f}s "
          f"coverage={coverage:.3f} min_range={min_range:.3f} max_lock_s={max_lock:.1f} "
          f"lock_events={lock_events} max_step_wall={max_step_wall:.3f}s")

    for entry in lock_log:
        print('AC5 lock start (t, pose, min_range, base_angle):', entry)
    assert cmd.state == fe.COMPLETE, f"not COMPLETE: {cmd.state} at sim {sim.time:.0f}s, wall {wall:.0f}s"
    assert sim.time <= SIM_LIMIT
    assert wall < WALL_LIMIT
    assert coverage >= 0.80, f"coverage {coverage:.3f}"
    assert min_range >= 0.15, f"min lidar range {min_range:.3f} m (collision)"
    assert max_lock <= 5.0, f"safety lock active for {max_lock:.1f} s in a row (deadlock)"
