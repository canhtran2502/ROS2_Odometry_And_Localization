"""Unit tests (AC1-AC3) of the pure-Python frontier exploration core."""
import math
import subprocess
import sys

import numpy as np
import pytest

from bumperbot_mapping import frontier_exploration as fe

U, F, O = -1, 0, 100


# ---------------------------------------------------------------------------
# AC1
# ---------------------------------------------------------------------------

def test_ac1_import_without_rclpy():
    code = ("import bumperbot_mapping.frontier_exploration, sys; "
            "assert 'rclpy' not in sys.modules")
    res = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True)
    assert res.returncode == 0, res.stderr


def test_ac1_detect_frontiers_exact():
    g = np.full((10, 10), U, dtype=np.int8)
    g[2:8, 2:8] = F
    g[2:8, 1] = O  # wall on the left, free cells there are not next to unknown
    mask = fe.detect_frontiers(g)
    # Brute-force reference of the definition
    expected = np.zeros_like(mask)
    for y in range(10):
        for x in range(10):
            if 0 <= g[y, x] < 50:
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < 10 and 0 <= ny < 10 and g[ny, nx] == U:
                        expected[y, x] = True
    assert np.array_equal(mask, expected)
    # Interior cells of the free block are not frontiers
    assert not mask[4, 4]
    # Diagonal-only unknown neighbour does not count
    g2 = np.zeros((3, 3), dtype=np.int8)
    g2[0, 0] = U
    m2 = fe.detect_frontiers(g2)
    assert not m2[1, 1] and m2[0, 1] and m2[1, 0]
    # Occupied cells are never frontiers
    assert not fe.detect_frontiers(np.array([[O, U]], dtype=np.int8)).any()


def test_ac1_detect_frontiers_edges_not_unknown():
    g = np.zeros((5, 5), dtype=np.int8)  # fully known: no frontier at the map edge
    assert not fe.detect_frontiers(g).any()


def test_ac1_cluster_counts_sizes_centroids():
    mask = np.zeros((20, 20), dtype=bool)
    mask[2, 2:12] = True            # horizontal line, 10 cells
    mask[10:20, 15] = True          # vertical line, 10 cells
    for i in range(9):              # diagonal, 8-connected, 9 cells
        mask[10 + i, 2 + i] = True
    mask[0, 19] = True              # tiny 1-cell cluster (dropped)
    cl = fe.cluster_frontiers(mask, 8)
    assert len(cl) == 3
    sizes = sorted(c.size for c in cl)
    assert sizes == [9, 10, 10]
    by_size = {}
    for c in cl:
        by_size.setdefault(c.size, []).append(c)
    horiz = [c for c in by_size[10] if c.centroid_cell[1] == 2][0]
    assert horiz.centroid_cell[1] == 2 and horiz.centroid_cell[0] in (6, 7)  # mean 6.5
    assert len(horiz.cells) == 10
    vert = [c for c in by_size[10] if c.centroid_cell[0] == 15][0]
    assert vert.centroid_cell[0] == 15 and vert.centroid_cell[1] in (14, 15)
    diag = by_size[9][0]
    assert diag.centroid_cell == (6, 14)


def test_ac1_cluster_min_size_drops_small():
    mask = np.zeros((10, 10), dtype=bool)
    mask[1, 1:5] = True   # 4 cells
    mask[5, 1:9] = True   # 8 cells
    assert len(fe.cluster_frontiers(mask, 8)) == 1
    assert len(fe.cluster_frontiers(mask, 4)) == 2
    assert len(fe.cluster_frontiers(mask, 9)) == 0
    assert fe.cluster_frontiers(np.zeros((4, 4), dtype=bool), 1) == []


def test_ac1_diagonal_connectivity_merges():
    mask = np.zeros((6, 6), dtype=bool)
    mask[0, 0] = mask[1, 1] = mask[2, 2] = True
    assert len(fe.cluster_frontiers(mask, 1)) == 1


# ---------------------------------------------------------------------------
# AC2
# ---------------------------------------------------------------------------

def two_rooms(door_rows=(17, 18, 19, 20, 21, 22, 23), width=60, height=40, wall_x=30):
    g = np.full((height, width), F, dtype=np.int8)
    g[0, :] = g[-1, :] = O
    g[:, 0] = g[:, -1] = O
    g[:, wall_x] = O
    for r in door_rows:
        g[r, wall_x] = F
    return g


def test_ac2_costmap_inflation_exact():
    g = np.full((30, 30), F, dtype=np.int8)
    g[15, 15] = O
    infl = 4.0
    trav = fe.build_costmap(g, infl)
    yy, xx = np.mgrid[0:30, 0:30]
    d = np.hypot(xx - 15, yy - 15)
    # every cell within the inflation radius is not traversable
    assert not trav[d <= infl].any()
    # cells clearly beyond are traversable
    assert trav[d > infl + 1e-6].all()


def test_ac2_costmap_unknown():
    g = np.full((10, 10), F, dtype=np.int8)
    g[5, 5] = U
    assert not fe.build_costmap(g, 0.0, unknown_is_obstacle=True)[5, 5]
    assert fe.build_costmap(g, 0.0, unknown_is_obstacle=False)[5, 5]
    g2 = np.full((10, 10), F, dtype=np.int8)
    assert fe.build_costmap(g2, 3.0).all()  # no obstacle: everything traversable


def test_ac2_astar_through_doorway():
    g = two_rooms()
    trav = fe.build_costmap(g, 3.0)  # doorway of 5 rows leaves a narrow gap
    assert trav[20, 30], "doorway centre must stay traversable"
    path = fe.astar(trav, (10, 20), (50, 20))
    assert path is not None
    assert path[0] == (10, 20) and path[-1] == (50, 20)
    _assert_valid_path(trav, path)
    # must cross the wall column through the doorway
    assert any(c[0] == 30 for c in path)


def _assert_valid_path(trav, path):
    for (x, y) in path:
        assert trav[y, x], f"cell {(x, y)} not traversable"
    for a, b in zip(path[:-1], path[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        assert max(abs(dx), abs(dy)) == 1, "path is not 8-connected"
        if dx != 0 and dy != 0:  # no corner cutting
            assert trav[a[1], a[0] + dx] and trav[a[1] + dy, a[0]], "corner cut"


def test_ac2_astar_no_corner_cutting():
    trav = np.ones((5, 5), dtype=bool)
    trav[1, 2] = False
    trav[2, 1] = False
    # diagonal squeeze between (1,2) and (2,1) is blocked
    path = fe.astar(trav, (1, 1), (2, 2))
    assert path is not None
    _assert_valid_path(trav, path)
    assert len(path) > 2


def test_ac2_astar_closed_room_none():
    g = two_rooms(door_rows=())
    trav = fe.build_costmap(g, 2.0)
    assert fe.astar(trav, (10, 20), (50, 20)) is None
    # blocked / out-of-bounds endpoints
    assert fe.astar(trav, (0, 0), (10, 20)) is None
    assert fe.astar(trav, (10, 20), (500, 500)) is None


def test_ac2_start_snap():
    g = np.full((30, 30), F, dtype=np.int8)
    g[:, 0] = g[:, -1] = g[0, :] = g[-1, :] = O
    trav = fe.build_costmap(g, 3.0)
    start = (2, 15)
    assert not trav[15, 2]
    snapped = fe.snap_to_traversable(trav, start, 7)
    assert snapped is not None and trav[snapped[1], snapped[0]]
    assert math.hypot(snapped[0] - 2, snapped[1] - 15) <= 7
    reach = fe.reachable_mask(trav, start)
    assert reach[snapped[1], snapped[0]] and reach.sum() > 100
    # Too far to snap
    assert fe.snap_to_traversable(np.zeros((10, 10), dtype=bool), (5, 5)) is None
    assert not fe.reachable_mask(np.zeros((10, 10), dtype=bool), (5, 5)).any()


def _frontier(cells):
    cells = np.array(cells)
    c = (int(round(cells[:, 0].mean())), int(round(cells[:, 1].mean())))
    return fe.Frontier(cells=cells, size=len(cells), centroid_cell=c)


def test_ac2_select_goal_blacklist_and_unreachable():
    g = two_rooms(door_rows=())  # closed: two disconnected rooms
    trav = fe.build_costmap(g, 2.0)
    robot = (10, 20)
    reach = fe.reachable_mask(trav, robot)
    fr_left = _frontier([(x, 10) for x in range(8, 16)])
    fr_right = _frontier([(x, 20) for x in range(45, 53)])
    # only an unreachable frontier -> None
    assert fe.select_goal([fr_right], trav, reach, robot, []) is None
    # reachable frontier selected, never the unreachable one
    goal, path = fe.select_goal([fr_right, fr_left], trav, reach, robot, [])
    assert reach[goal[1], goal[0]] and goal[0] < 30
    assert path[0] == robot or trav[path[0][1], path[0][0]]
    assert path[-1] == goal
    # blacklist the only reachable frontier region -> None
    res = fe.select_goal([fr_left], trav, reach, robot, [goal],
                         blacklist_radius=3.0, search_radius=0.5, resolution=0.05)
    assert res is None
    # with a blacklist only partially covering, the goal is never within the blacklist radius
    bl = [(12, 10)]
    res = fe.select_goal([fr_left], trav, reach, robot, bl, blacklist_radius=0.2,
                         resolution=0.05, search_radius=1.0)
    assert res is not None
    g2 = res[0]
    assert math.hypot(g2[0] - bl[0][0], g2[1] - bl[0][1]) > 0.2 / 0.05
    assert fe.select_goal([], trav, reach, robot, []) is None


def test_ac2_select_goal_prefers_nearer_and_path_valid():
    g = two_rooms()
    trav = fe.build_costmap(g, 2.0)
    robot = (10, 20)
    reach = fe.reachable_mask(trav, robot)
    near = _frontier([(x, 8) for x in range(8, 16)])
    far = _frontier([(x, 20) for x in range(45, 53)])
    goal, path = fe.select_goal([far, near], trav, reach, robot, [])
    _assert_valid_path(trav, path)
    assert goal[0] < 30


def test_ac2_simplify_path():
    info = fe.GridInfo(0.05, -1.0, -2.0, 100, 100)
    assert fe.simplify_path([], info) == []
    pts = fe.simplify_path([(i, 0) for i in range(21)], info, 0.1)
    assert pts[0] == info.cell_to_world(0, 0)
    assert pts[-1] == info.cell_to_world(20, 0)
    assert info.world_to_cell(*info.cell_to_world(7, 9)) == (7, 9)


# ---------------------------------------------------------------------------
# AC3
# ---------------------------------------------------------------------------

def test_ac3_pure_pursuit_limits():
    pp = fe.PurePursuit(0.15, 0.8, 0.35, 0.8, 0.25)
    rng = np.random.default_rng(1)
    for _ in range(500):
        pose = (rng.uniform(-2, 2), rng.uniform(-2, 2), rng.uniform(-math.pi, math.pi))
        path = [(rng.uniform(-3, 3), rng.uniform(-3, 3)) for _ in range(rng.integers(1, 6))]
        v, w, _ = pp.compute(pose, path)
        assert abs(v) <= 0.15 + 1e-9
        assert abs(w) <= 0.8 + 1e-9
        assert v >= 0.0


def test_ac3_pure_pursuit_rotate_in_place():
    pp = fe.PurePursuit()
    for err in (1.0, 2.0, 3.0, -1.0, -2.5):
        pose = (0.0, 0.0, -err)  # target along +x, heading error = err
        path = [(1.0, 0.0), (2.0, 0.0)]
        v, w, done = pp.compute(pose, path)
        assert v == 0.0 and w != 0.0 and not done
        assert math.copysign(1, w) == math.copysign(1, err)
    # small error: translates
    v, w, done = pp.compute((0.0, 0.0, 0.1), [(1.0, 0.0)])
    assert v > 0.0 and not done


def test_ac3_pure_pursuit_done():
    pp = fe.PurePursuit(goal_tolerance=0.25)
    assert pp.compute((0, 0, 0), [(0.2, 0.0)]) == (0.0, 0.0, True)
    assert pp.compute((0, 0, 0), [(1.0, 0.0)])[2] is False
    assert pp.compute((0, 0, 0), [])[2] is True


def _scan(front_range=None, rear_range=None, left=None, right=None, offset=0.0, n=360):
    """Scan in the laser frame; base-frame angle a corresponds to laser angle a - offset."""
    inc = 2 * math.pi / n
    r = np.full(n, 5.0)
    ang_base = fe.wrap_angle(0.0 + np.arange(n) * inc + offset)

    def setr(center, val, half=0.1):
        sel = np.abs(fe.wrap_angle(ang_base - center)) <= half
        r[sel] = val

    if front_range is not None:
        setr(0.0, front_range)
    if rear_range is not None:
        setr(math.pi, rear_range)
    if left is not None:
        setr(math.pi / 2, left)
    if right is not None:
        setr(-math.pi / 2, right)
    return r, 0.0, inc


def test_ac3_scan_safety_offset_pi_front_is_laser_pi():
    n = 360
    inc = 2 * math.pi / n
    r = np.full(n, 5.0)
    r[n // 2] = 0.1   # laser angle pi (index 180)
    scale, blocked, mn, esc = fe.scan_safety(r, 0.0, inc, math.pi, 0.25, 0.45, 0.6)
    assert scale == 0.0 and blocked and mn == pytest.approx(0.1)
    # the same beam with offset 0 is the robot's rear: not blocking
    scale0, blocked0, _, _ = fe.scan_safety(r, 0.0, inc, 0.0, 0.25, 0.45, 0.6)
    assert scale0 == 1.0 and not blocked0
    # the beam at laser angle 0 (index 0) is behind the robot with offset pi
    r2 = np.full(n, 5.0)
    r2[0] = 0.1
    scale2, blocked2, mn2, _ = fe.scan_safety(r2, 0.0, inc, math.pi, 0.25, 0.45, 0.6)
    assert scale2 == 1.0 and not blocked2 and mn2 == pytest.approx(0.1)


def test_ac3_scan_safety_scaling():
    for offset in (0.0, math.pi, 3.14):
        r, a0, inc = _scan(front_range=0.35, offset=offset)
        scale, blocked, _, _ = fe.scan_safety(r, a0, inc, offset, 0.25, 0.45, 0.6)
        assert 0.0 < scale < 1.0 and not blocked
        assert scale == pytest.approx(0.5, abs=0.15)
        r, a0, inc = _scan(front_range=0.2, offset=offset)
        scale, blocked, _, _ = fe.scan_safety(r, a0, inc, offset, 0.25, 0.45, 0.6)
        assert scale == 0.0 and blocked
        r, a0, inc = _scan(front_range=1.0, offset=offset)
        scale, blocked, _, _ = fe.scan_safety(r, a0, inc, offset, 0.25, 0.45, 0.6)
        assert scale == 1.0 and not blocked


def test_ac3_scan_safety_rear_obstacle_does_not_stop():
    r, a0, inc = _scan(rear_range=0.1, offset=math.pi)
    scale, blocked, mn, esc = fe.scan_safety(r, a0, inc, math.pi, 0.25, 0.45, 0.6)
    assert scale == 1.0 and not blocked and esc == (0, 0)
    assert mn == pytest.approx(0.1)


def _beams(base_angles_ranges, offset, n=360, default=5.0):
    """Scan whose beams at the given base-frame angles (+-0.05 rad) have the given ranges."""
    inc = 2 * math.pi / n
    r = np.full(n, default)
    ang_base = fe.wrap_angle(np.arange(n) * inc + offset)
    for center, val in base_angles_ranges:
        sel = np.abs(fe.wrap_angle(ang_base - center)) <= 0.05
        r[sel] = val
    return r, 0.0, inc


def test_ac3_scan_safety_escape_dir():
    off = math.pi
    # obstacle ahead-left (base +0.3) -> the right side is freer -> turn right (-1); rear clear
    r, a0, inc = _beams([(0.3, 0.2)], off)
    _, blocked, _, esc = fe.scan_safety(r, a0, inc, off, 0.25, 0.45, 0.6)
    assert blocked and esc == (-1, -1)
    # mirror: obstacle ahead-right -> turn left (+1)
    r, a0, inc = _beams([(-0.3, 0.2)], off)
    _, blocked, _, esc = fe.scan_safety(r, a0, inc, off, 0.25, 0.45, 0.6)
    assert blocked and esc == (-1, 1)
    # rear blocked too: no reverse
    r, a0, inc = _beams([(0.3, 0.2), (math.pi, 0.3)], off)
    _, _, _, esc = fe.scan_safety(r, a0, inc, off, 0.25, 0.45, 0.6)
    assert esc[0] == 0 and esc[1] in (-1, 1)
    # not blocked -> (0, 0)
    r, a0, inc = _beams([(0.3, 2.0)], off)
    assert fe.scan_safety(r, a0, inc, off, 0.25, 0.45, 0.6)[3] == (0, 0)


def test_ac3_scan_safety_inf_and_empty():
    r = np.full(360, np.inf)
    scale, blocked, mn, esc = fe.scan_safety(r, 0.0, 2 * math.pi / 360, math.pi)
    assert scale == 1.0 and not blocked and mn == math.inf
    assert fe.scan_safety([], 0.0, 0.01, 0.0)[1] is False


def test_ac3_completion_monitor():
    m = fe.CompletionMonitor(hysteresis=3, timeout=10.0)
    # enough updates but not enough time
    assert not m.update(False, 0.0)
    assert not m.update(False, 1.0)
    assert not m.update(False, 2.0)
    assert not m.update(False, 9.9)
    # time reached and count satisfied
    assert m.update(False, 10.0)
    # time satisfied but too few updates
    m2 = fe.CompletionMonitor(3, 10.0)
    assert not m2.update(False, 0.0)
    assert not m2.update(False, 20.0)
    assert m2.update(False, 21.0)
    # reset when a frontier returns
    m3 = fe.CompletionMonitor(3, 10.0)
    for t in (0.0, 5.0, 11.0):
        m3.update(False, t)
    assert not m3.update(True, 12.0)
    assert not m3.update(False, 13.0)
    assert not m3.update(False, 24.0)  # only 2 empties since reset
    assert m3.update(False, 25.0)  # count 3 and 12 s since the first empty update
    m4 = fe.CompletionMonitor(3, 10.0)
    assert not m4.update(False, 0.0)
    assert not m4.update(True, 5.0)
    assert not m4.update(False, 6.0)
    assert not m4.update(False, 7.0)
    assert not m4.update(False, 8.0)   # count 3, but only 2 s since reset
    assert m4.update(False, 16.0)


# ---------------------------------------------------------------------------
# ExplorerCore basic contract
# ---------------------------------------------------------------------------

def test_core_none_scan_and_no_map_give_zero():
    core = fe.ExplorerCore({}, math.pi)
    scan = fe.ScanData(np.full(360, 5.0), 0.0, 2 * math.pi / 360)
    assert core.step((0, 0, 0), scan, 0.0).v == 0.0
    g = np.full((20, 20), F, dtype=np.int8)
    core.update_map(g, fe.GridInfo(0.05, 0, 0, 20, 20), 0.0)
    cmd = core.step((0.5, 0.5, 0), None, 1.0)
    assert cmd.v == 0.0 and cmd.w == 0.0
