"""
AC3: stations lie on free space of the small_house map.

Every shipped station needs enough clearance and must be in the free connected component of the
dock.
"""
import os
import time

import numpy as np
import pytest
import yaml
from scipy import ndimage

HERE = os.path.dirname(os.path.abspath(__file__))
STATIONS = os.path.join(HERE, "..", "config", "stations.yaml")
MAP_DIR = os.path.join(HERE, "..", "..", "bumperbot_mapping", "maps", "small_house")

MIN_CLEARANCE = 0.30
CONNECT_CLEARANCE = 0.15


def read_pgm(path):
    with open(path, "rb") as f:
        data = f.read()
    tokens, pos = [], 0
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
    assert maxval < 256
    return np.frombuffer(data, dtype=np.uint8, count=w * h, offset=pos).reshape(h, w)


@pytest.fixture(scope="module")
def world():
    with open(os.path.join(MAP_DIR, "map.yaml")) as f:
        meta = yaml.safe_load(f)
    img = read_pgm(os.path.join(MAP_DIR, meta["image"]))
    res = float(meta["resolution"])
    ox, oy = float(meta["origin"][0]), float(meta["origin"][1])
    occ_prob = (255.0 - img) / 255.0 if not meta.get("negate", 0) else img / 255.0
    occupied = occ_prob > float(meta["occupied_thresh"])
    free = occ_prob < float(meta["free_thresh"])
    assert occupied.any() and free.any()
    # distance (m) from every cell to the nearest occupied cell
    clearance = ndimage.distance_transform_edt(~occupied) * res
    h = img.shape[0]

    def cell(x, y):
        col = int(np.floor((x - ox) / res))
        row = h - 1 - int(np.floor((y - oy) / res))
        return row, col

    return occupied, free, clearance, cell


@pytest.fixture(scope="module")
def stations():
    with open(STATIONS) as f:
        return yaml.safe_load(f)["stations"]


def test_station_count(stations):
    assert "dock" in stations
    assert len(stations) >= 4


def test_stations_free_clear_and_connected(world, stations):
    occupied, free, clearance, cell = world
    candidate = free & (clearance > CONNECT_CLEARANCE)
    labels, _ = ndimage.label(candidate)
    drow, dcol = cell(stations["dock"]["x"], stations["dock"]["y"])
    dock_label = labels[drow, dcol]
    assert dock_label != 0, "dock is not in the free space of the map"
    problems = []
    for name, s in stations.items():
        row, col = cell(s["x"], s["y"])
        if not (0 <= row < free.shape[0] and 0 <= col < free.shape[1]):
            problems.append(f"{name}: outside the map")
            continue
        if not free[row, col]:
            problems.append(f"{name}: not a free cell")
        if clearance[row, col] < MIN_CLEARANCE:
            problems.append(f"{name}: clearance {clearance[row, col]:.2f} m < {MIN_CLEARANCE}")
        if labels[row, col] != dock_label:
            problems.append(f"{name}: not in the free component of the dock")
    assert not problems, "; ".join(problems)


# ------------------------------------------------------------------ iteration 4: robustness
# Nav2 disappearing mid-goal, stalled /clock and the node-clock status timer. The tests start the
# real mission_manager executables (C++ and Python) next to a MockNav2. They live in this pytest
# file because it is registered with ament_add_pytest_test.
def _robust_imports():
    import sys
    if HERE not in sys.path:
        sys.path.insert(0, HERE)
    import robust_harness
    return robust_harness


@pytest.fixture(params=["cpp", "py"])
def impl(request):
    return request.param


@pytest.mark.parametrize("restart_server", [False, True], ids=["dock_failed", "server_back"])
def test_nav2_disappears_mid_goal_fails_after_server_timeout(impl, restart_server, tmp_path):
    """
    Nav2 disappears mid-goal.

    The mock stops its action server while a goal (30 s long) is active. The attempt must fail
    after about nav_server_timeout (2 s), far before nav_goal_timeout (60 s). Then the mission
    still tries to dock: with the server gone for good DOCK_FAILED / docked=false, with the server
    back COMPLETED_WITH_SKIPS / docked.
    """
    rh = _robust_imports()
    h = rh.Harness()
    try:
        h.start_node(impl, tmp_path, {
            "retry_delay": 0.1, "nav_server_timeout": 2.0, "nav_goal_timeout": 60.0,
            "max_retries": 0, "dock_max_retries": 0, "use_sim_time": False})
        assert h.wait_ready(), "execute_mission not available\n" + h.log()
        h.mock.reset({"a": {"duration": 30.0}})
        handle = h.send(["a"])
        assert rh.wait_until(lambda: any(g["name"] == "a" for g in h.mock.snapshot()[0]), 15.0)
        time.sleep(0.5)
        t0 = time.monotonic()
        h.mock.stop_server()
        assert rh.wait_until(lambda: "RETURNING_TO_DOCK" in h.status_states(), 20.0), \
            "the attempt did not fail: " + h.log()
        t_fail = time.monotonic() - t0
        assert 1.5 <= t_fail <= 2.0 + 3.5, \
            "attempt failed after {:.1f} s, expected about nav_server_timeout=2 s".format(t_fail)
        if restart_server:
            h.mock.start_server()
        res = h.result(handle, timeout=30.0)
        total = time.monotonic() - t0
        assert total < 30.0, "mission took {:.1f} s".format(total)
        out = h.log()
        assert "disappeared" in out or "not available" in out, out
        result = res.result
        assert list(result.skipped) == ["a"], result
        if restart_server:
            assert result.outcome == result.OUTCOME_COMPLETED_WITH_SKIPS
            assert result.docked
            assert [g["name"] for g in h.mock.snapshot()[0]] == ["a", "dock"]
        else:
            assert result.outcome == result.OUTCOME_DOCK_FAILED
            assert not result.docked
        assert "Traceback" not in out and "terminate" not in out, out
    finally:
        h.close()


def test_stalled_clock_wait_ends_after_clock_stall_timeout(impl, tmp_path):
    """
    A stalled /clock must not hang a dwell.

    use_sim_time=true with a /clock publisher. Control: with a running clock the dwell ends with
    no stall warning. Then the clock stops during the dwell: the wait ends after about
    clock_stall_timeout (3 s wall) with a warning and the mission completes (no hang).
    """
    rh = _robust_imports()
    h = rh.Harness(with_clock=True)
    try:
        h.start_node(impl, tmp_path, {
            "use_sim_time": True, "retry_delay": 0.1, "nav_server_timeout": 5.0,
            "nav_goal_timeout": 20.0, "clock_stall_timeout": 3.0})
        assert h.wait_ready(), "execute_mission not available\n" + h.log()
        # control: running clock, dwell of 'a' is 1 s of sim time
        h.mock.reset()
        t0 = time.monotonic()
        res = h.result(h.send(["a"]), timeout=30.0)
        assert res.result.outcome == res.result.OUTCOME_COMPLETED, res.result
        assert time.monotonic() - t0 < 3.0 + 6.0
        assert "did not advance" not in h.log(), "spurious stall warning with a running clock"
        # stalled clock: stop /clock as soon as the dwell starts
        h.mock.reset()
        with h.lock:
            h.statuses.clear()
        handle = h.send(["a"])
        assert rh.wait_until(lambda: "DWELLING" in h.status_states(), 20.0), h.log()
        h.clock.speed = 0.0
        t_stall = time.monotonic()
        res = h.result(handle, timeout=40.0)
        elapsed = time.monotonic() - t_stall
        out = h.log()
        assert res.result.outcome == res.result.OUTCOME_COMPLETED, (res.result, out)
        assert "did not advance" in out, "no stall warning logged\n" + out
        assert 2.5 <= elapsed <= 3.0 + 6.0, \
            "dwell ended {:.1f} s after the stall, expected about 3 s".format(elapsed)
    finally:
        h.close()


def test_status_timer_follows_node_clock(impl, tmp_path):
    """
    The status timer runs on the node clock.

    /mission/status is published by a node-clock timer in both languages: with a /clock running at
    2x the periodic rate doubles (2 Hz sim = 4 Hz wall), it stops while /clock is stalled and
    resumes afterwards.
    """
    rh = _robust_imports()
    h = rh.Harness(with_clock=True, clock_speed=2.0)
    try:
        h.start_node(impl, tmp_path, {"use_sim_time": True, "status_rate": 2.0})
        assert h.wait_ready(), "execute_mission not available\n" + h.log()
        assert rh.wait_until(lambda: len(h.status_times()) >= 3, 15.0), h.log()
        time.sleep(1.0)

        def count_during(seconds):
            before = len(h.status_times())
            time.sleep(seconds)
            return len(h.status_times()) - before

        fast = count_during(3.0)
        assert 9 <= fast <= 15, "2x clock: {} status msgs in 3 s wall, expected about 12".format(
            fast)
        h.clock.speed = 0.0
        time.sleep(0.5)
        stalled = count_during(3.0)
        assert stalled <= 1, "{} status msgs while /clock is stalled".format(stalled)
        h.clock.speed = 1.0
        time.sleep(0.5)
        resumed = count_during(3.0)
        assert 4 <= resumed <= 8, "1x clock: {} status msgs in 3 s wall, expected 6".format(
            resumed)
    finally:
        h.close()
