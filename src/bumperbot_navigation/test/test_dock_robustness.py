"""
AC3 (config parsing parity at node level) and AC14 (signals, stalled clock, sim time).

Both dock_manager executables (C++ and Python) are started as plain processes in their own process
group next to a MockDockWorld, so that a signal or a failing test never leaves an orphan node.
"""
import glob
import math
import os
import signal
import sys
import time

os.environ.setdefault("ROS_DOMAIN_ID", "90")
os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")

import pytest  # noqa: E402
from action_msgs.msg import GoalStatus  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from bumperbot_msgs.action import DockRobot  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from dock_harness import DATA, IMPLS, Rig, wait_until  # noqa: E402

ABORTED = GoalStatus.STATUS_ABORTED
R = DockRobot.Result
NEAR_START = (0.01, -0.01, 0.02)
FAR_START = (1.5, 1.4, 0.3)


@pytest.fixture(params=IMPLS)
def impl(request):
    return request.param


@pytest.fixture
def rig_factory():
    rigs = []

    def make(**kwargs):
        rig = Rig(**kwargs)
        rigs.append(rig)
        return rig

    yield make
    for rig in rigs:
        rig.close()


def moving(cmd):
    return cmd["v"] != 0.0 or cmd["w"] != 0.0


# ====================================================================== AC3
def shipped_stations():
    return os.path.join(get_package_share_directory("bumperbot_navigation"), "config",
                        "stations.yaml")


VALID_FILES = {
    "shipped": shipped_stations,
    "default_param": lambda: None,
    "stations_test": lambda: os.path.join(DATA, "stations_test.yaml"),
    "valid_exponent": lambda: os.path.join(DATA, "valid_exponent.yaml"),
}


def first_status(rig, impl, tmp_path, stations_file):
    rig.world.reset(tf_enabled=False)
    proc = rig.start(impl, tmp_path, {"stations_file": stations_file}, tag="valid")
    try:
        assert wait_until(lambda: rig.world.last_status() is not None, 20.0), proc.log()
        status = rig.world.last_status()
        assert proc.alive(), proc.log()
        assert not rig.world.cmds, "dock_vel published at startup"
    finally:
        code = rig.stop_node()
    assert code == 0, "exit code {}".format(code)
    return status


@pytest.mark.parametrize("case", sorted(VALID_FILES))
def test_first_status_is_identical_in_both_implementations(case, tmp_path, rig_factory):
    """AC3: UNDOCKED status with the same staging_pose, dock_pose and frame_id (1e-9)."""
    rig = rig_factory()
    stations_file = VALID_FILES[case]()
    statuses = {impl: first_status(rig, impl, tmp_path, stations_file) for impl in IMPLS}
    for impl, status in statuses.items():
        assert status["state"] == "UNDOCKED", impl
        assert not status["docked"], impl
        assert status["frame_id"] == "map", impl
        assert status["staging"] == pytest.approx((0.0, 0.0, 0.0), abs=1e-9), impl
        assert status["dock"] == pytest.approx((-0.5, 0.0, 0.0), abs=1e-9), impl
    for key in ("staging", "dock"):
        assert statuses["cpp"][key] == pytest.approx(statuses["py"][key], abs=1e-9)
    assert statuses["cpp"]["frame_id"] == statuses["py"]["frame_id"]


INVALID_FILES = sorted(
    os.path.basename(p) for p in glob.glob(os.path.join(DATA, "invalid_*.yaml")))
INVALID_PARAMS = {
    "other_frame": {"stations_file": os.path.join(DATA, "stations_other_frame.yaml")},
    "odom_equals_robot_frame": {"stations_file": os.path.join(DATA, "stations_test.yaml"),
                                "odom_frame": "base_footprint"},
    "bogus_detection_mode": {"stations_file": os.path.join(DATA, "stations_test.yaml"),
                             "detection_mode": "bogus"},
    "zero_staging_distance": {"stations_file": os.path.join(DATA, "stations_test.yaml"),
                              "staging_distance": 0.0},
    "missing_file": {"stations_file": os.path.join(DATA, "does_not_exist.yaml")},
}


def assert_refuses_to_start(rig, impl, tmp_path, params):
    rig.world.reset(tf_enabled=False)
    t0 = time.monotonic()
    proc = rig.start(impl, tmp_path, params, tag="invalid")
    code = proc.wait_exit(5.0)
    elapsed = time.monotonic() - t0
    log = proc.log()
    rig.stop_node()
    assert code is not None, "still running after 5 s\n" + log
    assert code != 0, "exit code 0\n" + log
    assert elapsed <= 5.0
    assert log.strip() != "", "no reason logged"
    assert not rig.world.cmds, "dock_vel was published"


def test_invalid_fixtures_exist():
    assert len(INVALID_FILES) >= 11
    for name in ("invalid_duplicate_key.yaml", "invalid_quoted_number.yaml", "invalid_nan.yaml",
                 "invalid_missing_field.yaml", "invalid_no_dock.yaml", "invalid_empty.yaml",
                 "invalid_not_a_mapping.yaml", "invalid_null_dock_station.yaml"):
        assert name in INVALID_FILES, name


@pytest.mark.parametrize("name", INVALID_FILES)
def test_invalid_stations_file_refuses_to_start(name, impl, tmp_path, rig_factory):
    """AC3: every invalid_*.yaml makes the node exit with a code != 0 within 5 s."""
    rig = rig_factory()
    assert_refuses_to_start(rig, impl, tmp_path, {"stations_file": os.path.join(DATA, name)})


@pytest.mark.parametrize("case", sorted(INVALID_PARAMS))
def test_invalid_configuration_refuses_to_start(case, impl, tmp_path, rig_factory):
    """AC3: other frame, equal frames, a bogus detection mode and a zero distance are refused."""
    rig = rig_factory()
    assert_refuses_to_start(rig, impl, tmp_path, INVALID_PARAMS[case])


# ====================================================================== AC14
SIGNALS = {"SIGINT": signal.SIGINT, "SIGTERM": signal.SIGTERM}


def start_ready(rig, impl, tmp_path, params=None, **world_kwargs):
    rig.world.reset(**world_kwargs)
    proc = rig.start(impl, tmp_path, params)
    assert rig.wait_ready(30.0), "dock_manager did not start\n" + proc.log()
    time.sleep(1.0)
    return proc


def send(rig):
    run = rig.world.send_goal(DockRobot.Goal.COMMAND_DOCK)
    assert rig.world.wait_decision(run, 5.0)
    assert run.accepted
    return run


def wait_state(rig, state, timeout, with_motion=False):
    w = rig.world

    def reached():
        if w.state != state:
            return False
        return not with_motion or any(c["state"] == state and moving(c) for c in w.cmds)

    assert wait_until(reached, timeout, 0.01), "state {} not reached (last {})".format(
        state, w.state)


def assert_aborted_with_error(run):
    assert wait_until(lambda: run.result is not None, 3.0), "the client saw no result"
    assert run.result.status == ABORTED
    assert run.result.result.outcome == R.OUTCOME_ERROR
    assert not run.result.result.docked


@pytest.mark.parametrize("signal_name", sorted(SIGNALS))
def test_signal_during_docking(signal_name, impl, tmp_path, rig_factory):
    """AC14a/b: SIGINT / SIGTERM during DOCKING stop the robot, abort the goal and exit 0."""
    rig = rig_factory()
    proc = start_ready(rig, impl, tmp_path, pose=NEAR_START)
    run = send(rig)
    wait_state(rig, "DOCKING", 40.0, with_motion=True)
    time.sleep(0.3)
    t_signal = time.monotonic()
    proc.send_signal(SIGNALS[signal_name])
    code = proc.wait_exit(10.0)
    assert code == 0, "exit code {} within 10 s\n{}".format(code, proc.log())
    assert time.monotonic() - t_signal <= 10.0
    assert_aborted_with_error(run)
    cmds = rig.world.cmds
    assert (cmds[-1]["v"], cmds[-1]["w"]) == (0.0, 0.0), "the last dock_vel message is not zero"
    assert "Traceback" not in proc.log() and "terminate called" not in proc.log(), proc.log()
    rig.proc.stop()


@pytest.mark.parametrize("signal_name", sorted(SIGNALS))
def test_signal_during_navigation(signal_name, impl, tmp_path, rig_factory):
    """AC14a/b: a signal during NAVIGATING_TO_STAGING cancels the Nav2 goal."""
    rig = rig_factory()
    proc = start_ready(rig, impl, tmp_path, pose=FAR_START, nav_script={"duration": 30.0})
    run = send(rig)
    wait_state(rig, "NAVIGATING_TO_STAGING", 10.0)
    assert wait_until(lambda: len(rig.world.nav_goals) == 1, 5.0)
    time.sleep(0.5)
    proc.send_signal(SIGNALS[signal_name])
    code = proc.wait_exit(10.0)
    assert code == 0, "exit code {}\n{}".format(code, proc.log())
    assert wait_until(lambda: len(rig.world.nav_cancels) >= 1, 3.0), \
        "the Nav2 goal was not canceled\n" + proc.log()
    assert_aborted_with_error(run)
    assert not [c for c in rig.world.cmds if moving(c)]
    if rig.world.cmds:
        assert (rig.world.cmds[-1]["v"], rig.world.cmds[-1]["w"]) == (0.0, 0.0)


def test_signal_while_idle_exits_zero(impl, tmp_path, rig_factory):
    rig = rig_factory()
    proc = start_ready(rig, impl, tmp_path, pose=NEAR_START)
    t0 = time.monotonic()
    proc.send_signal(signal.SIGINT)
    assert proc.wait_exit(10.0) == 0, proc.log()
    assert time.monotonic() - t0 < 5.0


SIM_PARAMS = {"use_sim_time": True, "clock_stall_timeout": 3.0}


def test_stalled_clock_aborts_with_error(impl, tmp_path, rig_factory):
    """AC14c: /clock stops during DOCKING: zero command, ABORTED ERROR, status keeps flowing."""
    rig = rig_factory(sim_time=True)
    assert wait_until(lambda: rig.clock.sim > 1000.5, 5.0)
    proc = start_ready(rig, impl, tmp_path, SIM_PARAMS, pose=NEAR_START)
    run = send(rig)
    wait_state(rig, "DOCKING", 60.0, with_motion=True)
    time.sleep(0.3)
    rig.clock.speed = 0.0
    t_stop = time.monotonic()
    assert wait_until(lambda: run.result is not None, 3.0 + 2.0 + 1.0), \
        "no result after the clock stopped\n" + proc.log()
    assert run.t_result - t_stop <= 3.0 + 2.0, "took {:.1f} s".format(run.t_result - t_stop)
    assert run.result.status == ABORTED
    assert run.result.result.outcome == R.OUTCOME_ERROR
    assert not run.result.result.docked
    time.sleep(3.0)
    cmds = rig.world.commands_since(t_stop)
    zero = next((c for c in cmds if not moving(c)), None)
    assert zero is not None
    assert not [c for c in cmds if c["t"] > zero["t"] and moving(c)]
    assert (rig.world.cmds[-1]["v"], rig.world.cmds[-1]["w"]) == (0.0, 0.0)
    arrivals = [t_stop] + [s["t"] for s in rig.world.status_log if s["t"] >= t_stop]
    arrivals.append(time.monotonic())
    gaps = [b - a for a, b in zip(arrivals, arrivals[1:])]
    assert max(gaps) <= 1.5, "status gap of {:.2f} s while the clock was stalled".format(max(gaps))
    assert proc.alive(), proc.log()


def test_sim_time_with_running_clock_docks(impl, tmp_path, rig_factory):
    """AC14d: the AC5 scenario with use_sim_time and a running /clock."""
    rig = rig_factory(sim_time=True)
    assert wait_until(lambda: rig.clock.sim > 1000.5, 5.0)
    start_ready(rig, impl, tmp_path, SIM_PARAMS, pose=FAR_START, staging_error=(0.08, -0.06, 0.2))
    w = rig.world
    run = send(rig)
    assert w.wait_result(run, 120.0), "no result"
    assert run.result.status == GoalStatus.STATUS_SUCCEEDED, run.result.result.message
    result = run.result.result
    assert result.outcome == R.OUTCOME_DOCKED
    assert result.docked and result.attempts == 1
    pos, yaw = w.charger_error()
    assert pos <= 0.03 and yaw <= 0.05, (pos, yaw)
    assert len(w.nav_goals) == 1
    states = w.states()
    order = [states.index(s) for s in ("NAVIGATING_TO_STAGING", "ALIGNING", "DOCKING", "DOCKED")]
    assert order == sorted(order), states
    time.sleep(0.4)
    assert (w.cmds[-1]["v"], w.cmds[-1]["w"]) == (0.0, 0.0)
    assert math.isfinite(pos)


# ====================================================================== review fix M1
# A Nav2 goal request that is still unanswered when the dock goal ends (cancel, shutdown, stalled
# clock, answer timeout) must not leave an orphan Nav2 goal: a goal accepted late is canceled
# exactly once. The mock Nav2 holds its goal response back (nav_accept_delay) and then accepts.
NAV_PARAMS = {"nav_server_timeout": 10.0, "nav_goal_timeout": 30.0}


def start_slow_nav(rig, impl, tmp_path, delay, params=None):
    """Start the node with a Nav2 whose goal response takes ``delay`` s; send a DOCK goal."""
    merged = dict(NAV_PARAMS)
    merged.update(params or {})
    proc = start_ready(rig, impl, tmp_path, merged, pose=FAR_START, nav_script={"duration": 30.0})
    rig.world.nav_accept_delay = delay
    run = send(rig)
    wait_state(rig, "NAVIGATING_TO_STAGING", 10.0)
    assert wait_until(lambda: len(rig.world.nav_goals) == 1, 5.0), "no Nav2 goal request"
    time.sleep(0.2)     # the request is in flight: Nav2 has not answered yet
    assert not rig.world.nav_cancel_requests
    return proc, run


def assert_one_late_cancel(rig, proc, timeout):
    """Expect exactly one cancel request for the late-accepted Nav2 goal, and it stays at one."""
    assert wait_until(lambda: len(rig.world.nav_cancel_requests) >= 1, timeout), \
        "the late-accepted Nav2 goal was never canceled\n" + proc.log()
    time.sleep(1.5)
    assert len(rig.world.nav_cancel_requests) == 1, rig.world.nav_cancel_requests
    assert len(rig.world.nav_goals) == 1, "unexpected extra Nav2 goals"
    assert wait_until(lambda: len(rig.world.nav_cancels) == 1, 3.0), \
        "the mock Nav2 did not end the goal as canceled"


@pytest.mark.parametrize("delay", [0.5, 2.0])
def test_m1_cancel_before_nav2_answers_cancels_the_late_goal(delay, impl, tmp_path, rig_factory):
    """M1: DockRobot cancel while the Nav2 request is unanswered -> late goal canceled once."""
    rig = rig_factory()
    proc, run = start_slow_nav(rig, impl, tmp_path, delay)
    t_cancel = time.monotonic()
    rig.world.cancel(run)
    assert wait_until(lambda: run.result is not None, 5.0), "no result\n" + proc.log()
    assert run.result.status == GoalStatus.STATUS_CANCELED
    assert run.result.result.outcome == R.OUTCOME_CANCELED
    assert not run.result.result.docked
    assert run.t_result - t_cancel <= 2.5, "the cancel took {:.2f} s".format(
        run.t_result - t_cancel)
    assert_one_late_cancel(rig, proc, delay + 3.0)
    assert not [c for c in rig.world.cmds if moving(c)]
    assert proc.alive(), proc.log()
    assert "Traceback" not in proc.log(), proc.log()


@pytest.mark.parametrize("signal_name", sorted(SIGNALS))
def test_m1_shutdown_before_nav2_answers_cancels_the_late_goal(
        signal_name, impl, tmp_path, rig_factory):
    """M1: SIGINT / SIGTERM while the request is unanswered: the late goal is canceled once."""
    rig = rig_factory()
    proc, run = start_slow_nav(rig, impl, tmp_path, 0.5)
    proc.send_signal(SIGNALS[signal_name])
    code = proc.wait_exit(10.0)
    assert code == 0, "exit code {}\n{}".format(code, proc.log())
    assert_aborted_with_error(run)
    assert_one_late_cancel(rig, proc, 3.0)
    assert not [c for c in rig.world.cmds if moving(c)]
    assert "Traceback" not in proc.log() and "terminate called" not in proc.log(), proc.log()


def test_m1_clock_stall_before_nav2_answers_cancels_the_late_goal(impl, tmp_path, rig_factory):
    """M1: the node clock stalls while the request is unanswered: the late goal is canceled."""
    rig = rig_factory(sim_time=True)
    assert wait_until(lambda: rig.clock.sim > 1000.5, 5.0)
    params = dict(SIM_PARAMS)
    params["nav_server_timeout"] = 20.0
    proc, run = start_slow_nav(rig, impl, tmp_path, 6.0, params)
    rig.clock.speed = 0.0
    t_stop = time.monotonic()
    assert wait_until(lambda: run.result is not None, 3.0 + 3.0), \
        "no result after the clock stopped\n" + proc.log()
    assert run.result.status == ABORTED
    assert run.result.result.outcome == R.OUTCOME_ERROR
    assert run.t_result - t_stop <= 3.0 + 3.0
    assert_one_late_cancel(rig, proc, 6.0 + 3.0)
    assert not [c for c in rig.world.cmds if moving(c)]
    assert proc.alive(), proc.log()


def test_m1_answer_timeout_cancels_every_late_goal(impl, tmp_path, rig_factory):
    """M1: Nav2 answers later than nav_server_timeout: each late goal is canceled exactly once."""
    rig = rig_factory()
    proc = start_ready(rig, impl, tmp_path, {"nav_server_timeout": 1.0}, pose=FAR_START,
                       nav_script={"duration": 30.0})
    rig.world.nav_accept_delay = 2.5
    run = send(rig)
    assert wait_until(lambda: run.result is not None, 60.0), "no result\n" + proc.log()
    assert run.result.status == ABORTED
    assert run.result.result.outcome in (R.OUTCOME_STAGING_FAILED, R.OUTCOME_ERROR)
    goals = len(rig.world.nav_goals)
    assert goals >= 1
    assert wait_until(lambda: len(rig.world.nav_cancel_requests) == goals, 2.5 + 3.0), \
        "{} cancels for {} Nav2 goals\n{}".format(
            len(rig.world.nav_cancel_requests), goals, proc.log())
    time.sleep(1.5)
    assert len(rig.world.nav_cancel_requests) == goals
    assert len(rig.world.nav_goals) == goals
    assert not [c for c in rig.world.cmds if moving(c)]


def test_m1_goal_answered_in_time_is_not_canceled_early(impl, tmp_path, rig_factory):
    """Control: with a Nav2 that answers within the timeout the goal runs until it succeeds."""
    rig = rig_factory()
    start_ready(rig, impl, tmp_path, NAV_PARAMS, pose=FAR_START, nav_script={"duration": 0.5})
    rig.world.nav_accept_delay = 1.0
    run = send(rig)
    assert rig.world.wait_result(run, 120.0), "no result"
    assert run.result.status == GoalStatus.STATUS_SUCCEEDED, run.result.result.message
    assert rig.world.nav_cancel_requests == []
    assert len(rig.world.nav_goals) == 1
