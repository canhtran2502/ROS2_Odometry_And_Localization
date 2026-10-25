"""
AC3 (configuration handling) and AC12 (robustness) of both safety_supervisor executables.

Process level: every case starts the real executable (C++ and Python) in its own process
group against MockSafetyWorld and checks exit codes, signals and what was published.

  ROS_DOMAIN_ID=92 ROS_LOCALHOST_ONLY=1 python3 -m pytest test_safety_robustness.py -v
"""
import glob
import os
import signal
import sys
import time

os.environ.setdefault("ROS_DOMAIN_ID", "92")
os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import pytest  # noqa: E402
import rclpy  # noqa: E402

from mock_safety_world import MockSafetyWorld  # noqa: E402
from safety_harness import DATA, SHIPPED_CONFIG, SupervisorProcess, Workdir  # noqa: E402

IMPLS = ["cpp", "py"]
INVALID = sorted(glob.glob(os.path.join(DATA, "invalid_*.yaml")))
VALID = sorted(glob.glob(os.path.join(DATA, "valid_*.yaml"))) + [SHIPPED_CONFIG]
RECOVERY_TIME = 0.5
SCAN_TIMEOUT = 0.6


@pytest.fixture(scope="module")
def world():
    rclpy.init()
    w = MockSafetyWorld("mock_safety_world_robustness")
    yield w
    w.shutdown()
    rclpy.shutdown()


@pytest.fixture()
def workdir():
    wd = Workdir()
    yield wd
    wd.cleanup()


def healthy(w):
    w.reset()
    w.set_laser_tf(True)
    w.set_odom("follow_output")


def start(impl, params, workdir, extra=()):
    return SupervisorProcess(impl, params, workdir, extra_args=extra)


def ids(paths):
    return [os.path.basename(p) for p in paths]


# ------------------------------------------------------------------ AC3
@pytest.mark.parametrize("path", VALID, ids=ids(VALID))
@pytest.mark.parametrize("impl", IMPLS)
def test_valid_config_starts_locked(world, workdir, impl, path):
    """AC3: the shipped and valid_* files start; the first status is INITIALIZING and locked."""
    world.reset()
    world.set_scan_mode("stop")
    mark = world.mark()
    proc = start(impl, path, workdir)
    try:
        got = world.wait_status(lambda m: True, 15.0)
        assert got is not None, "no status\n" + proc.output()
        assert got[1].state == "INITIALIZING", got[1]
        assert got[1].locked is True
        assert proc.alive(), proc.output()
        time.sleep(1.0)
        assert proc.alive()
        assert world.last_status().state == "INITIALIZING"
        outs = world.outs_since(mark)
        assert all(v == 0.0 for _, v in outs), "motion published while initializing"
    finally:
        proc.stop()


def run_invalid(world, workdir, impl, path):
    world.reset()
    world.set_scan_mode("stop")
    mark = world.mark()
    t0 = time.monotonic()
    proc = start(impl, path, workdir)
    rc = proc.wait(8.0)
    elapsed = time.monotonic() - t0
    time.sleep(0.5)
    out = proc.output()
    locks = [v for _, v in world.locks_since(mark)]
    outs = world.outs_since(mark)
    if rc is None:
        proc.stop()
    return rc, elapsed, locks, outs, out


@pytest.mark.parametrize("path", [p for p in INVALID if "duplicate" not in p],
                         ids=lambda p: os.path.basename(p))
@pytest.mark.parametrize("impl", IMPLS)
def test_invalid_config_fails_safe(world, workdir, impl, path):
    """AC3: exit 1 within 5 s, lock true published, nothing on cmd_vel_out_topic."""
    rc, elapsed, locks, outs, out = run_invalid(world, workdir, impl, path)
    assert rc == 1, "exit code {} (None = still running)\n{}".format(rc, out)
    assert elapsed <= 5.0, "exit took {:.2f} s".format(elapsed)
    assert True in locks, "no safety_stop: true published\n" + out
    assert False not in locks, "a release was published"
    assert outs == [], "velocity published on the output topic"
    assert "Traceback" not in out, out


def test_invalid_duplicate_key_same_outcome(world, workdir):
    """AC3: for a duplicate key both executables behave identically."""
    path = os.path.join(DATA, "invalid_duplicate_key.yaml")
    outcome = {}
    for impl in IMPLS:
        world.reset()
        world.set_scan_mode("stop")
        proc = start(impl, path, workdir)
        try:
            rc = proc.wait(6.0)
            if rc is not None:
                outcome[impl] = ("exit", rc)
            else:
                params = world.get_parameters("/safety_supervisor", timeout=10.0)
                assert params is not None, "no parameters\n" + proc.output()
                outcome[impl] = ("run", params["stop_margin"])
        finally:
            proc.stop()
    assert outcome["cpp"] == outcome["py"], outcome
    assert outcome["cpp"][0] == "exit" and outcome["cpp"][1] == 1 or outcome["cpp"][0] == "run"


# ------------------------------------------------------------------ AC12
def active_scenario(world, workdir, impl, overrides=None, clock=False):
    healthy(world)
    world.set_cmd(0.3)
    if clock:
        world.set_clock("running")
    params = workdir.write_params(overrides)
    proc = start(impl, params, workdir)
    got = world.wait_status(lambda m: m.state == "ACTIVE", 20.0)
    assert got is not None, "not ACTIVE: {}\n{}".format(world.last_status(), proc.output())
    assert world.wait_for(lambda: world.last_out() == 0.3, 3.0)
    return proc


@pytest.mark.parametrize("sig", [signal.SIGINT, signal.SIGTERM], ids=["SIGINT", "SIGTERM"])
@pytest.mark.parametrize("impl", IMPLS)
def test_signal_shutdown_locks(world, workdir, impl, sig):
    """AC12 (a), (b): exit 0 within 5 s, last lock true, last output zero, last status SHUTDOWN."""
    proc = active_scenario(world, workdir, impl)
    try:
        mark = world.mark()
        t0 = time.monotonic()
        proc.signal(sig)
        rc = proc.wait(5.0)
        elapsed = time.monotonic() - t0
        assert rc == 0, "exit code {} after {:.2f} s\n{}".format(rc, elapsed, proc.output())
        time.sleep(0.5)
        out = proc.output()
        assert "Traceback" not in out, out
        assert world.last_lock() is True, "last lock {}".format(world.last_lock())
        assert world.last_out() == 0.0, "last output {}".format(world.last_out())
        assert world.last_status().state == "SHUTDOWN", world.last_status().state
        assert world.last_status().locked is True
        assert world.statuses_since(mark), "no status after the signal"
    finally:
        proc.stop()


@pytest.mark.parametrize("impl", IMPLS)
def test_tick_exceptions_recover(world, workdir, impl):
    """AC12 (c): debug_inject_tick_exceptions: FAULT internal error, then ACTIVE again."""
    healthy(world)
    world.set_cmd(0.3)
    params = workdir.write_params({"debug_inject_tick_exceptions": 3})
    mark = world.mark()
    proc = start(impl, params, workdir)
    try:
        got = world.wait_status(lambda m: m.state == "FAULT", 15.0, since=mark)
        assert got is not None, "no FAULT\n{}".format(proc.output())
        assert got[1].reason.startswith("internal error"), got[1].reason
        assert got[1].locked
        t_fault = got[0]
        assert proc.alive()
        active = world.wait_status(lambda m: m.state == "ACTIVE", 10.0, since=mark)
        assert active is not None, "no recovery: {}".format(world.last_status())
        assert active[0] - t_fault <= RECOVERY_TIME + 1.0 + 2.0, "slow recovery {:.2f}".format(
            active[0] - t_fault)
        assert proc.alive()
        assert world.wait_for(lambda: world.last_out() == 0.3, 3.0)
    finally:
        proc.stop()


@pytest.mark.parametrize("impl", IMPLS)
def test_stalled_clock_keeps_protecting(world, workdir, impl):
    """AC12 (d): use_sim_time with a /clock that stops: status, lock and watchdog keep running."""
    proc = active_scenario(world, workdir, impl, overrides={"use_sim_time": True}, clock=True)
    try:
        time.sleep(2.0)
        world.set_clock("stopped")
        time.sleep(0.3)
        mark = world.mark()
        time.sleep(5.0)
        statuses = world.statuses_since(mark)
        locks = world.locks_since(mark)
        assert len(statuses) / 5.0 >= 2.0, "status rate {:.2f} Hz".format(len(statuses) / 5.0)
        assert len(locks) / 5.0 >= 10.0, "lock republish rate {:.1f} Hz".format(len(locks) / 5.0)
        assert world.last_status().state == "ACTIVE"
        # scan dropout during the stall
        world.set_scan_mode("stop")
        time.sleep(0.05)
        with world.lock:
            t_last = world.scan_log[-1][0]
        got = world.wait_status(lambda m: m.state == "FAULT", 3.0)
        assert got is not None, "no FAULT while the clock is stalled"
        assert got[0] - t_last <= SCAN_TIMEOUT + 0.15, "latency {:.3f}".format(got[0] - t_last)
        assert got[1].locked
        mark = world.mark()
        time.sleep(1.0)
        assert len(world.locks_since(mark)) >= 10
        assert all(v for _, v in world.locks_since(mark))
        assert proc.alive()
    finally:
        proc.stop()


@pytest.mark.parametrize("impl", IMPLS)
def test_second_sigint_during_shutdown(world, workdir, impl):
    """AC12 (e): further signals during the shutdown give no traceback and exit code 0."""
    proc = active_scenario(world, workdir, impl)
    try:
        proc.signal(signal.SIGINT)
        time.sleep(0.03)
        proc.signal(signal.SIGINT)
        time.sleep(0.03)
        proc.signal(signal.SIGTERM)
        time.sleep(0.03)
        proc.signal(signal.SIGINT)
        rc = proc.wait(5.0)
        out = proc.output()
        assert rc == 0, "exit code {}\n{}".format(rc, out)
        assert "Traceback" not in out, out
        time.sleep(0.5)
        assert world.last_lock() is True
        assert world.last_out() == 0.0
        assert world.last_status().state == "SHUTDOWN"
    finally:
        proc.stop()
