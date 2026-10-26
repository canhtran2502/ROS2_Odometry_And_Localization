"""AC14: documentation mentions the new package, launch file, interfaces and stations file."""
import os
import re
import signal
import subprocess
import time

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
DOCS = ["README.md", "doc/10_navigation.md", "doc/03_ros2_nodes_topics_services.md",
        "doc/01_repository_structure.md", "doc/11_build_run_and_command_reference.md"]
REQUIRED = ["bumperbot_navigation", "mission_sim.launch.py", "/execute_mission",
            "/mission/status", "stations.yaml"]


def read(rel):
    with open(os.path.join(ROOT, rel), encoding="utf-8") as f:
        return f.read()


def test_required_strings_in_every_doc():
    missing = []
    for rel in DOCS:
        text = read(rel)
        for token in REQUIRED:
            if token not in text:
                missing.append(f"{rel}: {token}")
    assert not missing, missing


def test_doc10_no_longer_says_nav2_is_missing():
    text = read("doc/10_navigation.md")
    assert not re.search(r"no\s+Nav2\s+navigation\s+stack\s+is\s+configured", text, re.I)
    assert not re.search(r"Nav2 (is|are) not (yet )?configured", text, re.I)


def test_doc10_describes_review_fix_rules():
    text = read("doc/10_navigation.md")
    flat = re.sub(r"\s+", " ", text)
    for token in ("nav_server_timeout", "retry_delay", "duplicated", "frame_id", "global_frame",
                  "1e3", "quoted", "REJECTED", "Terminal status precedence",
                  "DOCK_FAILED"):
        assert token in flat, token
    # iteration 4 capitalized the bullet ("Wall (steady) time:"): same rule, case-insensitive
    assert "wall (steady) time" in flat.lower()
    assert re.search(r"without counting a failed attempt", flat)
    assert re.search(r"refuses to start when `frame_id` differs from its `global_frame`", flat)
    assert re.search(r"not a non-null string", flat)
    assert re.search(r"has not started driving to another station", flat)


def test_nav2_params_controller_odom_topic():
    import yaml
    with open(os.path.join(HERE, "..", "config", "nav2_params.yaml")) as f:
        params = yaml.safe_load(f)
    assert params["controller_server"]["ros__parameters"]["odom_topic"] == \
        "/bumperbot_controller/odom"


# ------------------------------------------------------------------ iteration 4: shutdown
# Registered pytest file for the signal / constructor-error tests (see test_stations_on_map.py).
def _robust_imports():
    import sys
    if HERE not in sys.path:
        sys.path.insert(0, HERE)
    import robust_harness
    return robust_harness


@pytest.mark.parametrize("sig", [signal.SIGINT, signal.SIGTERM], ids=["SIGINT", "SIGTERM"])
@pytest.mark.parametrize("impl", ["cpp", "py"])
def test_signal_mid_mission_cancels_nav2_goal_and_exits_cleanly(impl, sig, tmp_path):
    rh = _robust_imports()
    h = rh.Harness()
    try:
        proc = h.start_node(impl, tmp_path, {
            "retry_delay": 0.1, "nav_server_timeout": 5.0, "nav_goal_timeout": 60.0})
        assert h.wait_ready(), "execute_mission not available\n" + h.log()
        h.mock.reset({"a": {"duration": 30.0}})
        h.send(["a"])
        assert rh.wait_until(lambda: any(g["name"] == "a" for g in h.mock.snapshot()[0]), 15.0)
        time.sleep(0.5)
        t0 = time.monotonic()
        os.kill(proc.pid, sig)
        try:
            rc = proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            pytest.fail("node did not exit within 15 s after the signal\n" + h.log())
        took = time.monotonic() - t0
        out = h.log()
        assert rc == 0, "exit code {} (signal {})\n{}".format(rc, -rc if rc < 0 else "", out)
        for bad in ("terminate", "Traceback", "Aborted", "core dumped"):
            assert bad not in out, out
        assert took < 10.0, "shutdown took {:.1f} s".format(took)
        assert rh.wait_until(lambda: h.mock.cancel_requests.get("a", 0) >= 1, 3.0), \
            "Nav2 never received a cancel request for the active goal\n" + out
        assert rh.wait_until(lambda: any(c["name"] == "a" for c in h.mock.snapshot()[1]), 3.0)
    finally:
        h.close()


@pytest.mark.parametrize("sig", [signal.SIGINT, signal.SIGTERM], ids=["SIGINT", "SIGTERM"])
@pytest.mark.parametrize("impl", ["cpp", "py"])
def test_signal_while_idle_exits_cleanly(impl, sig, tmp_path):
    rh = _robust_imports()
    h = rh.Harness()
    try:
        proc = h.start_node(impl, tmp_path)
        assert h.wait_ready(), "execute_mission not available\n" + h.log()
        os.kill(proc.pid, sig)
        try:
            rc = proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            pytest.fail("node did not exit within 10 s after the signal\n" + h.log())
        out = h.log()
        assert rc == 0, "exit code {}\n{}".format(rc, out)
        for bad in ("terminate", "Traceback", "Aborted"):
            assert bad not in out, out
    finally:
        h.close()


@pytest.mark.parametrize("impl", ["cpp", "py"])
def test_constructor_error_gives_nonzero_exit_and_fatal_log(impl, tmp_path):
    rh = _robust_imports()
    cmd = [rh.executable(impl), "--ros-args", "-p",
           "stations_file:=" + str(tmp_path / "does_not_exist.yaml")]
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
    out = proc.stdout + proc.stderr
    assert proc.returncode != 0, "node started with a missing stations file\n" + out
    assert proc.returncode > 0, "killed by a signal ({})\n{}".format(proc.returncode, out)
    assert "FATAL" in out, "no FATAL log for the constructor error\n" + out
