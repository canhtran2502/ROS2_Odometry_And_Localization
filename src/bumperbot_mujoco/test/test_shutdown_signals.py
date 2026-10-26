"""
Review fix L2: dock_marker_sim and mujoco_sim_node exit 0 without a traceback on SIGINT / SIGTERM.

Both executables are started as plain processes (own process group, headless: the MuJoCo viewer
is disabled with ``viewer:=false``) on a private ROS domain, once idle and once mid-operation:
dock_marker_sim while it publishes detections, mujoco_sim_node while it publishes /clock, /scan
and ground truth and receives wheel commands. The ros2 CLI tools are used as test peers.
"""
import os
import signal
import subprocess
import time

import pytest
from ament_index_python.packages import get_package_prefix

DOMAIN = "93"
SIGNALS = {"SIGINT": signal.SIGINT, "SIGTERM": signal.SIGTERM}
BAD_MARKERS = ("Traceback", "rcl_shutdown already called", "RCLError", "terminate called")


def peer_env():
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = DOMAIN
    env["ROS_LOCALHOST_ONLY"] = "1"
    return env


def executable(name):
    return os.path.join(get_package_prefix("bumperbot_mujoco"), "lib", "bumperbot_mujoco", name)


class Proc:
    """A process in its own process group with its output in a log file."""

    def __init__(self, args, log_path):
        self.log_path = str(log_path)
        self.log_file = open(self.log_path, "w")
        self.proc = subprocess.Popen(args, stdout=self.log_file, stderr=subprocess.STDOUT,
                                     env=peer_env(), start_new_session=True)

    def log(self):
        self.log_file.flush()
        with open(self.log_path) as f:
            return f.read()

    def wait_log(self, text, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if text in self.log():
                return True
            if self.proc.poll() is not None:
                break
            time.sleep(0.1)
        return text in self.log()

    def kill(self):
        if self.proc.poll() is None:
            try:
                os.killpg(self.proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            self.proc.wait(timeout=5)
        self.log_file.close()


@pytest.fixture
def procs():
    started = []
    yield started
    for proc in started:
        proc.kill()


def start(procs, args, log_path):
    proc = Proc(args, log_path)
    procs.append(proc)
    return proc


def ros2_once(args, timeout):
    """Run a ros2 CLI command to completion; returns True on exit code 0."""
    res = subprocess.run(["ros2"] + args, env=peer_env(), capture_output=True, text=True,
                         timeout=timeout)
    return res.returncode == 0 and res.stdout.strip() != ""


def stop_and_check(proc, signum, name):
    t0 = time.monotonic()
    os.kill(proc.proc.pid, signum)
    try:
        code = proc.proc.wait(timeout=10.0)
    except subprocess.TimeoutExpired:
        pytest.fail("{} still running 10 s after the signal\n{}".format(name, proc.log()))
    assert time.monotonic() - t0 < 10.0
    log = proc.log()
    assert code == 0, "{} exit code {}\n{}".format(name, code, log)
    for marker in BAD_MARKERS:
        assert marker not in log, "{}: '{}' in the output\n{}".format(name, marker, log)


# ------------------------------------------------------------------ dock_marker_sim
@pytest.mark.parametrize("signal_name", sorted(SIGNALS))
def test_dock_marker_sim_idle_exits_zero(signal_name, procs, tmp_path):
    proc = start(procs, [executable("dock_marker_sim")], tmp_path / "marker_idle.log")
    assert proc.wait_log("Dock marker emulator started", 20.0), proc.log()
    time.sleep(1.0)
    stop_and_check(proc, SIGNALS[signal_name], "dock_marker_sim (idle)")


@pytest.mark.parametrize("signal_name", sorted(SIGNALS))
def test_dock_marker_sim_publishing_exits_zero(signal_name, procs, tmp_path):
    """Mid-operation: ground truth arrives and detections are being published."""
    proc = start(procs, [executable("dock_marker_sim")], tmp_path / "marker_busy.log")
    assert proc.wait_log("Dock marker emulator started", 20.0), proc.log()
    start(procs, [
        "ros2", "topic", "pub", "-r", "20", "/mujoco/ground_truth", "nav_msgs/msg/Odometry",
        "{header: {frame_id: world}, pose: {pose: {position: {x: 0.5, y: 0.0, z: 0.0}, "
        "orientation: {w: 1.0}}}}"], tmp_path / "gt_pub.log")
    assert ros2_once(["topic", "echo", "--once", "/dock/detected_pose"], 40), \
        "dock_marker_sim published no detection\n" + proc.log()
    stop_and_check(proc, SIGNALS[signal_name], "dock_marker_sim (publishing)")


# ------------------------------------------------------------------ mujoco_sim_node
@pytest.mark.parametrize("signal_name", sorted(SIGNALS))
def test_mujoco_sim_node_running_exits_zero(signal_name, procs, tmp_path):
    """The headless simulator started: /clock flows; the signal ends it cleanly."""
    proc = start(procs, [executable("mujoco_sim_node"), "--ros-args", "-p", "viewer:=false"],
                 tmp_path / "mujoco_idle.log")
    assert proc.wait_log("MuJoCo simulation started", 60.0), proc.log()
    assert ros2_once(["topic", "echo", "--once", "/clock"], 40), proc.log()
    time.sleep(0.5)
    stop_and_check(proc, SIGNALS[signal_name], "mujoco_sim_node (idle)")


@pytest.mark.parametrize("signal_name", sorted(SIGNALS))
def test_mujoco_sim_node_with_wheel_commands_exits_zero(signal_name, procs, tmp_path):
    """Mid-operation: wheel commands arrive while /clock, /scan and ground truth are published."""
    proc = start(procs, [executable("mujoco_sim_node"), "--ros-args", "-p", "viewer:=false"],
                 tmp_path / "mujoco_busy.log")
    assert proc.wait_log("MuJoCo simulation started", 60.0), proc.log()
    start(procs, [
        "ros2", "topic", "pub", "-r", "20", "/mujoco/joint_commands", "sensor_msgs/msg/JointState",
        "{name: [wheel_left_joint, wheel_right_joint], velocity: [2.0, 2.0]}"],
        tmp_path / "cmd_pub.log")
    assert ros2_once(["topic", "echo", "--once", "/scan"], 40), proc.log()
    assert ros2_once(["topic", "echo", "--once", "/mujoco/ground_truth"], 40), proc.log()
    time.sleep(0.5)
    stop_and_check(proc, SIGNALS[signal_name], "mujoco_sim_node (driving)")


# ------------------------------------------------------------------ static guard
@pytest.mark.parametrize("module", ["dock_marker_sim", "mujoco_sim_node"])
def test_shutdown_pattern_is_present(module):
    """The shared pattern: catch the signal exceptions, tolerate RCLError, shutdown only if ok."""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "bumperbot_mujoco",
                        module + ".py")
    with open(path) as f:
        text = f.read()
    assert "ExternalShutdownException" in text
    assert "except RCLError" in text
    assert "if rclpy.ok():" in text
