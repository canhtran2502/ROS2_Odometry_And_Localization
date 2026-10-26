"""
Process-level helpers for the dock_manager tests.

DockProcess starts a dock_manager executable (C++ or Python) as a plain process in its own process
group, so that signals and orphan nodes never leak into other tests. Rig bundles a MockDockWorld,
an optional test /clock and the executor thread.
"""
import os
import signal
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from mock_dock_world import MockDockWorld, spin_in_thread
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "data")
STATIONS_FILE = os.path.join(DATA, "stations_test.yaml")
IMPLS = ["cpp", "py"]

# Parameters of the node under test in all node tests (see the plan)
TEST_PARAMS = {
    "stations_file": STATIONS_FILE,
    "stall_timeout": 2.0,
    "safety_stop_timeout": 3.0,
    "detection_timeout": 2.0,
    "nav_server_timeout": 3.0,
    "nav_goal_timeout": 10.0,
    "use_sim_time": False,
}


def executable(impl):
    name = "dock_manager" if impl == "cpp" else "dock_manager.py"
    return os.path.join(get_package_prefix("bumperbot_navigation"), "lib",
                        "bumperbot_navigation", name)


def wait_until(predicate, timeout, period=0.02):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(period)
    return bool(predicate())


def format_param(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


class DockProcess:
    """One dock_manager process with its own process group and a log file."""

    def __init__(self, impl, tmp_dir, params=None, tag="node", namespace=None, name=None):
        self.impl = impl
        merged = dict(TEST_PARAMS)
        merged.update(params or {})
        merged = {k: v for k, v in merged.items() if v is not None}   # None drops a parameter
        self.params = merged
        args = [executable(impl), "--ros-args"]
        for key, value in merged.items():
            args += ["-p", "{}:={}".format(key, format_param(value))]
        if namespace:
            args += ["-r", "__ns:=" + namespace]
        if name:
            args += ["-r", "__node:=" + name]
        self.log_path = os.path.join(str(tmp_dir), "{}_{}.log".format(tag, impl))
        self.log_file = open(self.log_path, "w")
        self.t_start = time.monotonic()
        self.proc = subprocess.Popen(args, stdout=self.log_file, stderr=subprocess.STDOUT,
                                     start_new_session=True)

    @property
    def pid(self):
        return self.proc.pid

    def alive(self):
        return self.proc.poll() is None

    def log(self):
        if not self.log_file.closed:
            self.log_file.flush()
        with open(self.log_path) as f:
            return f.read()

    def send_signal(self, signum):
        os.kill(self.proc.pid, signum)

    def wait_exit(self, timeout):
        """Return the exit code, or None when the process is still running after the timeout."""
        try:
            return self.proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            return None

    def stop(self, timeout=12.0):
        """SIGTERM the process group; returns the exit code (SIGKILL when it does not exit)."""
        code = self.proc.poll()
        if code is None:
            try:
                os.kill(self.proc.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            code = self.wait_exit(timeout)
            if code is None:
                try:
                    os.killpg(self.proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                self.proc.wait(timeout=5)
                code = "killed"
        self.log_file.close()
        return code


class ClockPublisher(Node):
    """Publishes /clock; ``speed`` sim seconds per wall second, 0 means stalled."""

    def __init__(self, start=1000.0, speed=1.0):
        super().__init__("test_clock_publisher")
        self.sim = start
        self.speed = speed
        self.period = 0.02
        self.pub = self.create_publisher(Clock, "/clock", 10)
        self.create_timer(self.period, self.tick)
        self.last = time.monotonic()

    def tick(self):
        now = time.monotonic()
        self.sim += (now - self.last) * self.speed
        self.last = now
        msg = Clock()
        msg.clock.sec = int(self.sim)
        msg.clock.nanosec = int((self.sim - int(self.sim)) * 1e9)
        self.pub.publish(msg)


class Rig:
    """A MockDockWorld (optionally on sim time with a /clock) spinning in a background thread."""

    def __init__(self, sim_time=False, clock_speed=1.0):
        if not rclpy.ok():
            rclpy.init()
        self.clock = ClockPublisher(speed=clock_speed) if sim_time else None
        self.world = MockDockWorld(use_sim_time=sim_time)
        nodes = [self.world] + ([self.clock] if self.clock is not None else [])
        self.executor, self.thread = spin_in_thread(nodes)
        self.proc = None

    def start(self, impl, tmp_dir, params=None, tag="node"):
        self.proc = DockProcess(impl, tmp_dir, params, tag)
        return self.proc

    def wait_ready(self, timeout=30.0):
        """Wait for the DockRobot server and the first /dock/status message."""
        if not self.world.client.wait_for_server(timeout_sec=timeout):
            return False
        return wait_until(lambda: self.world.last_status() is not None, 10.0)

    def stop_node(self):
        code = None
        if self.proc is not None:
            code = self.proc.stop()
            self.proc = None
        return code

    def close(self):
        self.stop_node()
        self.world.close()
        self.executor.shutdown()
        self.thread.join(timeout=5)
        for node in (self.world, self.clock):
            if node is not None:
                node.destroy_node()
        rclpy.shutdown()
