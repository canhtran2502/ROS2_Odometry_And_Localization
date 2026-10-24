"""
Shared helpers for the iteration 4 robustness tests (mission_manager started as a plain process).

The tests run inside pytest files registered with ament_add_pytest_test; each test starts the C++
or Python mission_manager executable directly, next to a MockNav2 and an optional /clock
publisher, all headless.
"""
import os
import subprocess
import threading
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from bumperbot_msgs.action import ExecuteMission
from bumperbot_msgs.msg import MissionStatus
from mock_nav2 import MockNav2
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock

HERE = os.path.dirname(os.path.abspath(__file__))
STATIONS_FILE = os.path.join(HERE, "data", "stations_test.yaml")
IMPLS = ["cpp", "py"]


def executable(impl):
    name = "mission_manager" if impl == "cpp" else "mission_manager.py"
    return os.path.join(get_package_prefix("bumperbot_navigation"), "lib",
                        "bumperbot_navigation", name)


def wait_until(predicate, timeout, period=0.02):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(period)
    return predicate()


class ClockPublisher(Node):
    """Publishes /clock; ``speed`` sim seconds per wall second, 0 = stalled."""

    def __init__(self, start=1000.0, speed=1.0):
        super().__init__("test_clock_publisher")
        self.sim = start
        self.speed = speed
        self.pub = self.create_publisher(Clock, "/clock", 10)
        self.period = 0.05
        self.create_timer(self.period, self.tick)

    def tick(self):
        self.sim += self.period * self.speed
        msg = Clock()
        msg.clock.sec = int(self.sim)
        msg.clock.nanosec = int((self.sim - int(self.sim)) * 1e9)
        self.pub.publish(msg)


class Harness:
    """Mock Nav2, status recorder, action client (and optional /clock) on one executor."""

    def __init__(self, with_clock=False, clock_speed=1.0):
        if not rclpy.ok():
            rclpy.init()
        self.mock = MockNav2(STATIONS_FILE)
        self.node = Node("robust_test_client")
        self.clock = ClockPublisher(speed=clock_speed) if with_clock else None
        self.lock = threading.Lock()
        self.statuses = []  # (wall time, MissionStatus)
        qos = QoSProfile(depth=500, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(MissionStatus, "/mission/status", self.on_status, qos)
        self.client = ActionClient(self.node, ExecuteMission, "execute_mission")
        self.executor = MultiThreadedExecutor(num_threads=6)
        for n in (self.mock, self.node, self.clock):
            if n is not None:
                self.executor.add_node(n)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        self.proc = None
        self.log_path = None
        self.log_file = None

    def on_status(self, msg):
        with self.lock:
            self.statuses.append((time.monotonic(), msg))

    def status_states(self):
        with self.lock:
            return [m.state for _, m in self.statuses]

    def status_times(self):
        with self.lock:
            return [t for t, _ in self.statuses]

    def start_node(self, impl, tmp_path, params=None, tag="node"):
        args = [executable(impl), "--ros-args"]
        merged = {"stations_file": STATIONS_FILE}
        merged.update(params or {})
        for key, value in merged.items():
            if isinstance(value, bool):
                value = "true" if value else "false"
            args += ["-p", "{}:={}".format(key, value)]
        self.log_path = os.path.join(str(tmp_path), "{}_{}.log".format(tag, impl))
        self.log_file = open(self.log_path, "w")
        self.proc = subprocess.Popen(args, stdout=self.log_file, stderr=subprocess.STDOUT,
                                     start_new_session=True)
        return self.proc

    def log(self):
        if self.log_file is not None:
            self.log_file.flush()
        with open(self.log_path) as f:
            return f.read()

    def wait_ready(self, timeout=30.0):
        return self.client.wait_for_server(timeout_sec=timeout)

    def send(self, stations, timeout=20.0):
        goal = ExecuteMission.Goal()
        goal.stations = list(stations)
        fut = self.client.send_goal_async(goal)
        assert wait_until(fut.done, timeout), "no goal response"
        handle = fut.result()
        assert handle.accepted, "mission goal rejected"
        return handle

    def result(self, handle, timeout=60.0):
        fut = handle.get_result_async()
        assert wait_until(fut.done, timeout), "mission did not finish in time"
        return fut.result()

    def stop_node(self):
        if self.proc is not None and self.proc.poll() is None:
            import signal
            try:
                os.killpg(self.proc.pid, signal.SIGTERM)
                self.proc.wait(timeout=10)
            except Exception:
                try:
                    os.killpg(self.proc.pid, signal.SIGKILL)
                    self.proc.wait(timeout=5)
                except Exception:
                    pass
        if self.log_file is not None:
            self.log_file.close()

    def close(self):
        self.stop_node()
        self.mock.close()
        self.executor.shutdown()
        self.thread.join(timeout=5)
        for n in (self.mock, self.node, self.clock):
            if n is not None:
                n.destroy_node()
        # fresh context for the next test: goal threads of the mock must not outlive this one
        rclpy.shutdown()
