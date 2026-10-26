"""
AC1 (interfaces) and AC20 (static checks of the docking sources).

AC17 (static part): the simulation-only nodes appear only in dock_sim.launch.py.
"""
import glob
import os
import re
import subprocess

import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.abspath(os.path.join(HERE, ".."))
SRC = os.path.abspath(os.path.join(PKG, ".."))

PY_NODE = os.path.join(PKG, "bumperbot_navigation", "dock_manager.py")
CPP_NODE = os.path.join(PKG, "src", "dock_manager.cpp")

# Source files without any simulator knowledge (AC20)
SIM_FREE_FILES = [
    os.path.join(PKG, "bumperbot_navigation", "dock_manager.py"),
    os.path.join(PKG, "src", "dock_manager.cpp"),
    os.path.join(PKG, "include", "bumperbot_navigation", "dock_manager.hpp"),
    os.path.join(PKG, "bumperbot_navigation", "dock_logic.py"),
    os.path.join(PKG, "src", "dock_logic.cpp"),
    os.path.join(PKG, "include", "bumperbot_navigation", "dock_logic.hpp"),
    os.path.join(PKG, "bumperbot_navigation", "battery_monitor.py"),
    os.path.join(PKG, "bumperbot_navigation", "battery_logic.py"),
]

# Parameter table of the plan: name -> default
PLAN_PARAMS = {
    "stations_file": "", "dock_station": "", "nav_action_name": "navigate_to_pose",
    "global_frame": "map", "odom_frame": "odom", "robot_frame": "base_footprint",
    "cmd_vel_topic": "dock_vel", "detection_topic": "dock/detected_pose",
    "safety_stop_topic": "safety_stop", "battery_topic": "battery_state",
    "mission_status_topic": "mission/status", "dock_backwards": True,
    "staging_distance": 0.5, "staging_tolerance": 0.25, "staging_yaw_tolerance": 0.6,
    "nav_max_retries": 2, "nav_server_timeout": 10.0, "nav_goal_timeout": 180.0,
    "detection_mode": "optional", "detection_timeout": 3.0, "detection_max_age": 0.5,
    "detection_max_jump": 0.3, "detection_filter_alpha": 0.3, "detection_offset_x": 0.0,
    "detection_offset_y": 0.0, "detection_offset_yaw": 0.0, "align_position_tolerance": 0.02,
    "align_yaw_tolerance": 0.03, "dock_position_tolerance": 0.03, "dock_yaw_tolerance": 0.05,
    "max_linear_speed": 0.15, "max_angular_speed": 0.6, "approach_speed": 0.05,
    "slow_down_distance": 0.15, "k_lateral": 2.0, "k_heading": 2.0,
    "max_insert_lateral_error": 0.08, "max_insert_yaw_error": 0.3, "max_retries": 2,
    "align_timeout": 30.0, "docking_timeout": 40.0, "undocking_timeout": 30.0,
    "stall_timeout": 5.0, "safety_stop_timeout": 10.0, "settle_time": 0.5,
    "undock_detect_distance": 0.10, "tf_timeout": 1.0, "tf_stale_timeout": 0.5,
    "control_rate": 20.0, "status_rate": 2.0, "clock_stall_timeout": 5.0,
}


def interface_show(name):
    return subprocess.run(["ros2", "interface", "show", name], capture_output=True, text=True,
                          timeout=60)


# ------------------------------------------------------------------ AC1
@pytest.mark.parametrize("name", [
    "bumperbot_msgs/action/DockRobot",
    "bumperbot_msgs/msg/DockStatus",
    "bumperbot_msgs/action/ExecuteMission",
    "bumperbot_msgs/msg/MissionStatus",
    "bumperbot_msgs/action/Fibonacci",
    "bumperbot_msgs/srv/AddTwoInts",
    "bumperbot_msgs/srv/GetTransform",
])
def test_interfaces_interfaces_show(name):
    res = interface_show(name)
    assert res.returncode == 0, res.stderr
    assert res.stdout.strip() != ""


def test_interfaces_dock_robot_fields():
    out = interface_show("bumperbot_msgs/action/DockRobot").stdout
    goal, result, feedback = out.split("\n---\n")
    assert re.search(r"^uint8 COMMAND_DOCK=0\b", goal, re.M)
    assert re.search(r"^uint8 COMMAND_UNDOCK=1\b", goal, re.M)
    assert re.search(r"^uint8 command\b", goal, re.M)
    for const, val in (("DOCKED", 0), ("UNDOCKED", 1), ("ALREADY_DOCKED", 2), ("CANCELED", 3),
                       ("STAGING_FAILED", 4), ("DETECTION_FAILED", 5), ("DOCKING_FAILED", 6),
                       ("UNDOCKING_FAILED", 7), ("ERROR", 8)):
        assert re.search(r"^uint8 OUTCOME_%s=%d\b" % (const, val), result, re.M), const
    for decl in ("uint8 outcome", "bool docked", "uint32 attempts", "float32 position_error",
                 "float32 yaw_error", "string message"):
        assert re.search(r"^%s\b" % re.escape(decl), result, re.M), decl
    for decl in ("string state", "uint32 attempt", "float32 distance_to_dock",
                 "float32 yaw_error", "bool detection_used"):
        assert re.search(r"^%s\b" % re.escape(decl), feedback, re.M), decl


def test_interfaces_dock_status_fields():
    out = interface_show("bumperbot_msgs/msg/DockStatus").stdout
    for st in ("UNDOCKED", "NAVIGATING_TO_STAGING", "WAITING_FOR_DETECTION", "ALIGNING",
               "DOCKING", "DOCKED", "CHARGING", "UNDOCKING", "FAILED"):
        assert re.search(r"^string STATE_%s=%s$" % (st, st), out, re.M), st
    for decl in ("builtin_interfaces/Time stamp", "string state", "bool docked", "bool charging",
                 "float32 battery_percentage", "uint32 attempt", "float32 position_error",
                 "float32 yaw_error", "bool detection_used", "string frame_id",
                 "geometry_msgs/Pose2D staging_pose", "geometry_msgs/Pose2D dock_pose",
                 "string message"):
        assert re.search(r"^%s\b" % re.escape(decl), out, re.M), decl


def test_interfaces_generated_python_types():
    from bumperbot_msgs.action import DockRobot, ExecuteMission, Fibonacci  # noqa: F401
    from bumperbot_msgs.msg import DockStatus, MissionStatus  # noqa: F401
    from bumperbot_msgs.srv import AddTwoInts, GetTransform  # noqa: F401
    assert DockRobot.Goal.get_fields_and_field_types() == {"command": "uint8"}
    assert DockRobot.Result.get_fields_and_field_types() == {
        "outcome": "uint8", "docked": "boolean", "attempts": "uint32",
        "position_error": "float", "yaw_error": "float", "message": "string"}
    assert DockRobot.Feedback.get_fields_and_field_types() == {
        "state": "string", "attempt": "uint32", "distance_to_dock": "float",
        "yaw_error": "float", "detection_used": "boolean"}
    assert list(DockStatus.get_fields_and_field_types()) == [
        "stamp", "state", "docked", "charging", "battery_percentage", "attempt",
        "position_error", "yaw_error", "detection_used", "frame_id", "staging_pose",
        "dock_pose", "message"]


def test_interfaces_files_registered_in_cmake():
    with open(os.path.join(SRC, "bumperbot_msgs", "CMakeLists.txt")) as f:
        text = f.read()
    assert "action/DockRobot.action" in text and "msg/DockStatus.msg" in text


# ------------------------------------------------------------------ AC20: simulator independence
def test_sim_independence_dock_sources():
    pattern = re.compile(r"mujoco|gazebo|\bgz\b|/clock", re.I)
    offenders = []
    for path in SIM_FREE_FILES:
        assert os.path.isfile(path), path
        with open(path, encoding="utf-8") as f:
            for n, line in enumerate(f, 1):
                if pattern.search(line):
                    offenders.append("{}:{}: {}".format(os.path.basename(path), n, line.strip()))
    assert not offenders, offenders


# ------------------------------------------------------------------ AC20: ASCII comments
def new_source_files():
    files = []
    for pattern in ("bumperbot_navigation/*.py", "src/*.cpp", "include/**/*.hpp",
                    "config/*.yaml", "launch/*.py", "test/*.py", "test/*.cpp",
                    "test/data/*.yaml", "CMakeLists.txt", "package.xml"):
        files += glob.glob(os.path.join(PKG, pattern), recursive=True)
    files += [
        os.path.join(SRC, "bumperbot_msgs", "action", "DockRobot.action"),
        os.path.join(SRC, "bumperbot_msgs", "msg", "DockStatus.msg"),
        os.path.join(SRC, "bumperbot_msgs", "CMakeLists.txt"),
        os.path.join(SRC, "bumperbot_controller", "config", "twist_mux_topics.yaml"),
        os.path.join(SRC, "bumperbot_mujoco", "bumperbot_mujoco", "dock_marker_sim.py"),
        os.path.join(SRC, "bumperbot_mujoco", "bumperbot_mujoco", "mujoco_sim_node.py"),
        os.path.join(SRC, "bumperbot_mujoco", "config", "dock_marker_sim.yaml"),
        os.path.join(SRC, "bumperbot_mujoco", "setup.py"),
        os.path.join(SRC, "bumperbot_mujoco", "package.xml"),
        os.path.join(SRC, "bumperbot_bringup", "launch", "dock_sim.launch.py"),
        os.path.join(SRC, "bumperbot_bringup", "CMakeLists.txt"),
        os.path.join(SRC, "bumperbot_bringup", "package.xml"),
    ]
    files += glob.glob(os.path.join(SRC, "bumperbot_mujoco", "test", "test_dock_*.py"))
    files += glob.glob(os.path.join(SRC, "bumperbot_bringup", "test", "test_dock_*.py"))
    return files


def test_ascii_only_sources():
    files = new_source_files()
    assert len(files) > 40
    bad = []
    for path in files:
        assert os.path.isfile(path), path
        with open(path, "rb") as f:
            for n, line in enumerate(f, 1):
                try:
                    line.decode("ascii")
                except UnicodeDecodeError:
                    bad.append("{}:{}".format(os.path.relpath(path, SRC), n))
    assert not bad, bad


# ------------------------------------------------------------------ AC20: parameter parity
def py_params(text):
    out = {}
    for m in re.finditer(r'declare_parameter\(\s*"(\w+)"\s*,\s*([^)]+?)\s*\)', text):
        out[m.group(1)] = m.group(2)
    return out


def cpp_params(text):
    out = {}
    for m in re.finditer(r'declare_parameter<[\w:]+>\(\s*"(\w+)"\s*,\s*([^)]+?)\s*\)', text):
        out[m.group(1)] = m.group(2)
    return out


def norm(value):
    value = value.strip()
    if value in ("True", "true"):
        return True
    if value in ("False", "false"):
        return False
    if value.startswith('"'):
        return value.strip('"')
    return float(value)


def declared(path, finder):
    with open(path) as f:
        return {k: norm(v) for k, v in finder(f.read()).items()}


def values_equal(a, b):
    if isinstance(a, bool) or isinstance(b, bool):
        return a is b
    if isinstance(a, str) or isinstance(b, str):
        return a == b
    return abs(float(a) - float(b)) < 1e-12


def diff(left, right):
    keys = set(left) | set(right)
    return {k: (left.get(k), right.get(k)) for k in keys
            if k not in left or k not in right or not values_equal(left[k], right[k])}


def test_parameter_parity_py_cpp():
    py = declared(PY_NODE, py_params)
    cpp = declared(CPP_NODE, cpp_params)
    assert len(py) == 51 and len(cpp) == 51, (len(py), len(cpp))
    assert not diff(py, cpp), diff(py, cpp)


def test_parameter_defaults_match_plan():
    py = declared(PY_NODE, py_params)
    assert not diff(py, PLAN_PARAMS), diff(py, PLAN_PARAMS)
    cpp = declared(CPP_NODE, cpp_params)
    assert not diff(cpp, PLAN_PARAMS), diff(cpp, PLAN_PARAMS)


def test_parameter_defaults_match_yaml():
    with open(os.path.join(PKG, "config", "dock_manager.yaml")) as f:
        cfg = yaml.safe_load(f)["dock_manager"]["ros__parameters"]
    assert not diff(cfg, PLAN_PARAMS), diff(cfg, PLAN_PARAMS)


def test_battery_parameter_defaults_match_plan():
    with open(os.path.join(PKG, "config", "battery_monitor.yaml")) as f:
        cfg = yaml.safe_load(f)["battery_monitor"]["ros__parameters"]
    plan = {"low_threshold": 0.20, "full_threshold": 0.95, "rearm_hysteresis": 0.05,
            "battery_timeout": 5.0, "mission_cancel_timeout": 180.0, "dock_retry_delay": 5.0,
            "dock_request_timeout": 120.0, "max_dock_attempts": 3, "undock_when_full": True,
            "update_rate": 2.0, "clock_stall_timeout": 5.0}
    assert not diff(cfg, plan), diff(cfg, plan)
    declared_in_node = declared(os.path.join(PKG, "bumperbot_navigation", "battery_monitor.py"),
                                py_params)
    assert not diff(declared_in_node, plan), diff(declared_in_node, plan)


def test_battery_sim_config_marked_simulation_only():
    path = os.path.join(PKG, "config", "battery_sim.yaml")
    with open(path) as f:
        first = f.readline()
    assert "simulation only" in first.lower()
    with open(os.path.join(PKG, "bumperbot_navigation", "battery_sim.py")) as f:
        head = f.read(400)
    assert "SIMULATION ONLY" in head


# ------------------------------------------------------------------ AC17 (static)
def test_sim_only_nodes_only_in_dock_sim_launch():
    bringup = os.path.join(SRC, "bumperbot_bringup", "launch", "dock_sim.launch.py")
    with open(bringup) as f:
        text = f.read()
    assert "battery_sim" in text and "dock_marker_sim" in text
    forbidden = [
        os.path.join(PKG, "launch", "docking.launch.py"),
        os.path.join(PKG, "launch", "navigation.launch.py"),
        os.path.join(SRC, "bumperbot_bringup", "launch", "real_robot.launch.py"),
        os.path.join(SRC, "bumperbot_bringup", "launch", "simulated_robot.launch.py"),
        os.path.join(SRC, "bumperbot_bringup", "launch", "mission_sim.launch.py"),
    ]
    forbidden += [p for p in glob.glob(os.path.join(SRC, "bumperbot_firmware", "**", "*"),
                                       recursive=True)
                  if os.path.isfile(p) and "/test/" not in p and "__pycache__" not in p
                  and not p.endswith((".pyc", ".so", ".o"))]
    offenders = []
    for path in forbidden:
        try:
            with open(path, encoding="utf-8") as f:
                content = f.read()
        except UnicodeDecodeError:
            continue
        if "battery_sim" in content or "dock_marker_sim" in content:
            offenders.append(os.path.relpath(path, SRC))
    assert not offenders, offenders
