"""AC1 (interfaces) and AC13 (simulator independence, parameter parity, ASCII comments)."""
import glob
import os
import re
import signal
import subprocess

import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.abspath(os.path.join(HERE, ".."))
SRC = os.path.abspath(os.path.join(PKG, ".."))

LOGIC_FILES = [
    os.path.join(PKG, "bumperbot_navigation", "mission_logic.py"),
    os.path.join(PKG, "src", "mission_logic.cpp"),
    os.path.join(PKG, "include", "bumperbot_navigation", "mission_logic.hpp"),
]
NODE_FILES = [
    os.path.join(PKG, "bumperbot_navigation", "mission_manager.py"),
    os.path.join(PKG, "src", "mission_manager.cpp"),
    os.path.join(PKG, "include", "bumperbot_navigation", "mission_manager.hpp"),
]
PY_NODE, CPP_NODE = NODE_FILES[0], NODE_FILES[1]


def interface_show(name):
    return subprocess.run(["ros2", "interface", "show", name], capture_output=True, text=True,
                          timeout=60)


# ------------------------------------------------------------------ AC1
@pytest.mark.parametrize("name", [
    "bumperbot_msgs/action/ExecuteMission",
    "bumperbot_msgs/msg/MissionStatus",
    "bumperbot_msgs/action/Fibonacci",
    "bumperbot_msgs/srv/AddTwoInts",
    "bumperbot_msgs/srv/GetTransform",
])
def test_interfaces_show(name):
    res = interface_show(name)
    assert res.returncode == 0, res.stderr
    assert res.stdout.strip() != ""


def test_interfaces_execute_mission_fields():
    out = interface_show("bumperbot_msgs/action/ExecuteMission").stdout
    goal, result, feedback = [p for p in out.split("\n---\n")]
    assert re.search(r"^string\[\] stations\b", goal, re.M)
    for const, val in (("COMPLETED", 0), ("COMPLETED_WITH_SKIPS", 1), ("ABORTED", 2),
                       ("CANCELED", 3), ("DOCK_FAILED", 4)):
        assert re.search(r"^uint8 OUTCOME_%s=%d\b" % (const, val), result, re.M), const
    for decl in ("uint8 outcome", "bool docked", "string[] visited", "string[] skipped",
                 "string message"):
        assert re.search(r"^%s\b" % re.escape(decl), result, re.M), decl
    for decl in ("string state", "string current_station", "int32 current_index",
                 "uint32 total_stations", "uint32 attempt", "float32 progress",
                 "float32 distance_remaining"):
        assert re.search(r"^%s\b" % re.escape(decl), feedback, re.M), decl


def test_interfaces_mission_status_fields():
    out = interface_show("bumperbot_msgs/msg/MissionStatus").stdout
    for st in ("IDLE", "NAVIGATING", "DWELLING", "RETURNING_TO_DOCK", "COMPLETED", "ABORTED",
               "CANCELED", "DOCK_FAILED"):
        assert re.search(r"^string STATE_%s=%s$" % (st, st), out, re.M), st
    for decl in ("builtin_interfaces/Time stamp", "string state", "string current_station",
                 "int32 current_index", "uint32 total_stations", "uint32 attempt",
                 "float32 progress", "string[] stations", "string[] visited",
                 "string[] skipped", "string message"):
        assert re.search(r"^%s$" % re.escape(decl), out, re.M), decl


def test_interfaces_generated_python_types():
    from bumperbot_msgs.action import ExecuteMission, Fibonacci  # noqa: F401
    from bumperbot_msgs.msg import MissionStatus
    from bumperbot_msgs.srv import AddTwoInts, GetTransform  # noqa: F401
    assert ExecuteMission.Goal.get_fields_and_field_types() == {"stations": "sequence<string>"}
    assert ExecuteMission.Result.OUTCOME_DOCK_FAILED == 4
    assert MissionStatus.STATE_RETURNING_TO_DOCK == "RETURNING_TO_DOCK"


# ------------------------------------------------------------------ AC13
def test_sim_independence_logic_and_nodes():
    pattern = re.compile(r"mujoco|gazebo|\bgz\b|/clock", re.I)
    offenders = []
    for path in LOGIC_FILES + NODE_FILES:
        assert os.path.isfile(path), path
        with open(path, encoding="utf-8") as f:
            for n, line in enumerate(f, 1):
                if pattern.search(line):
                    offenders.append(f"{os.path.basename(path)}:{n}: {line.strip()}")
    assert not offenders, offenders


def _py_params(text):
    out = {}
    for m in re.finditer(r'declare_parameter\(\s*"(\w+)"\s*,\s*([^)]+?)\s*\)', text):
        out[m.group(1)] = m.group(2)
    return out


def _cpp_params(text):
    out = {}
    for m in re.finditer(r'declare_parameter<[\w:]+>\(\s*"(\w+)"\s*,\s*([^)]+?)\s*\)', text):
        out[m.group(1)] = m.group(2)
    return out


def _norm(v):
    v = v.strip()
    if v in ("True", "true"):
        return True
    if v in ("False", "false"):
        return False
    if v.startswith('"'):
        return v.strip('"')
    return float(v)


def test_parameter_parity():
    with open(PY_NODE) as f:
        py = {k: _norm(v) for k, v in _py_params(f.read()).items()}
    with open(CPP_NODE) as f:
        cpp = {k: _norm(v) for k, v in _cpp_params(f.read()).items()}
    assert len(py) == 16, py.keys()
    assert py["clock_stall_timeout"] == 30.0 and cpp["clock_stall_timeout"] == 30.0
    assert py == cpp, {k: (py.get(k), cpp.get(k)) for k in set(py) | set(cpp)
                       if py.get(k) != cpp.get(k)}


def test_parameter_defaults_match_plan_and_yaml():
    with open(PY_NODE) as f:
        py = {k: _norm(v) for k, v in _py_params(f.read()).items()}
    plan = {"stations_file": "", "nav_action_name": "navigate_to_pose", "global_frame": "map",
            "robot_frame": "base_footprint", "goal_tolerance": 0.2, "max_retries": 2,
            "retry_delay": 1.0, "max_consecutive_failures": 2, "dock_max_retries": 3,
            "return_to_dock": True, "nav_server_timeout": 10.0, "nav_goal_timeout": 300.0,
            "tf_timeout": 1.0, "clock_stall_timeout": 30.0, "status_rate": 2.0,
            "behavior_tree": ""}
    assert py == plan
    with open(os.path.join(PKG, "config", "mission_manager.yaml")) as f:
        cfg = yaml.safe_load(f)["mission_manager"]["ros__parameters"]
    assert cfg == plan


def _new_source_files():
    files = []
    for pattern in ("bumperbot_navigation/*.py", "src/*.cpp", "include/**/*.hpp",
                    "config/*.yaml", "launch/*.py", "test/*.py", "test/*.cpp",
                    "test/data/*.yaml", "CMakeLists.txt", "package.xml"):
        files += glob.glob(os.path.join(PKG, pattern), recursive=True)
    files += [
        os.path.join(SRC, "bumperbot_bringup", "launch", "mission_sim.launch.py"),
        os.path.join(SRC, "bumperbot_bringup", "test", "test_mission_launch_config.py"),
        os.path.join(SRC, "bumperbot_bringup", "test", "test_mission_sim_mujoco.launch.py"),
        os.path.join(SRC, "bumperbot_msgs", "action", "ExecuteMission.action"),
        os.path.join(SRC, "bumperbot_msgs", "msg", "MissionStatus.msg"),
        os.path.join(SRC, "bumperbot_controller", "config", "twist_mux_topics.yaml"),
    ]
    return files


def test_ascii_comments():
    files = _new_source_files()
    assert len(files) > 20
    bad = []
    for path in files:
        assert os.path.isfile(path), path
        with open(path, "rb") as f:
            for n, line in enumerate(f, 1):
                try:
                    line.decode("ascii")
                except UnicodeDecodeError:
                    bad.append(f"{os.path.relpath(path, SRC)}:{n}")
    assert not bad, bad


# ------------------------------------------------------------------ review fix: frame check
@pytest.mark.parametrize("executable", ["mission_manager", "mission_manager.py"])
def test_node_refuses_start_when_station_frame_differs_from_global_frame(executable):
    fixture = os.path.join(HERE, "data", "stations_other_frame.yaml")
    cmd = ["ros2", "run", "bumperbot_navigation", executable, "--ros-args",
           "-p", "stations_file:=" + fixture, "-p", "global_frame:=map"]
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
    out = proc.stdout + proc.stderr
    assert proc.returncode != 0, "node started although frame_id != global_frame\n" + out
    assert "global_frame" in out and "odom" in out, out


@pytest.mark.parametrize("executable", ["mission_manager", "mission_manager.py"])
def test_node_starts_when_station_frame_matches_global_frame(executable):
    """Control for the test above: with a matching global_frame the node keeps running."""
    fixture = os.path.join(HERE, "data", "stations_other_frame.yaml")
    cmd = ["ros2", "run", "bumperbot_navigation", executable, "--ros-args",
           "-p", "stations_file:=" + fixture, "-p", "global_frame:=odom"]
    # own process group: `ros2 run` does not forward the signal to the node it starts
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                            start_new_session=True)
    try:
        with pytest.raises(subprocess.TimeoutExpired):
            proc.wait(timeout=4.0)
    finally:
        os.killpg(proc.pid, signal.SIGTERM)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(timeout=5)
