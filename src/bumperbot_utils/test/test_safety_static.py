"""AC1 (interfaces) and AC15 (static checks) of the safety supervisor."""
import os
import re
import subprocess

import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.abspath(os.path.join(HERE, ".."))
SRC = os.path.abspath(os.path.join(PKG, ".."))
ROOT = os.path.abspath(os.path.join(SRC, ".."))

# Plan: bumperbot_msgs/msg/SafetyStatus.msg
EXPECTED_CONSTANTS = {
    "STATE_INITIALIZING": "INITIALIZING", "STATE_ACTIVE": "ACTIVE",
    "STATE_DOCK_APPROACH": "DOCK_APPROACH", "STATE_FAULT": "FAULT", "STATE_ESTOP": "ESTOP",
    "STATE_SHUTDOWN": "SHUTDOWN", "ZONE_CLEAR": "CLEAR", "ZONE_WARNING": "WARNING",
    "ZONE_STOP": "STOP", "ZONE_CONTACT": "CONTACT", "ZONE_UNKNOWN": "UNKNOWN",
}
EXPECTED_FIELDS = [
    ("stamp", "builtin_interfaces/Time"), ("state", "string"), ("enforcement", "string"),
    ("active_zone", "string"), ("front_zone", "string"), ("rear_zone", "string"),
    ("reason", "string"), ("locked", "boolean"), ("braking", "boolean"),
    ("estop_latched", "boolean"), ("dock_profile_active", "boolean"),
    ("nearest_obstacle_distance", "float"), ("nearest_obstacle_bearing", "float"),
    ("front_clearance", "float"), ("rear_clearance", "float"), ("max_forward_speed", "float"),
    ("max_reverse_speed", "float"), ("scan_age", "float"), ("odom_age", "float"),
    ("cmd_age", "float"),
]
OLD_INTERFACES = [
    "bumperbot_msgs/action/DockRobot", "bumperbot_msgs/msg/DockStatus",
    "bumperbot_msgs/action/ExecuteMission", "bumperbot_msgs/msg/MissionStatus",
    "bumperbot_msgs/action/Fibonacci", "bumperbot_msgs/srv/AddTwoInts",
    "bumperbot_msgs/srv/GetTransform",
]


def show(name):
    return subprocess.run(["ros2", "interface", "show", name], capture_output=True, text=True,
                          timeout=60)


# ------------------------------------------------------------------ AC1: interfaces
def test_interfaces_safety_status_shows():
    res = show("bumperbot_msgs/msg/SafetyStatus")
    assert res.returncode == 0, res.stderr
    for name, value in EXPECTED_CONSTANTS.items():
        assert re.search(r"string\s+%s\s*=\s*%s\b" % (name, value), res.stdout), (
            "constant {}={} missing:\n{}".format(name, value, res.stdout))


def test_interfaces_safety_status_fields_exact():
    from bumperbot_msgs.msg import SafetyStatus
    fields = SafetyStatus.get_fields_and_field_types()
    assert list(fields.items()) == EXPECTED_FIELDS
    for name, value in EXPECTED_CONSTANTS.items():
        assert getattr(SafetyStatus, name) == value, name
    constants = {n for n in dir(SafetyStatus) if n.startswith(("STATE_", "ZONE_"))}
    assert constants == set(EXPECTED_CONSTANTS), constants ^ set(EXPECTED_CONSTANTS)


def test_interfaces_old_interfaces_still_show():
    for name in OLD_INTERFACES:
        res = show(name)
        assert res.returncode == 0, "{}: {}".format(name, res.stderr)
        assert res.stdout.strip(), name


def test_interfaces_msg_registered_in_cmake():
    with open(os.path.join(SRC, "bumperbot_msgs", "CMakeLists.txt")) as f:
        assert "msg/SafetyStatus.msg" in f.read()


# ------------------------------------------------------------------ AC15: static checks
NODE_FILES = [
    "src/safety_supervisor.cpp", "include/bumperbot_utils/safety_supervisor.hpp",
    "bumperbot_utils/safety_supervisor.py", "src/safety_logic.cpp",
    "include/bumperbot_utils/safety_logic.hpp", "bumperbot_utils/safety_logic.py",
]
FORBIDDEN = re.compile(r"mujoco|gazebo|\bgz\b|/clock", re.IGNORECASE)


def read(rel):
    with open(os.path.join(PKG, rel)) as f:
        return f.read()


@pytest.mark.parametrize("rel", NODE_FILES)
def test_static_no_simulator_tokens(rel):
    hits = [(n + 1, line.strip()) for n, line in enumerate(read(rel).splitlines())
            if FORBIDDEN.search(line)]
    assert not hits, "{} mentions a simulator: {}".format(rel, hits[:5])


def feature_files():
    """Return the new and modified files of the feature (committed or in the work tree)."""
    base = subprocess.run(
        ["git", "log", "--format=%H", "--", ".claude/features/safety-supervisor/plan.md"],
        cwd=ROOT, capture_output=True, text=True).stdout.split()
    assert base, "plan commit not found"
    changed = subprocess.run(["git", "diff", "--name-only", base[-1], "--", "src"], cwd=ROOT,
                             capture_output=True, text=True).stdout.split()
    untracked = subprocess.run(["git", "ls-files", "--others", "--exclude-standard", "src"],
                               cwd=ROOT, capture_output=True, text=True).stdout.split()
    files = sorted(set(changed + untracked))
    return [f for f in files if f.endswith((".py", ".cpp", ".hpp", ".yaml", ".xacro", ".msg"))
            and os.path.exists(os.path.join(ROOT, f))]


def comment_text(path, line):
    if path.endswith((".py", ".yaml")):
        match = re.search(r"#.*$", line)
        return match.group(0) if match else ""
    if path.endswith((".cpp", ".hpp")):
        match = re.search(r"//.*$|/\*.*", line)
        return match.group(0) if match else ""
    return ""


def test_static_comments_are_ascii():
    files = feature_files()
    assert any(f.endswith("safety_supervisor.py") for f in files), files
    bad = []
    for rel in files:
        with open(os.path.join(ROOT, rel), encoding="utf-8") as f:
            for n, line in enumerate(f, 1):
                text = comment_text(rel, line)
                if any(ord(c) > 127 for c in text):
                    bad.append("{}:{}".format(rel, n))
    assert not bad, "non-ASCII comments: {}".format(bad[:10])


def test_static_sources_are_ascii_only():
    """The safety sources (not only their comments) contain no non-ASCII character."""
    for rel in NODE_FILES + ["config/safety_supervisor.yaml",
                             "launch/safety_supervisor.launch.py"]:
        text = read(rel)
        assert all(ord(c) < 128 for c in text), rel


# Parameter table of the plan: name -> default
PLAN_DEFAULTS = {
    "enforcement": "filter", "robot_frame": "base_footprint", "scan_topic": "scan",
    "odom_topic": "bumperbot_controller/odom", "cmd_vel_in_topic": "safety/cmd_vel_in",
    "cmd_vel_out_topic": "bumperbot_controller/cmd_vel_unstamped", "lock_topic": "safety_stop",
    "status_topic": "safety/status", "estop_topic": "safety/estop",
    "dock_status_topic": "dock/status", "use_odom": True,
    "footprint": [0.10, 0.09, 0.10, -0.09, -0.10, -0.09, -0.10, 0.09],
    "lateral_margin": 0.03, "stop_margin": 0.05, "stop_release_hysteresis": 0.02,
    "slowdown_distance": 0.35, "contact_margin": 0.015, "max_linear_speed": 0.7,
    "max_angular_speed": 5.0, "response_time": 0.3, "max_decel": 0.5, "stopped_speed": 0.02,
    "brake_hold_time": 0.5, "scan_timeout": 0.6, "odom_timeout": 0.5, "cmd_timeout": 0.5,
    "recovery_time": 0.5, "max_invalid_beam_ratio": 0.5, "latch_faults": False,
    "latch_on_contact": True, "rate": 20.0, "status_rate": 5.0, "dock_approach_enabled": True,
    "dock_max_speed": 0.06, "dock_stop_margin": 0.01, "dock_approach_radius": 0.8,
    "dock_approach_max_time": 30.0, "dock_status_timeout": 2.0,
    "debug_inject_tick_exceptions": 0,
}


def yaml_params():
    with open(os.path.join(PKG, "config", "safety_supervisor.yaml")) as f:
        return yaml.safe_load(f)["safety_supervisor"]["ros__parameters"]


def python_defaults():
    from bumperbot_utils import safety_logic as sl
    cfg = sl.Config()
    return {name: getattr(cfg, name) for name in PLAN_DEFAULTS}


def cpp_defaults():
    """Parse the default member initializers of struct Config in safety_logic.hpp."""
    text = read("include/bumperbot_utils/safety_logic.hpp")
    start = text.index("{", text.index("struct Config")) + 1
    body = text[start:text.index("\n};", start)]
    out = {}
    for match in re.finditer(r"^\s*[\w:<>]+\s+(\w+)\s*(?:=|\{)\s*(.+?)\s*[;}]\s*(?://.*)?$",
                             body, re.MULTILINE):
        out[match.group(1)] = match.group(2)
    return out, body


def test_static_python_defaults_equal_plan_and_yaml():
    py = python_defaults()
    params = yaml_params()
    assert set(params) == set(PLAN_DEFAULTS), set(params) ^ set(PLAN_DEFAULTS)
    for name, expected in PLAN_DEFAULTS.items():
        assert py[name] == expected, "python default of {}: {!r}".format(name, py[name])
        assert params[name] == expected, "yaml value of {}: {!r}".format(name, params[name])
        assert type(params[name]) is type(expected), name


def test_static_cpp_defaults_equal_plan():
    parsed, body = cpp_defaults()
    missing = [n for n in PLAN_DEFAULTS if n not in parsed]
    assert not missing, "C++ Config has no default for {}".format(missing)
    for name, expected in PLAN_DEFAULTS.items():
        raw = parsed[name]
        if name == "footprint":
            numbers = [float(x) for x in re.findall(r"-?\d+\.\d+|-?\d+", raw)]
            assert numbers == expected, (name, raw)
        elif isinstance(expected, bool):
            assert raw == str(expected).lower(), (name, raw)
        elif isinstance(expected, str):
            raw = {"ENFORCEMENT_FILTER": '"filter"'}.get(raw, raw)
            assert raw.strip('"') == expected, (name, raw)
        else:
            assert float(raw) == expected, (name, raw)


def declared_names(rel, pattern):
    return set(re.findall(pattern, read(rel)))


def test_static_both_nodes_declare_the_same_parameters():
    cpp_text = read("src/safety_supervisor.cpp")
    py_text = read("bumperbot_utils/safety_supervisor.py")
    cpp = set(re.findall(r'"([a-z_]+)"', cpp_text)) & set(PLAN_DEFAULTS)
    py = set(re.findall(r'"([a-z_]+)"', py_text)) & set(PLAN_DEFAULTS)
    assert cpp == set(PLAN_DEFAULTS), set(PLAN_DEFAULTS) - cpp
    assert py == set(PLAN_DEFAULTS), set(PLAN_DEFAULTS) - py
    # use_sim_time is declared by the node base class in both languages
    for text in (cpp_text, py_text):
        assert "declare_parameter" in text


def test_static_safety_stop_legacy_unchanged_by_the_feature():
    res = subprocess.run(
        ["git", "diff", "--quiet", "HEAD", "--", "src/bumperbot_utils/src/safety_stop.cpp",
         "src/bumperbot_utils/bumperbot_utils/safety_stop.py"], cwd=ROOT)
    assert res.returncode == 0, "the legacy safety_stop node was modified"
