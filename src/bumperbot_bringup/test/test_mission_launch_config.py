"""AC12: static and CLI checks of the mission launch files and configuration."""
import ast
import importlib.util
import os
import subprocess

import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.abspath(os.path.join(HERE, "..", ".."))
ROOT = os.path.abspath(os.path.join(SRC, ".."))
BRINGUP_LAUNCH = os.path.join(SRC, "bumperbot_bringup", "launch")
NAV_LAUNCH = os.path.join(SRC, "bumperbot_navigation", "launch")
MISSION_SIM = os.path.join(BRINGUP_LAUNCH, "mission_sim.launch.py")
NAVIGATION = os.path.join(NAV_LAUNCH, "navigation.launch.py")


def load_launch(path):
    spec = importlib.util.spec_from_file_location(
        "lf_" + os.path.basename(path).replace(".", "_"), path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def text(subs):
    if isinstance(subs, str):
        return subs
    if not isinstance(subs, (list, tuple)):
        subs = [subs]
    return "".join(getattr(s, "text", "") for s in subs)


def show_args(pkg, name):
    return subprocess.run(["ros2", "launch", pkg, name, "--show-args"], capture_output=True,
                          text=True, timeout=60)


def include_location(include):
    """Text of the included launch file path (the public .location property only gives a repr)."""
    src = include.launch_description_source
    return text(getattr(src, "_LaunchDescriptionSource__location"))


def declared_defaults(ld):
    return {e.name: text(e.default_value) for e in ld.entities
            if isinstance(e, DeclareLaunchArgument)}


# ------------------------------------------------------------- arguments
def test_mission_sim_show_args():
    res = show_args("bumperbot_bringup", "mission_sim.launch.py")
    assert res.returncode == 0, res.stderr
    for arg in ("simulator", "world_name", "use_rviz", "viewer", "use_python", "stations_file",
                "mission_config"):
        assert f"'{arg}'" in res.stdout, f"{arg} not listed:\n{res.stdout}"


def test_navigation_show_args():
    res = show_args("bumperbot_navigation", "navigation.launch.py")
    assert res.returncode == 0, res.stderr
    for arg in ("use_sim_time", "nav2_params"):
        assert f"'{arg}'" in res.stdout, f"{arg} not listed:\n{res.stdout}"


def test_mission_sim_argument_defaults():
    d = declared_defaults(load_launch(MISSION_SIM).generate_launch_description())
    assert d["simulator"] == "gazebo"
    assert d["world_name"] == "small_house"
    assert d["use_rviz"].lower() == "true"
    assert d["viewer"].lower() == "true"
    assert d["use_python"].lower() == "false"
    assert d["stations_file"].endswith(os.path.join("config", "stations.yaml"))
    assert d["mission_config"].endswith(os.path.join("config", "mission_manager.yaml"))


def test_navigation_argument_defaults():
    d = declared_defaults(load_launch(NAVIGATION).generate_launch_description())
    assert d["use_sim_time"].lower() == "false"
    assert d["nav2_params"].endswith(os.path.join("config", "nav2_params.yaml"))


# ------------------------------------------------------------- static launch content
def string_constants_outside_remappings(path):
    with open(path) as f:
        tree = ast.parse(f.read())
    in_remap = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.keyword) and node.arg == "remappings":
            for sub in ast.walk(node.value):
                in_remap.add(id(sub))
    return [n.value for n in ast.walk(tree)
            if isinstance(n, ast.Constant) and isinstance(n.value, str)
            and id(n) not in in_remap]


def test_no_absolute_paths_and_share_directory():
    for path in (MISSION_SIM, NAVIGATION):
        bad = [s for s in string_constants_outside_remappings(path) if s.startswith("/")]
        assert not bad, f"{path}: absolute path literals {bad}"
        with open(path) as f:
            assert "get_package_share_directory" in f.read(), path


def test_mission_sim_includes_simulated_robot_without_slam():
    ld = load_launch(MISSION_SIM).generate_launch_description()
    includes = [e for e in ld.entities if isinstance(e, IncludeLaunchDescription)]
    sim = [i for i in includes
           if "simulated_robot.launch.py" in include_location(i)]
    assert len(sim) == 1
    args = {k if isinstance(k, str) else text(k): text(v) for k, v in sim[0].launch_arguments}
    assert args.get("use_slam", "").lower() == "false", args
    nav = [i for i in includes
           if "navigation.launch.py" in include_location(i)]
    assert len(nav) == 1
    nav_args = {k if isinstance(k, str) else text(k): text(v) for k, v in nav[0].launch_arguments}
    assert nav_args.get("use_sim_time", "").lower() == "true"


def active_mission_managers(use_python):
    ld = load_launch(MISSION_SIM).generate_launch_description()
    ctx = LaunchContext()
    ctx.launch_configurations["use_python"] = use_python
    out = []
    for e in ld.entities:
        if isinstance(e, Node) and text(e.node_package) == "bumperbot_navigation":
            if e.condition is None or e.condition.evaluate(ctx):
                out.append(text(e.node_executable))
    return out


def test_exactly_one_mission_manager_cpp_default():
    assert active_mission_managers("false") == ["mission_manager"]


def test_exactly_one_mission_manager_python():
    assert active_mission_managers("true") == ["mission_manager.py"]


def test_navigation_remaps_cmd_vel_to_nav_vel():
    with open(NAVIGATION) as f:
        tree = ast.parse(f.read())
    seen = {}
    for call in ast.walk(tree):
        if not (isinstance(call, ast.Call) and getattr(call.func, "id", "") == "Node"):
            continue
        kw = {k.arg: k.value for k in call.keywords}
        exe = kw["executable"].value
        remaps = []
        if "remappings" in kw:
            remaps = [tuple(e.value for e in t.elts) for t in kw["remappings"].elts]
        seen[exe] = remaps
    for exe in ("controller_server", "behavior_server"):
        assert ("cmd_vel", "nav_vel") in seen[exe], (exe, seen)
    assert set(seen) >= {"controller_server", "planner_server", "behavior_server",
                         "bt_navigator", "lifecycle_manager"}


# ------------------------------------------------------------- configuration
def walk(d, key):
    if isinstance(d, dict):
        for k, v in d.items():
            if k == key:
                yield v
            yield from walk(v, key)
    elif isinstance(d, list):
        for v in d:
            yield from walk(v, key)


def test_nav2_params():
    with open(os.path.join(SRC, "bumperbot_navigation", "config", "nav2_params.yaml")) as f:
        cfg = yaml.safe_load(f)
    frames = list(walk(cfg, "robot_base_frame"))
    assert frames and all(v == "base_footprint" for v in frames), frames
    assert list(walk(cfg, "odom_topic")) == ["/bumperbot_controller/odom"] or all(
        v == "/bumperbot_controller/odom" for v in walk(cfg, "odom_topic"))
    assert list(walk(cfg, "odom_topic"))
    tol = list(walk(cfg, "xy_goal_tolerance"))
    assert tol and all(float(v) <= 0.15 for v in tol), tol
    assert "bt_navigator" in cfg and "plugin_lib_names" in cfg["bt_navigator"]["ros__parameters"]


def test_twist_mux_nav_vel_priority():
    path = os.path.join(SRC, "bumperbot_controller", "config", "twist_mux_topics.yaml")
    with open(path) as f:
        topics = yaml.safe_load(f)["twist_mux"]["ros__parameters"]["topics"]
    by_topic = {v["topic"]: v["priority"] for v in topics.values()}
    assert "nav_vel" in by_topic
    assert by_topic["explore_vel"] < by_topic["nav_vel"] < by_topic["key_vel"]
    assert by_topic["nav_vel"] < 255


# ------------------------------------------------------------- unchanged launch files
def feature_base_commit():
    """Parent of the first commit that added mission_sim.launch.py (the feature base)."""
    rel = "src/bumperbot_bringup/launch/mission_sim.launch.py"
    out = subprocess.run(["git", "log", "--diff-filter=A", "--format=%H", "--", rel], cwd=ROOT,
                         capture_output=True, text=True).stdout.split()
    if out:
        return out[-1] + "^"
    return "HEAD"  # not committed yet: compare with HEAD


def feature_end_commit():
    """Last commit of the waypoint-mission-manager feature (later features may change files)."""
    out = subprocess.run(
        ["git", "log", "-1", "--format=%H", "--", ".claude/features/waypoint-mission-manager"],
        cwd=ROOT, capture_output=True, text=True).stdout.split()
    assert out, "no commit of the waypoint-mission-manager feature was found"
    return out[0]


def test_real_and_simulated_robot_launch_unchanged_by_feature():
    base = feature_base_commit()
    end = feature_end_commit()
    res = subprocess.run(
        ["git", "diff", "--quiet", base, end, "--",
         "src/bumperbot_bringup/launch/real_robot.launch.py",
         "src/bumperbot_bringup/launch/simulated_robot.launch.py"], cwd=ROOT)
    assert res.returncode == 0, (
        f"real_robot/simulated_robot launch files changed by the feature ({base}..{end})")


def test_real_robot_launch_unchanged_vs_main():
    res = subprocess.run(
        ["git", "diff", "--quiet", "main", "--",
         "src/bumperbot_bringup/launch/real_robot.launch.py"], cwd=ROOT)
    assert res.returncode == 0


def test_existing_launch_files_still_show_args():
    for name in ("real_robot.launch.py", "simulated_robot.launch.py"):
        res = show_args("bumperbot_bringup", name)
        assert res.returncode == 0, f"{name}: {res.stderr}"
