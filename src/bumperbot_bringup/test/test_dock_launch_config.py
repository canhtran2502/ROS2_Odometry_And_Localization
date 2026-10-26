"""AC19: launch and configuration checks of the docking launch files."""
import ast
import importlib.util
import os
import subprocess

import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.abspath(os.path.join(HERE, "..", ".."))
ROOT = os.path.abspath(os.path.join(SRC, ".."))
BRINGUP_LAUNCH = os.path.join(SRC, "bumperbot_bringup", "launch")
NAV_PKG = os.path.join(SRC, "bumperbot_navigation")
DOCKING = os.path.join(NAV_PKG, "launch", "docking.launch.py")
DOCK_SIM = os.path.join(BRINGUP_LAUNCH, "dock_sim.launch.py")
UNCHANGED = ["src/bumperbot_bringup/launch/real_robot.launch.py",
             "src/bumperbot_bringup/launch/simulated_robot.launch.py",
             "src/bumperbot_bringup/launch/mission_sim.launch.py"]


def load_launch(path):
    spec = importlib.util.spec_from_file_location(
        "lf_" + os.path.basename(path).replace(".", "_"), path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def text(subs, context=None):
    if isinstance(subs, str):
        return subs
    if context is not None:
        return perform_substitutions(context, normalize_to_list_of_substitutions(subs))
    if not isinstance(subs, (list, tuple)):
        subs = [subs]
    return "".join(getattr(s, "text", "") for s in subs)


def show_args(pkg, name):
    return subprocess.run(["ros2", "launch", pkg, name, "--show-args"], capture_output=True,
                          text=True, timeout=60)


def include_location(include):
    src = include.launch_description_source
    return text(getattr(src, "_LaunchDescriptionSource__location"))


def declared_defaults(ld):
    return {e.name: text(e.default_value) for e in ld.entities
            if isinstance(e, DeclareLaunchArgument)}


def include_args(include, context):
    return {text(k, context): text(v, context) for k, v in include.launch_arguments}


# ------------------------------------------------------------------ arguments
def test_docking_show_args():
    res = show_args("bumperbot_navigation", "docking.launch.py")
    assert res.returncode == 0, res.stderr
    for arg in ("use_sim_time", "use_python", "use_battery_monitor", "stations_file",
                "dock_config", "battery_config"):
        assert "'{}'".format(arg) in res.stdout, "{} not listed:\n{}".format(arg, res.stdout)


def test_dock_sim_show_args():
    res = show_args("bumperbot_bringup", "dock_sim.launch.py")
    assert res.returncode == 0, res.stderr
    for arg in ("simulator", "world_name", "use_rviz", "viewer", "use_python", "battery_initial",
                "battery_sim_config"):
        assert "'{}'".format(arg) in res.stdout, "{} not listed:\n{}".format(arg, res.stdout)


def test_argument_defaults():
    d = declared_defaults(load_launch(DOCKING).generate_launch_description())
    assert d["use_sim_time"].lower() == "false"
    assert d["use_python"].lower() == "false"
    assert d["use_battery_monitor"].lower() == "true"
    assert d["stations_file"].endswith(os.path.join("config", "stations.yaml"))
    assert d["dock_config"].endswith(os.path.join("config", "dock_manager.yaml"))
    assert d["battery_config"].endswith(os.path.join("config", "battery_monitor.yaml"))
    s = declared_defaults(load_launch(DOCK_SIM).generate_launch_description())
    assert s["simulator"] == "mujoco"
    assert s["world_name"] == "small_house"
    assert s["use_rviz"].lower() == "true" and s["viewer"].lower() == "true"
    assert s["use_python"].lower() == "false"
    assert float(s["battery_initial"]) == 1.0
    assert s["battery_sim_config"].endswith(os.path.join("config", "battery_sim.yaml"))


# ------------------------------------------------------------------ static content
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
    for path in (DOCKING, DOCK_SIM):
        bad = [s for s in string_constants_outside_remappings(path) if s.startswith("/")]
        assert not bad, "{}: absolute path literals {}".format(path, bad)
        with open(path) as f:
            assert "get_package_share_directory" in f.read(), path


def nodes_of(ld, context):
    out = []
    for e in ld.entities:
        if isinstance(e, Node) and (e.condition is None or e.condition.evaluate(context)):
            out.append((text(e.node_package), text(e.node_executable)))
    return out


def context_with(ld=None, **configurations):
    """Return a launch context with the argument defaults of ld and the given overrides."""
    ctx = LaunchContext()
    if ld is not None:
        ctx.launch_configurations.update(declared_defaults(ld))
    ctx.launch_configurations.update(configurations)
    return ctx


def test_dock_sim_includes_mission_sim_and_docking_with_sim_time():
    ld = load_launch(DOCK_SIM).generate_launch_description()
    ctx = context_with(ld, use_python="false", simulator="mujoco")
    includes = [e for e in ld.entities if isinstance(e, IncludeLaunchDescription)]
    mission = [i for i in includes if "mission_sim.launch.py" in include_location(i)]
    docking = [i for i in includes if "docking.launch.py" in include_location(i)]
    assert len(mission) == 1 and len(docking) == 1
    assert include_args(docking[0], ctx).get("use_sim_time", "").lower() == "true"
    args = include_args(mission[0], ctx)
    assert args.get("simulator") == "mujoco"
    assert "viewer" in args and "use_rviz" in args and "world_name" in args


def dock_managers(use_python):
    """Return the dock_manager executables started by dock_sim.launch.py (via docking)."""
    sim = load_launch(DOCK_SIM).generate_launch_description()
    ctx = context_with(sim, use_python=use_python, simulator="mujoco")
    docking_include = [e for e in sim.entities if isinstance(e, IncludeLaunchDescription)
                       and "docking.launch.py" in include_location(e)][0]
    forwarded = include_args(docking_include, ctx)
    inner = load_launch(DOCKING).generate_launch_description()
    inner_ctx = context_with(inner, use_python=forwarded["use_python"],
                             use_sim_time=forwarded["use_sim_time"])
    return [exe for pkg, exe in nodes_of(inner, inner_ctx)
            if pkg == "bumperbot_navigation" and exe.startswith("dock_manager")]


def test_exactly_one_dock_manager_cpp_by_default():
    assert dock_managers("false") == ["dock_manager"]


def test_exactly_one_dock_manager_python_with_use_python():
    assert dock_managers("true") == ["dock_manager.py"]


def test_one_battery_sim_and_marker_only_for_mujoco():
    ld = load_launch(DOCK_SIM).generate_launch_description()
    for simulator, marker in (("mujoco", 1), ("gazebo", 0)):
        ctx = context_with(ld, use_python="false", simulator=simulator)
        nodes = nodes_of(ld, ctx)
        assert [n for n in nodes if n[1].startswith("battery_sim")] == [
            ("bumperbot_navigation", "battery_sim.py")], nodes
        markers = [n for n in nodes if n[1] == "dock_marker_sim"]
        assert len(markers) == marker, (simulator, nodes)
        assert all(n[0] == "bumperbot_mujoco" for n in markers)


def test_battery_sim_and_marker_use_sim_time():
    ld = load_launch(DOCK_SIM).generate_launch_description()
    with open(DOCK_SIM) as f:
        src = f.read()
    assert src.count('"use_sim_time": True') >= 2, "battery_sim and dock_marker_sim need sim time"
    assert any(isinstance(e, Node) for e in ld.entities)


def test_launch_does_not_start_nodes_on_the_real_robot_path():
    with open(DOCKING) as f:
        src = f.read()
    assert "battery_sim" not in src and "dock_marker_sim" not in src
    with open(os.path.join(BRINGUP_LAUNCH, "real_robot.launch.py")) as f:
        assert "dock" not in f.read().lower().replace("docker", "")


# ------------------------------------------------------------------ configuration
def walk(d, key):
    if isinstance(d, dict):
        for k, v in d.items():
            if k == key:
                yield v
            yield from walk(v, key)
    elif isinstance(d, list):
        for v in d:
            yield from walk(v, key)


def test_frames_are_consistent_between_dock_manager_mission_manager_and_nav2():
    with open(os.path.join(NAV_PKG, "config", "mission_manager.yaml")) as f:
        mission = yaml.safe_load(f)["mission_manager"]["ros__parameters"]
    with open(os.path.join(NAV_PKG, "config", "dock_manager.yaml")) as f:
        dock = yaml.safe_load(f)["dock_manager"]["ros__parameters"]
    assert mission["global_frame"] == dock["global_frame"] == "map"
    assert mission["robot_frame"] == dock["robot_frame"] == "base_footprint"
    with open(os.path.join(NAV_PKG, "config", "nav2_params.yaml")) as f:
        nav2 = yaml.safe_load(f)
    frames = list(walk(nav2, "robot_base_frame"))
    assert frames and all(v == "base_footprint" for v in frames), frames


# ------------------------------------------------------------------ unchanged launch files
def feature_base():
    """Parent of the commit that added the feature plan (the state before auto-docking)."""
    out = subprocess.run(
        ["git", "log", "--diff-filter=A", "--format=%H", "--",
         ".claude/features/auto-docking/plan.md"], cwd=ROOT, capture_output=True,
        text=True).stdout.split()
    assert out, "the auto-docking plan commit was not found"
    return out[-1] + "^"


def test_real_robot_launch_unchanged_vs_main():
    res = subprocess.run(["git", "diff", "--quiet", "main", "--", UNCHANGED[0]], cwd=ROOT)
    assert res.returncode == 0, "real_robot.launch.py differs from main"


def test_existing_launch_files_unchanged_by_the_feature():
    """Earlier features changed simulated_robot / mission_sim after main: compare to the base."""
    res = subprocess.run(["git", "diff", "--quiet", feature_base(), "--"] + UNCHANGED, cwd=ROOT)
    assert res.returncode == 0, "real_robot / simulated_robot / mission_sim changed by the feature"


def test_existing_launch_files_still_show_args():
    for pkg, name in (("bumperbot_bringup", "real_robot.launch.py"),
                      ("bumperbot_bringup", "simulated_robot.launch.py"),
                      ("bumperbot_bringup", "mission_sim.launch.py")):
        res = show_args(pkg, name)
        assert res.returncode == 0, "{}: {}".format(name, res.stderr)
