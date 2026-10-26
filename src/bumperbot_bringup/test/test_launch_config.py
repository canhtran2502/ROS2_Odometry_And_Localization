"""AC7: static and CLI checks of the launch files, twist_mux config, git diff and docs."""
import ast
import importlib.util
import os
import subprocess

import pytest
import yaml
from launch.actions import DeclareLaunchArgument, OpaqueFunction

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
SRC = os.path.join(ROOT, "src")
LAUNCH_DIR = os.path.join(SRC, "bumperbot_bringup", "launch")


def load_launch(name):
    path = os.path.join(LAUNCH_DIR, name)
    spec = importlib.util.spec_from_file_location("lf_" + name.replace(".", "_"), path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def show_args(name):
    return subprocess.run(["ros2", "launch", "bumperbot_bringup", name, "--show-args"],
                          capture_output=True, text=True, timeout=60)


def test_auto_explore_show_args():
    res = show_args("auto_explore.launch.py")
    assert res.returncode == 0, res.stderr
    for arg in ("simulator", "world_name", "use_rviz", "viewer", "map_save_path"):
        assert f"'{arg}'" in res.stdout, f"{arg} not listed:\n{res.stdout}"


def test_real_and_simulated_show_args():
    for name in ("real_robot.launch.py", "simulated_robot.launch.py"):
        res = show_args(name)
        assert res.returncode == 0, f"{name}: {res.stderr}"


def test_simulated_robot_new_args_and_defaults():
    res = show_args("simulated_robot.launch.py")
    assert "'use_rviz'" in res.stdout and "'viewer'" in res.stdout
    ld = load_launch("simulated_robot.launch.py").generate_launch_description()
    defaults = {}
    for e in ld.entities:
        if isinstance(e, DeclareLaunchArgument):
            defaults[e.name] = "".join(getattr(s, "text", "") for s in e.default_value)
    assert defaults["use_rviz"].lower() == "true"
    assert defaults["viewer"].lower() == "true"
    assert defaults["simulator"] == "gazebo"
    assert defaults["use_slam"].lower() == "false"


def test_auto_explore_description_has_explorer_node():
    mod = load_launch("auto_explore.launch.py")
    ld = mod.generate_launch_description()
    assert ld is not None
    opaque = [e for e in ld.entities if isinstance(e, OpaqueFunction)]
    assert opaque, "no OpaqueFunction that creates the explorer node"
    names = {e.name for e in ld.entities if isinstance(e, DeclareLaunchArgument)}
    assert {"simulator", "world_name", "use_rviz", "viewer", "map_save_path"} <= names
    # Resolve the OpaqueFunction with a real context and look at the produced node
    from launch import LaunchContext
    from launch.utilities import perform_substitutions
    ctx = LaunchContext()
    for e in ld.entities:
        if isinstance(e, DeclareLaunchArgument):
            ctx.launch_configurations[e.name] = perform_substitutions(ctx, e.default_value)
    produced = opaque[0].execute(ctx)
    from launch_ros.actions import Node
    nodes = [n for n in produced if isinstance(n, Node)]
    assert len(nodes) == 1
    assert nodes[0].node_executable == "frontier_explorer.py"
    assert nodes[0].node_package == "bumperbot_mapping"


def string_literals(path):
    with open(path, encoding="utf-8") as f:
        tree = ast.parse(f.read(), filename=path)
    for node in ast.walk(tree):
        if isinstance(node, ast.Constant) and isinstance(node.value, str):
            yield node.value, node.lineno


def test_no_absolute_paths_in_launch_files():
    bad = []
    for pkg in sorted(os.listdir(SRC)):
        ldir = os.path.join(SRC, pkg, "launch")
        if not os.path.isdir(ldir):
            continue
        for fn in os.listdir(ldir):
            if fn.endswith(".py"):
                for lit, line in string_literals(os.path.join(ldir, fn)):
                    if lit.startswith("/home") or lit.startswith("/opt"):
                        bad.append(f"{pkg}/launch/{fn}:{line}: {lit}")
    assert not bad, "absolute path literals: " + "; ".join(bad)


def load_yaml(rel):
    with open(os.path.join(SRC, rel), encoding="utf-8") as f:
        return yaml.safe_load(f)


def test_twist_mux_priorities():
    topics = load_yaml("bumperbot_controller/config/twist_mux_topics.yaml")["twist_mux"][
        "ros__parameters"]["topics"]
    assert topics["explore"]["topic"] == "explore_vel"
    explore = topics["explore"]["priority"]
    assert explore == 10
    assert explore < topics["keyboard"]["priority"] < topics["joystick"]["priority"]
    assert topics["keyboard"]["priority"] == 90 and topics["joystick"]["priority"] == 99
    assert topics["explore"]["timeout"] == 0.5
    locks = load_yaml("bumperbot_controller/config/twist_mux_locks.yaml")["twist_mux"][
        "ros__parameters"]["locks"]
    assert locks["safety_stop"]["priority"] == 255


def test_git_diff_main_untouched_paths():
    res = subprocess.run(
        ["git", "diff", "main", "--stat", "--",
         "src/bumperbot_bringup/launch/real_robot.launch.py", "src/bumperbot_firmware"],
        cwd=ROOT, capture_output=True, text=True)
    assert res.returncode == 0, res.stderr
    assert res.stdout.strip() == "", res.stdout


@pytest.mark.parametrize("doc", ["README.md", "doc/08_mapping_and_slam.md",
                                 "doc/03_ros2_nodes_topics_services.md",
                                 "doc/10_navigation.md"])
def test_docs_mention_feature(doc):
    with open(os.path.join(ROOT, doc), encoding="utf-8") as f:
        text = f.read()
    assert "auto_explore.launch.py" in text, f"{doc} lacks auto_explore.launch.py"
    assert "frontier_explorer" in text, f"{doc} lacks frontier_explorer"
