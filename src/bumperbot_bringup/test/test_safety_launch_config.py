"""
AC14: launch and configuration checks of the safety supervisor integration.

The launch files are expanded for real (a LaunchService with the process start replaced by a
recorder), so conditions, include arguments and generated parameter files are evaluated by the
launch system itself. Expansion runs in a child process: python3 <this file> --expand ...
"""
import ast
import json
import os
import subprocess
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.abspath(os.path.join(HERE, "..", ".."))
ROOT = os.path.abspath(os.path.join(SRC, ".."))
LAUNCH_FILES = [("bumperbot_utils", "safety_supervisor.launch.py"),
                ("bumperbot_bringup", "simulated_robot.launch.py"),
                ("bumperbot_bringup", "real_robot.launch.py"),
                ("bumperbot_bringup", "mission_sim.launch.py"),
                ("bumperbot_bringup", "dock_sim.launch.py"),
                ("bumperbot_bringup", "auto_explore.launch.py"),
                ("bumperbot_controller", "joystick_teleop.launch.py")]
SIM_ARGS = ["simulator:=mujoco", "use_rviz:=false", "viewer:=false"]
UNCHANGED_VS_BASE = ["src/bumperbot_bringup/launch/real_robot.launch.py",
                     "src/bumperbot_bringup/launch/mission_sim.launch.py",
                     "src/bumperbot_bringup/launch/dock_sim.launch.py",
                     "src/bumperbot_bringup/launch/auto_explore.launch.py",
                     "src/bumperbot_firmware"]


# ------------------------------------------------------------------ launch expansion helper
def expand_main(pkg, name, argv):
    """Run inside the child process: expand a launch file and print the started processes."""
    import launch
    import launch.launch_description_sources as sources
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchService
    from launch.actions import ExecuteLocal

    records = []

    async def record_instead_of_start(self, context):
        args = self._ExecuteLocal__process_event_args
        cmd = [str(c) for c in args["cmd"]]
        params = []
        for i, part in enumerate(cmd):
            if part == "--params-file" and i + 1 < len(cmd) and os.path.exists(cmd[i + 1]):
                with open(cmd[i + 1]) as f:
                    params.append({"file": cmd[i + 1], "content": f.read()})
        records.append({"name": args["name"], "cmd": cmd, "params": params})
        future = self._ExecuteLocal__completed_future
        if future is not None and not future.done():
            future.set_result(None)

    ExecuteLocal._ExecuteLocal__execute_process = record_instead_of_start
    path = os.path.join(get_package_share_directory(pkg), "launch", name)
    arguments = dict(a.split(":=", 1) for a in argv)
    include = launch.actions.IncludeLaunchDescription(
        sources.PythonLaunchDescriptionSource(path), launch_arguments=arguments.items())
    service = LaunchService()
    service.include_launch_description(launch.LaunchDescription([include]))
    service.run()
    print("EXPANDED-JSON " + json.dumps(records))


def expand(pkg, name, argv=()):
    res = subprocess.run([sys.executable, os.path.abspath(__file__), "--expand", pkg, name,
                          *argv], capture_output=True, text=True, timeout=120)
    assert res.returncode == 0, res.stderr[-2000:]
    line = [ln for ln in res.stdout.splitlines() if ln.startswith("EXPANDED-JSON ")]
    assert line, res.stdout[-2000:]
    return json.loads(line[-1][len("EXPANDED-JSON "):])


def supervisors(records):
    return [r for r in records if r["name"].startswith("safety_supervisor")]


def legacy_nodes(records):
    return [r for r in records if r["name"].startswith("safety_stop")]


def twist_mux_output(records):
    mux = [r for r in records if r["name"].startswith("twist_mux")]
    assert len(mux) == 1, [r["name"] for r in mux]
    remaps = [c for c in mux[0]["cmd"] if c.startswith("/cmd_vel_out:=")]
    assert len(remaps) == 1, mux[0]["cmd"]
    return remaps[0].split(":=", 1)[1]


def parameters_of(record):
    """Merge the parameter files of a process: later files win (the node name is not checked)."""
    merged = {}
    for item in record["params"]:
        data = yaml.safe_load(item["content"])
        for node_params in data.values():
            merged.update(node_params["ros__parameters"])
    return merged


# ------------------------------------------------------------------ arguments
def show_args(pkg, name):
    return subprocess.run(["ros2", "launch", pkg, name, "--show-args"], capture_output=True,
                          text=True, timeout=60)


def test_show_args_succeeds_for_every_launch_file():
    for pkg, name in LAUNCH_FILES:
        res = show_args(pkg, name)
        assert res.returncode == 0, "{} {}: {}".format(pkg, name, res.stderr)


def argument_default(output, arg):
    lines = output.splitlines()
    for i, line in enumerate(lines):
        if line.strip() == "'{}':".format(arg):
            for follow in lines[i + 1:i + 4]:
                if "(default:" in follow:
                    return follow.split("(default:", 1)[1].strip().rstrip(")").strip().strip("'\"")
    return None


def test_new_arguments_and_defaults_are_listed():
    sim = show_args("bumperbot_bringup", "simulated_robot.launch.py").stdout
    assert argument_default(sim, "safety") == "supervisor", sim
    assert argument_default(sim, "safety_use_python") == "false", sim
    joy = show_args("bumperbot_controller", "joystick_teleop.launch.py").stdout
    assert argument_default(joy, "cmd_vel_out") == "bumperbot_controller/cmd_vel_unstamped", joy
    sup = show_args("bumperbot_utils", "safety_supervisor.launch.py").stdout
    assert argument_default(sup, "use_sim_time") == "false", sup
    assert argument_default(sup, "use_python") == "false", sup
    assert argument_default(sup, "enforcement") == "filter", sup
    assert "safety_supervisor.yaml" in (argument_default(sup, "config") or ""), sup


# ------------------------------------------------------------------ simulated_robot wiring
def test_default_simulated_robot_has_one_cpp_supervisor_and_no_safety_stop():
    records = expand("bumperbot_bringup", "simulated_robot.launch.py", SIM_ARGS)
    sup = supervisors(records)
    assert len(sup) == 1, [r["name"] for r in records]
    exe = sup[0]["cmd"][0]
    assert exe.endswith("/lib/bumperbot_utils/safety_supervisor"), exe
    assert legacy_nodes(records) == []
    params = parameters_of(sup[0])
    assert params["use_sim_time"] is True, params
    assert params["enforcement"] == "filter", params
    assert twist_mux_output(records) == "safety/cmd_vel_in"


def test_python_supervisor_with_safety_use_python():
    records = expand("bumperbot_bringup", "simulated_robot.launch.py",
                     SIM_ARGS + ["safety_use_python:=true"])
    sup = supervisors(records)
    assert len(sup) == 1
    assert sup[0]["cmd"][0].endswith("/lib/bumperbot_utils/safety_supervisor.py")
    assert legacy_nodes(records) == []


def test_legacy_simulated_robot_restores_the_old_node_and_wiring():
    records = expand("bumperbot_bringup", "simulated_robot.launch.py",
                     SIM_ARGS + ["safety:=legacy"])
    assert supervisors(records) == []
    legacy = legacy_nodes(records)
    assert len(legacy) == 1, [r["name"] for r in records]
    assert legacy[0]["cmd"][0].endswith("/lib/bumperbot_utils/safety_stop")
    assert twist_mux_output(records) == "bumperbot_controller/cmd_vel_unstamped"


def test_real_robot_starts_neither_supervisor_nor_safety_stop():
    records = expand("bumperbot_bringup", "real_robot.launch.py")
    assert supervisors(records) == [] and legacy_nodes(records) == []


def test_mission_dock_and_explore_launches_get_exactly_one_supervisor():
    for name in ("mission_sim.launch.py", "dock_sim.launch.py", "auto_explore.launch.py"):
        records = expand("bumperbot_bringup", name, SIM_ARGS)
        assert len(supervisors(records)) == 1, (name, [r["name"] for r in records])
        assert legacy_nodes(records) == [], name


def test_standalone_supervisor_launch_starts_exactly_one_node():
    records = expand("bumperbot_utils", "safety_supervisor.launch.py")
    assert len(records) == 1
    assert records[0]["cmd"][0].endswith("/lib/bumperbot_utils/safety_supervisor")
    records = expand("bumperbot_utils", "safety_supervisor.launch.py",
                     ["use_python:=true", "enforcement:=lock_only"])
    assert len(records) == 1
    assert records[0]["cmd"][0].endswith("/lib/bumperbot_utils/safety_supervisor.py")
    assert parameters_of(records[0])["enforcement"] == "lock_only"


# ------------------------------------------------------------------ unchanged files
def feature_base():
    out = subprocess.run(
        ["git", "log", "--diff-filter=A", "--format=%H", "--",
         ".claude/features/safety-supervisor/plan.md"], cwd=ROOT, capture_output=True,
        text=True).stdout.split()
    assert out, "the safety-supervisor plan commit was not found"
    return out[-1] + "^"


def test_launch_files_and_firmware_unchanged_vs_feature_base():
    res = subprocess.run(["git", "diff", "--quiet", feature_base(), "--"] + UNCHANGED_VS_BASE,
                         cwd=ROOT)
    assert res.returncode == 0, "a file that must stay unchanged differs from the feature base"
    untracked = subprocess.run(["git", "ls-files", "--others", "--exclude-standard",
                                "src/bumperbot_firmware"], cwd=ROOT, capture_output=True,
                               text=True).stdout.split()
    assert untracked == []


def test_real_robot_launch_unchanged_vs_main():
    res = subprocess.run(["git", "diff", "--quiet", "main", "--",
                          "src/bumperbot_bringup/launch/real_robot.launch.py"], cwd=ROOT)
    assert res.returncode == 0, "real_robot.launch.py differs from main"


# ------------------------------------------------------------------ static content
NEW_OR_CHANGED = [os.path.join(SRC, "bumperbot_utils", "launch", "safety_supervisor.launch.py"),
                  os.path.join(SRC, "bumperbot_bringup", "launch", "simulated_robot.launch.py"),
                  os.path.join(SRC, "bumperbot_controller", "launch",
                               "joystick_teleop.launch.py")]


def test_no_launch_literal_starts_with_an_absolute_system_path():
    for path in NEW_OR_CHANGED:
        with open(path) as f:
            tree = ast.parse(f.read())
        bad = [n.value for n in ast.walk(tree)
               if isinstance(n, ast.Constant) and isinstance(n.value, str)
               and (n.value.startswith("/home") or n.value.startswith("/opt"))]
        assert not bad, "{}: {}".format(path, bad)
        with open(path) as f:
            assert "/home/" not in f.read() and "/opt/" not in f.read(), path


def walk(d, key):
    if isinstance(d, dict):
        for k, v in d.items():
            if k == key:
                yield v
            yield from walk(v, key)
    elif isinstance(d, list):
        for v in d:
            yield from walk(v, key)


def test_robot_frame_is_base_footprint_for_supervisor_dock_manager_and_nav2():
    with open(os.path.join(SRC, "bumperbot_utils", "config", "safety_supervisor.yaml")) as f:
        supervisor = yaml.safe_load(f)["safety_supervisor"]["ros__parameters"]
    with open(os.path.join(SRC, "bumperbot_navigation", "config", "dock_manager.yaml")) as f:
        dock = yaml.safe_load(f)["dock_manager"]["ros__parameters"]
    with open(os.path.join(SRC, "bumperbot_navigation", "config", "nav2_params.yaml")) as f:
        nav2 = yaml.safe_load(f)
    frames = list(walk(nav2, "robot_base_frame"))
    assert supervisor["robot_frame"] == dock["robot_frame"] == "base_footprint"
    assert frames and all(v == "base_footprint" for v in frames), frames


def test_twist_mux_locks_unchanged_vs_main():
    res = subprocess.run(["git", "diff", "--quiet", "main", "--",
                          "src/bumperbot_controller/config/twist_mux_locks.yaml"], cwd=ROOT)
    assert res.returncode == 0


if __name__ == "__main__":
    if len(sys.argv) >= 4 and sys.argv[1] == "--expand":
        expand_main(sys.argv[2], sys.argv[3], sys.argv[4:])
