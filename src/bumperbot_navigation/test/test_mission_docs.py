"""AC14: documentation mentions the new package, launch file, interfaces and stations file."""
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
DOCS = ["README.md", "doc/10_navigation.md", "doc/03_ros2_nodes_topics_services.md",
        "doc/01_repository_structure.md", "doc/11_build_run_and_command_reference.md"]
REQUIRED = ["bumperbot_navigation", "mission_sim.launch.py", "/execute_mission",
            "/mission/status", "stations.yaml"]


def read(rel):
    with open(os.path.join(ROOT, rel), encoding="utf-8") as f:
        return f.read()


def test_required_strings_in_every_doc():
    missing = []
    for rel in DOCS:
        text = read(rel)
        for token in REQUIRED:
            if token not in text:
                missing.append(f"{rel}: {token}")
    assert not missing, missing


def test_doc10_no_longer_says_nav2_is_missing():
    text = read("doc/10_navigation.md")
    assert not re.search(r"no\s+Nav2\s+navigation\s+stack\s+is\s+configured", text, re.I)
    assert not re.search(r"Nav2 (is|are) not (yet )?configured", text, re.I)


def test_doc10_describes_review_fix_rules():
    text = read("doc/10_navigation.md")
    flat = re.sub(r"\s+", " ", text)
    for token in ("nav_server_timeout", "retry_delay", "duplicated", "frame_id", "global_frame",
                  "1e3", "quoted", "REJECTED", "wall (steady) time", "Terminal status precedence",
                  "DOCK_FAILED"):
        assert token in flat, token
    assert re.search(r"without counting a failed attempt", flat)
    assert re.search(r"refuses to start when `frame_id` differs from its `global_frame`", flat)
    assert re.search(r"not a non-null string", flat)
    assert re.search(r"has not started driving to another station", flat)


def test_nav2_params_controller_odom_topic():
    import yaml
    with open(os.path.join(HERE, "..", "config", "nav2_params.yaml")) as f:
        params = yaml.safe_load(f)
    assert params["controller_server"]["ros__parameters"]["odom_topic"] == \
        "/bumperbot_controller/odom"


def test_review_fixes_launch_cpp():
    """
    Review-fix launch tests (rejection, cancel races) for the C++ mission_manager.

    test_mission_review_fixes.launch.py is not registered with CMake on its own; the two
    implementations run from pytest wrappers (this one and test_stations_on_map.py) so that both
    stay in `colcon test` within the per-file timeout.
    """
    import subprocess
    env = dict(os.environ, REVIEW_IMPL="cpp")
    proc = subprocess.run(
        ["launch_test", os.path.join(HERE, "test_mission_review_fixes.launch.py")],
        capture_output=True, text=True, timeout=115, env=env)
    out = proc.stdout + proc.stderr
    assert proc.returncode == 0, out[-6000:]
    assert "Ran 6 tests" in out, "review-fix launch tests did not run\n" + out[-3000:]
