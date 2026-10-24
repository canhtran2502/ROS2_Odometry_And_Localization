"""AC23: the documentation describes the docking feature."""
import os
import re

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))

TOKENS = ["dock_manager", "/dock_robot", "DockRobot", "/dock/status", "/dock/detected_pose",
          "battery_monitor", "battery_sim", "/battery_state", "dock_sim.launch.py", "dock_vel"]
ALL_DOCS = ["README.md", "doc/10_navigation.md", "doc/03_ros2_nodes_topics_services.md",
            "doc/04_tf_and_coordinate_frames.md", "doc/05_robot_description_and_simulation.md",
            "doc/01_repository_structure.md", "doc/11_build_run_and_command_reference.md"]
# The feature documents describe everything; the topical documents at least their own part.
FULL_DOCS = ["README.md", "doc/10_navigation.md", "doc/03_ros2_nodes_topics_services.md",
             "doc/11_build_run_and_command_reference.md"]
TOPICAL = {
    "doc/01_repository_structure.md": [
        "dock_manager", "battery_monitor", "battery_sim", "dock_sim.launch.py"],
    "doc/04_tf_and_coordinate_frames.md": ["dock_manager", "/dock/detected_pose"],
    "doc/05_robot_description_and_simulation.md": [
        "battery_sim", "/dock/detected_pose", "/battery_state", "dock_sim.launch.py"],
}


def read(rel):
    with open(os.path.join(ROOT, rel), encoding="utf-8") as f:
        return f.read()


@pytest.mark.parametrize("rel", FULL_DOCS)
def test_feature_docs_mention_every_token(rel):
    text = read(rel)
    missing = [t for t in TOKENS if t not in text]
    assert not missing, "{}: {}".format(rel, missing)


@pytest.mark.parametrize("rel", sorted(TOPICAL))
def test_topical_docs_mention_their_tokens(rel):
    text = read(rel)
    missing = [t for t in TOPICAL[rel] if t not in text]
    assert not missing, "{}: {}".format(rel, missing)


def test_every_token_is_documented_somewhere():
    text = "\n".join(read(rel) for rel in ALL_DOCS)
    assert [t for t in TOKENS if t not in text] == []


@pytest.mark.parametrize("rel", ALL_DOCS)
def test_battery_sim_is_marked_simulation_only(rel):
    """A document that mentions battery_sim marks it as simulation only next to a mention."""
    lines = read(rel).splitlines()
    mentions = [n for n, line in enumerate(lines) if "battery_sim" in line]
    marked = [n for n in mentions
              if re.search(r"simulation[- ]only|sim only",
                           " ".join(lines[max(0, n - 2):n + 3]), re.I)]
    assert not mentions or marked, "{}: battery_sim is never marked as simulation only".format(rel)


def test_troubleshooting_mentions_safety_stop_lock_near_dock():
    text = read("doc/12_troubleshooting.md")
    assert "safety_stop" in text
    flat = re.sub(r"\s+", " ", text)
    assert re.search(r"safety_stop[^.]*dock|dock[^.]*safety_stop", flat, re.I)
    assert "danger_distance" in flat


def test_docs_name_the_run_commands():
    text = read("doc/10_navigation.md") + read("README.md")
    for command in ("ros2 launch bumperbot_bringup dock_sim.launch.py",
                    "ros2 action send_goal /dock_robot bumperbot_msgs/action/DockRobot",
                    "ros2 topic echo /dock/status",
                    "ros2 param set /battery_sim percentage_override"):
        assert command in text, command
