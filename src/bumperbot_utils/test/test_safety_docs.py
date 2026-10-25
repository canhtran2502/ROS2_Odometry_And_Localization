"""AC19: the documentation describes the safety supervisor."""
import os
import re

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))

TOKENS = ["safety_supervisor", "/safety/status", "SafetyStatus", "safety/cmd_vel_in",
          "/safety_supervisor/reset", "trigger_estop", "enforcement", "lock_only",
          "dock_max_speed", "safety:=legacy"]
ALL_DOCS = ["README.md", "doc/07_lidar_and_sensor_processing.md",
            "doc/03_ros2_nodes_topics_services.md", "doc/02_system_architecture.md",
            "doc/04_tf_and_coordinate_frames.md", "doc/10_navigation.md",
            "doc/11_build_run_and_command_reference.md", "doc/12_troubleshooting.md",
            "doc/01_repository_structure.md"]
# The feature documents describe everything; the topical documents at least their own part.
FULL_DOCS = ["README.md", "doc/07_lidar_and_sensor_processing.md",
             "doc/03_ros2_nodes_topics_services.md"]
TOPICAL = {
    "doc/01_repository_structure.md": ["safety_supervisor", "safety:=legacy"],
    "doc/02_system_architecture.md": ["safety_supervisor", "safety/cmd_vel_in", "SafetyStatus"],
    "doc/04_tf_and_coordinate_frames.md": ["safety_supervisor"],
    "doc/10_navigation.md": ["safety_supervisor", "dock_max_speed", "lock_only", "safety:=legacy"],
    "doc/11_build_run_and_command_reference.md": [
        "safety_supervisor", "/safety_supervisor/reset", "trigger_estop", "lock_only",
        "safety:=legacy", "/safety/status"],
    "doc/12_troubleshooting.md": ["safety_supervisor", "/safety_supervisor/reset", "lock_only",
                                  "safety:=legacy", "/safety/status"],
}
RECIPE = "ros2 launch bumperbot_utils safety_supervisor.launch.py enforcement:=lock_only"


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


def test_every_token_is_documented_somewhere_and_every_doc_exists():
    text = "\n".join(read(rel) for rel in ALL_DOCS)
    assert [t for t in TOKENS if t not in text] == []


@pytest.mark.parametrize("rel", ALL_DOCS)
def test_every_listed_doc_mentions_the_node(rel):
    assert "safety_supervisor" in read(rel), rel


@pytest.mark.parametrize("rel", ["README.md", "doc/10_navigation.md", "doc/12_troubleshooting.md"])
def test_real_robot_does_not_start_the_supervisor_and_recipe(rel):
    text = read(rel)
    assert re.search(r"real_robot\.launch\.py`? (is not modified and )?does not start "
                     r"`?safety_supervisor", text), rel
    assert RECIPE in text, "{}: hardware recipe missing".format(rel)


def test_troubleshooting_lock_kept_and_reset_refusals():
    text = read("doc/12_troubleshooting.md")
    assert re.search(r"[Ll]ock stays engaged after the supervisor exits", text)
    assert '/safety_supervisor/reset' in text
    for rule in ("estop input active", "fault:", "obstacle in contact zone"):
        assert rule in text, "reset refusal rule missing: " + rule
    assert "ros2 topic pub --once /safety_stop std_msgs/msg/Bool" in text


def test_commands_are_documented():
    text = read("README.md") + read("doc/11_build_run_and_command_reference.md")
    for command in ("ros2 topic echo /safety/status",
                    "ros2 service call /safety_supervisor/trigger_estop std_srvs/srv/Trigger",
                    "ros2 service call /safety_supervisor/reset std_srvs/srv/Trigger",
                    RECIPE):
        assert command in text, command


def test_existing_dock_doc_sentences_stay_intact():
    """The sentences that test_dock_docs.py relies on are still present."""
    readme = read("README.md")
    assert "real_robot.launch.py` does not start `safety_stop`" in readme
    assert "real_robot.launch.py` does not start `safety_stop`" in read("doc/10_navigation.md")
    assert "real_robot.launch.py` does not start `safety_stop`" in read(
        "doc/12_troubleshooting.md")
