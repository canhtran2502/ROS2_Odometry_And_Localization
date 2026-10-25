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


# ---------------------------------------------------------------------------------------------
# Iteration 3 amendments (Opus review): documented wording
# ---------------------------------------------------------------------------------------------

def test_warning_zone_is_documented_by_distance_only():
    """The WARNING label depends on the distance only, not on the speed limit (review L1)."""
    for rel in ("README.md", "doc/07_lidar_and_sensor_processing.md",
                "doc/12_troubleshooting.md"):
        text = read(rel)
        assert "by distance only" in text, rel
        assert "stop_margin + slowdown_distance" in text, rel
        assert "WARNING is set when the speed limit is below" not in text, rel
        assert "WARNING means the speed limit is below" not in text, rel
    # the allowed speed can be below the maximum while the label still reads CLEAR
    assert "CLEAR" in read("doc/12_troubleshooting.md")
    assert "max_forward_speed" in read("doc/12_troubleshooting.md")


def test_crash_versus_clean_exit_lock_statement_m2():
    """M2: the lock stays true after a clean exit, after a crash the last value stays."""
    for rel in ("README.md", "doc/12_troubleshooting.md"):
        text = read(rel)
        assert re.search(r"clean exit", text, re.IGNORECASE), rel
        assert "SIGKILL" in text, rel
        assert re.search(r"crash", text), rel
        assert "may be false" in text or "normally `false`" in text, rel
        assert "lock_only" in text and "unprotected" in text, rel
        assert re.search(r"systemd|respawn", text), rel
    ts = read("doc/12_troubleshooting.md")
    assert "in `filter` mode the robot still stops" in ts
    # the shipped YAML says the same
    with open(os.path.join(ROOT, "src/bumperbot_utils/config/safety_supervisor.yaml"),
              encoding="utf-8") as f:
        yaml_text = f.read()
    assert "CLEAN exit" in yaml_text and "crash" in yaml_text
    assert "It stays true when the node exits" not in yaml_text


def test_restart_limitation_and_new_parameters_are_documented():
    """M5/M6: an e-stop does not survive a restart; the two new parameters are documented."""
    ts = read("doc/12_troubleshooting.md")
    assert "E-stop after a restart" in ts
    assert "in memory only" in ts and "require_reset_after_restart: true" in ts
    assert "transient_local" in ts and "ros2 topic pub --once" in ts
    readme = read("README.md")
    assert "An e-stop does not survive" in readme
    for rel in ("README.md", "doc/03_ros2_nodes_topics_services.md"):
        text = read(rel)
        assert "contact_debounce_scans" in text or "3 consecutive scans" in text, rel
    assert "require_reset_after_restart" in read("doc/03_ros2_nodes_topics_services.md")
    assert "contact_debounce_scans" in read("doc/07_lidar_and_sensor_processing.md")
    with open(os.path.join(ROOT, "src/bumperbot_utils/config/safety_supervisor.yaml"),
              encoding="utf-8") as f:
        yaml_text = f.read()
    assert "contact_debounce_scans: 3" in yaml_text
    assert "require_reset_after_restart: false" in yaml_text


def test_contact_reset_rule_and_range_min_semantics_are_documented():
    """H1/M3/M4/L4: reset rule, sub-range_min returns, dock reverse cap and the safety value."""
    ts = read("doc/12_troubleshooting.md")
    assert "swept path" in ts and "lock_only" in ts and "both sides" in ts
    assert "commanded toward" in ts
    assert "finite returns below `range_min`" in ts
    assert "Wall beside the robot" in ts and "never latches" in ts
    readme = read("README.md")
    assert "min(dock_max_speed, braking-distance limit for dock_stop_margin)" in readme
    assert "at most 0.06 m/s" in readme
    assert "swept path" in readme
    assert "any other value fails the launch" in read("doc/11_build_run_and_command_reference.md")
    # the stale statement is gone
    stale = "the reset is refused while `safety/estop` is true, a fault exists, or an obstacle"
    assert stale not in readme
