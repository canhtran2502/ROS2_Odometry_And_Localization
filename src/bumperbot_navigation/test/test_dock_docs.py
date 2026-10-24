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


# ---------------------------------------------------------------- iteration 2 (review fixes)
def flat(rel):
    """Document text without markdown code ticks and with collapsed whitespace."""
    return re.sub(r"\s+", " ", read(rel).replace("`", ""))


def test_doc10_has_the_mission_dock_interlock_section():
    """M2/M3/M4 are documented in doc/10: the section, the three parts and the exact outcome."""
    text = flat("doc/10_navigation.md")
    assert "Mission/dock interlock." in text
    section = text[text.index("Mission/dock interlock."):]
    section = section[:section.index("Cancel after the decision.")]
    for needle in (
            "A mission and a dock goal never drive together",
            "battery_monitor repeats the cancel-all",
            "dock_retry_delay",
            "CANCELING_MISSION, DOCKING and CHARGING",
            "dock_manager rejects DOCK while a mission is active",
            "inactive to active edge on /mission/status",
            "ABORTED with OUTCOME_ERROR",
            "a mission became active during the dock goal",
            "counts it as a failed attempt",
            "mission_manager is unchanged",
            "latched /mission/status is stale",
            "ignores it after 5 s with a warning",
            "dock_request_timeout watchdog",
            "a goal accepted late is canceled",
            "same for UNDOCK",
            "retry/ERROR logic",
            "unanswered when a dock cancel, a shutdown or a clock stall ends the goal is "
            "abandoned",
            "both languages"):
        assert needle in section, needle


def test_doc10_documents_the_cancel_after_the_decision():
    text = flat("doc/10_navigation.md")
    assert "Cancel after the decision." in text
    part = text[text.index("Cancel after the decision."):]
    for needle in ("DOCKED, UNDOCKED, ALREADY_DOCKED", "ends SUCCEEDED", "mission_manager"):
        assert needle in part, needle


def test_readme_documents_the_interlock():
    text = flat("README.md")
    assert "A mission and a dock goal never run together" in text
    assert "keeps canceling missions while docking or charging" in text
    assert "aborts its goal if a mission becomes active" in text


def test_doc03_battery_monitor_lists_the_cancelgoal_service_client():
    """battery_monitor has one DockRobot action client and a CancelGoal service client."""
    lines = [line for line in read("doc/03_ros2_nodes_topics_services.md").splitlines()
             if "/dock/status" in line and "/mission/status" in line and "Subscriptions" in line]
    assert len(lines) == 1, lines
    line = lines[0].replace("`", "")
    assert "Action client**: dock_robot" in line
    assert "Service client**: execute_mission/_action/cancel_goal" in line
    assert "action_msgs/srv/CancelGoal" in line
    assert "Action clients" not in line
    assert "execute_mission" not in line.split("Service client")[0], \
        "execute_mission is still listed as an action client"


def test_battery_monitor_really_has_only_that_client_surface():
    """The doc line matches the node: one ActionClient (DockRobot) and one CancelGoal client."""
    path = os.path.join(ROOT, "src", "bumperbot_navigation", "bumperbot_navigation",
                        "battery_monitor.py")
    with open(path) as f:
        code = f.read()
    assert len(re.findall(r"ActionClient\(", code)) == 1
    assert re.search(r"ActionClient\(\s*self,\s*DockRobot", code)
    assert len(re.findall(r"create_client\(", code)) == 1
    assert "CancelGoal" in code and "execute_mission/_action/cancel_goal" in code


@pytest.mark.parametrize("rel", ["README.md", "doc/10_navigation.md", "doc/12_troubleshooting.md"])
def test_docs_state_that_real_robot_launch_has_no_safety_stop(rel):
    text = flat(rel)
    assert re.search(r"real_robot\.launch\.py does not start safety_stop", text), rel
    assert re.search(r"(start|starts) safety_stop (yourself|separately)|"
                     r"user starts safety_stop", text), rel


def test_real_robot_launch_really_has_no_safety_stop():
    """The documented statement is true: no safety_stop node or include in real_robot.launch.py."""
    path = os.path.join(ROOT, "src", "bumperbot_bringup", "launch", "real_robot.launch.py")
    with open(path) as f:
        code = f.read()
    assert "safety_stop" not in code


def test_doc12_known_limitations_list():
    text = flat("doc/12_troubleshooting.md")
    assert "Known limitations (hardware)." in text
    part = text[text.index("Known limitations (hardware)."):]
    part = part[:part.index("safety_stop on the real robot.")]
    for marker, needles in (
            ("(a)", ["stamp 0 skips the age check", "yaw gate", "relative to the map prior"]),
            ("(b)", ["tf_stale_timeout (0.5 s)", "tf_timeout (1.0 s)", "timers firing",
                     "synchronize the clocks"]),
            ("(c)", ["dock_request_timeout (120 s)", "battery_monitor to ERROR", "~/reset"]),
            ("(d)", ["already active before dock_manager started", "publishes only on change",
                     "twist_mux still blocks", "STALL"])):
        assert marker in part, marker
        for needle in needles:
            assert needle in part, "{} {}".format(marker, needle)
    assert part.index("(a)") < part.index("(b)") < part.index("(c)") < part.index("(d)")
    assert "safety_stop on the real robot." in text
