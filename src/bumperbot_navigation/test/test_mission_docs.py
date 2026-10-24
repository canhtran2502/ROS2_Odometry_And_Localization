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
