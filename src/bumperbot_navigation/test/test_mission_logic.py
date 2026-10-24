"""AC2 and AC4 (Python side): station loading, mission validation and MissionTracker cases."""
import glob
import math
import os

import pytest
import yaml

from bumperbot_navigation.mission_logic import (
    MissionTracker, TrackerParams, load_stations, planar_distance, validate_mission,
    yaw_to_quaternion, STEP_DWELL, STEP_FINISHED, STEP_NAVIGATE)

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "data")
CONFIG = os.path.join(HERE, "..", "config")


def test_stations_shipped_file_loads():
    st = load_stations(os.path.join(CONFIG, "stations.yaml"))
    assert st.frame_id == "map"
    assert st.dock_station == "dock"
    names = [s.name for s in st.stations]
    assert "dock" in names and len(names) >= 4


def test_stations_test_file_values():
    st = load_stations(os.path.join(DATA, "stations_test.yaml"))
    assert st.frame_id == "map" and st.dock_station == "dock"
    got = {s.name: (s.x, s.y, s.yaw, s.dwell) for s in st.stations}
    assert got == {
        "dock": (0.0, 0.0, 0.0, 0.0),
        "a": (1.0, 0.5, 0.5, 1.0),
        "b": (2.0, -1.0, 1.57, 0.2),
        "c": (-1.5, 2.0, 3.14, 0.2),
        "u1": (3.0, 3.0, 0.0, 0.2),
        "u2": (-3.0, -3.0, -1.0, 0.2),
    }


@pytest.mark.parametrize("path", sorted(glob.glob(os.path.join(DATA, "invalid_*.yaml"))))
def test_stations_invalid_fixture_rejected(path):
    with pytest.raises(ValueError):
        load_stations(path)


def test_stations_missing_file_rejected():
    with pytest.raises(ValueError):
        load_stations(os.path.join(DATA, "does_not_exist.yaml"))


def test_stations_invalid_fixtures_exist():
    assert len(glob.glob(os.path.join(DATA, "invalid_*.yaml"))) >= 6


def test_validate_mission():
    st = load_stations(os.path.join(DATA, "stations_test.yaml"))
    assert validate_mission(["a", "b"], st) == ""
    assert validate_mission([], st) != ""
    assert "nowhere" in validate_mission(["a", "nowhere"], st)


def test_stations_helpers():
    z, w = yaw_to_quaternion(1.0)
    assert z == pytest.approx(math.sin(0.5)) and w == pytest.approx(math.cos(0.5))
    assert planar_distance(0, 0, 3, 4) == pytest.approx(5.0)


# ---------------------------------------------------------------- AC4: shared cases
def load_cases():
    with open(os.path.join(DATA, "mission_cases.yaml")) as f:
        doc = yaml.safe_load(f)
    return doc["stations_file"], doc["cases"]


_FILE, _CASES = load_cases()


def fmt(step):
    if step.kind == STEP_FINISHED:
        return "FINISHED"
    if step.kind == STEP_DWELL:
        return "DWELL:" + step.station
    assert step.kind == STEP_NAVIGATE
    return "NAVIGATE:{}:{}".format(step.station, step.attempt) + (":dock" if step.docking else "")


def run_case(case):
    st = load_stations(os.path.join(DATA, _FILE))
    params = TrackerParams()
    for k, v in (case.get("params") or {}).items():
        setattr(params, k, v)
    tr = MissionTracker(st, case["stations"], params)
    events = list(case["events"])
    steps, progress = [], []
    for _ in range(200):
        step = tr.next_step()
        steps.append(fmt(step))
        progress.append(tr.progress)
        if step.kind == STEP_FINISHED:
            break
        assert events, "case '{}' ran out of events at step {}".format(case["name"], steps[-1])
        ev = events.pop(0)
        if "cancel" in ev:
            tr.request_cancel()
            # the cancel replaces the report: the current step is re-evaluated
            continue
        if "nav" in ev:
            assert step.kind == STEP_NAVIGATE, steps
            err = ev["nav"]["err"]
            tr.report_navigation(bool(ev["nav"]["ok"]), math.nan if err < 0 else float(err))
        elif "dwell_done" in ev:
            assert step.kind == STEP_DWELL, steps
            tr.report_dwell_done()
    else:
        pytest.fail("no FINISHED within 200 steps: {}".format(steps))
    return tr, steps, progress


@pytest.mark.parametrize("case", _CASES, ids=[c["name"] for c in _CASES])
def test_cases(case):
    tr, steps, progress = run_case(case)
    exp = case["expect"]
    assert steps == exp["steps"]
    assert tr.outcome == exp["outcome"]
    assert tr.docked == exp["docked"]
    assert list(tr.visited) == exp["visited"]
    assert list(tr.skipped) == exp["skipped"]
    if "state" in exp:
        assert tr.state == exp["state"]
    if "progress" in exp:
        assert tr.progress == pytest.approx(exp["progress"])
    assert all(b >= a - 1e-9 for a, b in zip(progress, progress[1:])), progress


def test_cases_cover_required_scenarios():
    names = {c["name"] for c in _CASES}
    for required in ("ordered_success_then_dock", "retry_then_success", "dock_unreachable",
                     "cancel_during_navigation_then_dock", "cancel_during_dwell_then_dock",
                     "distance_error_too_large_counts_as_failure",
                     "abort_after_consecutive_skips_then_dock",
                     "mission_ending_at_dock_sends_no_second_dock_goal"):
        assert required in names
