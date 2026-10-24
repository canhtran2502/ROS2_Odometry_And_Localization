"""AC2 (Python side): dock_logic geometry, controller, detection filter, timer and tracker."""
import math
import os

import pytest
import yaml

from bumperbot_navigation import dock_logic as dl

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "data")
TOL = 1e-9


def load_cases():
    with open(os.path.join(DATA, "dock_cases.yaml")) as f:
        return yaml.safe_load(f)


CASES = load_cases()


def pose(values):
    return dl.Pose2D(float(values[0]), float(values[1]), float(values[2]))


def assert_pose(actual, expected, abs_yaw=False):
    assert actual.x == pytest.approx(expected[0], abs=TOL)
    assert actual.y == pytest.approx(expected[1], abs=TOL)
    if abs_yaw:
        assert abs(actual.yaw) == pytest.approx(abs(expected[2]), abs=TOL)
    else:
        assert actual.yaw == pytest.approx(expected[2], abs=TOL)


# ------------------------------------------------------------------ geometry
@pytest.mark.parametrize("case", CASES["geometry"]["compute_dock_pose"])
def test_compute_dock_pose(case):
    got = dl.compute_dock_pose(pose(case["staging"]), case["distance"], case["backwards"])
    assert_pose(got, case["expect"])


def test_compute_dock_pose_shipped_config():
    got = dl.compute_dock_pose(dl.Pose2D(0.0, 0.0, 0.0), 0.5, True)
    assert_pose(got, [-0.5, 0.0, 0.0])


@pytest.mark.parametrize("case", CASES["geometry"]["normalize_angle"])
def test_normalize_angle(case):
    got = dl.normalize_angle(case["angle"])
    if "expect_abs" in case:
        assert abs(got) == pytest.approx(case["expect_abs"], abs=TOL)
    else:
        assert got == pytest.approx(case["expect"], abs=TOL)
    assert -math.pi - TOL <= got <= math.pi + TOL


@pytest.mark.parametrize("case", CASES["geometry"]["compose"])
def test_compose(case):
    assert_pose(dl.compose(pose(case["a"]), pose(case["b"])), case["expect"])


@pytest.mark.parametrize("case", CASES["geometry"]["inverse"])
def test_inverse(case):
    assert_pose(dl.inverse(pose(case["a"])), case["expect"])


@pytest.mark.parametrize("case", CASES["geometry"]["relative"])
def test_relative(case):
    got = dl.relative(pose(case["frame"]), pose(case["pose"]))
    assert_pose(got, case["expect"], abs_yaw=case.get("expect_abs_yaw", False))


@pytest.mark.parametrize("values", CASES["geometry"]["roundtrip"])
def test_compose_inverse_round_trip(values):
    a = pose(values)
    for got in (dl.compose(a, dl.inverse(a)), dl.compose(dl.inverse(a), a)):
        assert math.hypot(got.x, got.y) < TOL
        assert abs(dl.normalize_angle(got.yaw)) < TOL
    # relative(frame, frame) is the identity
    ident = dl.relative(a, a)
    assert math.hypot(ident.x, ident.y) < TOL and abs(ident.yaw) < TOL


@pytest.mark.parametrize("case", CASES["geometry"]["project_pose"])
def test_project_pose(case):
    assert_pose(dl.project_pose(case["frame"], case["pose"]), case["expect"])


# ------------------------------------------------------------------ controller
def control_params(overrides):
    return dl.ControlParams(**overrides)


@pytest.mark.parametrize("case", CASES["controller"], ids=lambda c: c["name"])
def test_controller_cases(case):
    par = control_params(case["params"])
    ctl = dl.DockController(par)
    fn = ctl.insert_command if case["op"] == "insert" else ctl.undock_command
    for p, exp in zip(case["poses"], case["expect"]):
        got = fn(pose(p))
        assert got.v == pytest.approx(exp["v"], abs=TOL), p
        assert got.w == pytest.approx(exp["w"], abs=TOL), p
        assert got.done == exp["done"], p
        assert got.failed == exp["failed"], p
        assert abs(got.v) <= par.max_linear_speed + 1e-12
        assert abs(got.w) <= par.max_angular_speed + 1e-12


@pytest.mark.parametrize("case", CASES["controller"], ids=lambda c: c["name"])
def test_controller_direction_properties(case):
    """Backwards insertion never drives forward, undock never drives backwards (and mirrored)."""
    par = control_params(case["params"])
    backwards = par.dock_backwards
    ctl = dl.DockController(par)
    for p in case["poses"]:
        if case["op"] == "insert":
            v = ctl.insert_command(pose(p)).v
            assert (v <= 0.0) if backwards else (v >= 0.0)
        else:
            v = ctl.undock_command(pose(p)).v
            assert (v >= 0.0) if backwards else (v <= 0.0)


@pytest.mark.parametrize("case", CASES["aligner"], ids=lambda c: c["name"])
def test_aligner_sequences(case):
    par = control_params(case["params"])
    ctl = dl.DockController(par)
    for p, exp in zip(case["poses"], case["expect"]):
        got = ctl.align_command(pose(p))
        assert got.v == pytest.approx(exp["v"], abs=TOL), p
        assert got.w == pytest.approx(exp["w"], abs=TOL), p
        assert got.done == exp["done"], p
        assert abs(got.v) <= par.max_linear_speed + 1e-12
        assert abs(got.w) <= par.max_angular_speed + 1e-12
        assert got.v >= 0.0, "the aligner never reverses"


def test_aligner_done_only_inside_tolerance():
    par = dl.ControlParams()
    ctl = dl.DockController(par)
    inside = ctl.align_command(dl.Pose2D(0.5 + 0.01, 0.01, 0.02))
    assert inside.done and inside.v == 0.0 and inside.w == 0.0
    ctl.reset()
    for p in (dl.Pose2D(0.5 + 0.05, 0.0, 0.0), dl.Pose2D(0.5, 0.0, 0.1),
              dl.Pose2D(0.5, 0.05, 0.0), dl.Pose2D(0.0, 0.0, 0.0)):
        assert not ctl.align_command(p).done
        ctl.reset()


def test_pose_error():
    pos, yaw = dl.DockController.pose_error(dl.Pose2D(0.03, 0.04, -0.2))
    assert pos == pytest.approx(0.05, abs=TOL)
    assert yaw == pytest.approx(0.2, abs=TOL)
    pos, yaw = dl.DockController.pose_error(dl.Pose2D(0.0, 0.0, 3.0 * math.pi))
    assert yaw == pytest.approx(math.pi, abs=TOL)


def simulate(par, start, goal_fn, steps=4000, dt=0.05):
    """Integrate a unicycle with the controller output; returns the final pose and done flag."""
    x, y, yaw = start
    cmd = None
    for _ in range(steps):
        cmd = goal_fn(dl.Pose2D(x, y, yaw))
        if cmd.done:
            return (x, y, yaw), True
        if cmd.failed:
            return (x, y, yaw), False
        x += cmd.v * math.cos(yaw) * dt
        y += cmd.v * math.sin(yaw) * dt
        yaw = dl.normalize_angle(yaw + cmd.w * dt)
    return (x, y, yaw), False


@pytest.mark.parametrize("start", [(0.3, 0.2, 0.5), (0.8, -0.3, -1.0), (0.1, 0.1, 2.5),
                                   (0.5, 0.05, 0.1)])
def test_closed_loop_align_then_insert_reaches_charger(start):
    par = dl.ControlParams()
    ctl = dl.DockController(par)
    pos, done = simulate(par, start, ctl.align_command)
    assert done, pos
    pos, done = simulate(par, pos, ctl.insert_command)
    assert done, pos
    err_pos, err_yaw = dl.DockController.pose_error(dl.Pose2D(*pos))
    assert err_pos < 0.02 and err_yaw < 0.05


def test_closed_loop_undock_reaches_staging():
    par = dl.ControlParams()
    ctl = dl.DockController(par)
    pos, done = simulate(par, (0.0, 0.01, 0.02), ctl.undock_command)
    assert done
    assert abs(pos[0] - 0.5) < 0.02 and abs(pos[1]) < 0.03


# ------------------------------------------------------------------ detection filter
@pytest.mark.parametrize("case", CASES["filter"], ids=lambda c: c["name"])
def test_detection_filter(case):
    flt = dl.DetectionFilter(case["max_jump"], case["alpha"])
    accepted, new = flt.update(pose(case["estimate"]), pose(case["detection"]))
    assert accepted == case["expect_accepted"]
    assert_pose(new, case["expect"])


# ------------------------------------------------------------------ phase timer
def run_timer_events(timer, events):
    for node, wall, expired, stalled in events:
        assert timer.expired(node, wall) == expired, (node, wall)
        assert timer.stalled() == stalled, (node, wall)


@pytest.mark.parametrize("case", CASES["timer"], ids=lambda c: c["name"])
def test_phase_timer(case):
    timer = dl.PhaseTimer(case["timeout"], case["clock_stall_timeout"])
    timer.start(*case["start"])
    run_timer_events(timer, case["events"])
    if "restart" in case:
        timer.start(*case["restart"])
        assert not timer.stalled()
        run_timer_events(timer, case["events_after"])


# ------------------------------------------------------------------ tracker
def parse_command(value):
    return {"DOCK": dl.COMMAND_DOCK, "UNDOCK": dl.COMMAND_UNDOCK}[value]


def apply_event(tracker, event):
    if isinstance(event, dict):
        pos, yaw = event["verify"]
        tracker.report_verify(float(pos), float(yaw))
    elif event == "cancel":
        tracker.request_cancel()
    elif event == "abort":
        tracker.abort("internal error")
    elif event.startswith("nav:"):
        tracker.report_nav(event == "nav:ok")
    elif event.startswith("det:"):
        tracker.report_detection(event == "det:ok")
    elif event.startswith("phase:"):
        tracker.report_phase(event.split(":", 1)[1])
    else:
        raise AssertionError("unknown event " + str(event))


def fmt(step):
    if step.kind == dl.STEP_FINISHED:
        return "FINISHED"
    return "{}:{}".format(step.kind, step.attempt)


def check_fields(tracker, expect):
    for key, value in expect.items():
        actual = getattr(tracker, key)
        assert actual == value, "{}: {!r} != {!r}".format(key, actual, value)


def run_goal(tracker, goal):
    tracker.start(parse_command(goal["command"]), goal["docked"], goal["near"])
    if "start_expect" in goal:
        check_fields(tracker, goal["start_expect"])
    events = list(goal["events"])
    steps = []
    for index in range(100):
        step = tracker.next_step()
        steps.append(fmt(step))
        if index == 0 and "first_step_expect" in goal:
            check_fields(tracker, goal["first_step_expect"])
        if step.kind == dl.STEP_FINISHED:
            break
        assert events, "ran out of events at {}".format(steps)
        apply_event(tracker, events.pop(0))
    else:
        pytest.fail("no FINISHED within 100 steps: {}".format(steps))
    assert not events, "unused events {}".format(events)
    return steps


@pytest.mark.parametrize("case", CASES["tracker"], ids=lambda c: c["name"])
def test_tracker_cases(case):
    params = dl.TrackerParams(**case["params"])
    tracker = dl.DockTracker(params)
    for goal in case["goals"]:
        steps = run_goal(tracker, goal)
        exp = dict(goal["expect"])
        assert steps == exp.pop("steps")
        assert tracker.outcome == exp.pop("outcome")
        check_fields(tracker, exp)
        for event in goal.get("after_finish", []):
            before = (tracker.outcome, tracker.docked, tracker.state, tracker.attempt)
            apply_event(tracker, event)
            assert before == (tracker.outcome, tracker.docked, tracker.state, tracker.attempt)
        if tracker.outcome in (dl.OUTCOME_CANCELED, dl.OUTCOME_ERROR):
            assert not tracker.docked
        assert tracker.finished


def test_tracker_next_step_is_idempotent():
    tracker = dl.DockTracker()
    tracker.start(dl.COMMAND_DOCK, False, True)
    first = tracker.next_step()
    again = tracker.next_step()
    assert (first.kind, first.attempt) == (again.kind, again.attempt)


def test_tracker_message_set_on_failure():
    tracker = dl.DockTracker(dl.TrackerParams(max_retries=0))
    tracker.start(dl.COMMAND_DOCK, False, True)
    tracker.next_step()
    tracker.report_detection(True)
    tracker.next_step()
    tracker.report_phase("TIMEOUT")
    assert tracker.outcome == dl.OUTCOME_DOCKING_FAILED
    assert tracker.message != ""


def test_tracker_constants_match_interface():
    from bumperbot_msgs.action import DockRobot
    from bumperbot_msgs.msg import DockStatus
    res = DockRobot.Result
    assert (dl.OUTCOME_DOCKED, dl.OUTCOME_UNDOCKED, dl.OUTCOME_ALREADY_DOCKED,
            dl.OUTCOME_CANCELED, dl.OUTCOME_STAGING_FAILED, dl.OUTCOME_DETECTION_FAILED,
            dl.OUTCOME_DOCKING_FAILED, dl.OUTCOME_UNDOCKING_FAILED, dl.OUTCOME_ERROR) == (
        res.OUTCOME_DOCKED, res.OUTCOME_UNDOCKED, res.OUTCOME_ALREADY_DOCKED,
        res.OUTCOME_CANCELED, res.OUTCOME_STAGING_FAILED, res.OUTCOME_DETECTION_FAILED,
        res.OUTCOME_DOCKING_FAILED, res.OUTCOME_UNDOCKING_FAILED, res.OUTCOME_ERROR)
    assert dl.STATE_NAVIGATING_TO_STAGING == DockStatus.STATE_NAVIGATING_TO_STAGING
    assert dl.STATE_WAITING_FOR_DETECTION == DockStatus.STATE_WAITING_FOR_DETECTION
    assert dl.STATE_FAILED == DockStatus.STATE_FAILED
    assert dl.COMMAND_DOCK == DockRobot.Goal.COMMAND_DOCK
    assert dl.COMMAND_UNDOCK == DockRobot.Goal.COMMAND_UNDOCK


# ------------------------------------------------------------------ table coverage
def test_table_covers_required_scenarios():
    names = {c["name"] for c in CASES["tracker"]}
    for required in (
            "success_first_attempt", "near_staging_skips_nav", "nav_fails_all_attempts",
            "required_without_detection", "optional_without_detection_continues",
            "verify_failure_retry_then_success", "docking_failed_timeouts",
            "docking_failed_mixed_reasons", "cancel_during_navigation",
            "cancel_during_wait_detection", "cancel_during_align", "cancel_during_insert",
            "cancel_during_verify", "abort_during_align", "undock_success",
            "undock_failure_timeout", "dock_while_docked_is_already_docked",
            "reuse_after_failure_starts_clean"):
        assert required in names, required
    reasons = " ".join(" ".join(str(e) for e in g["events"])
                       for c in CASES["tracker"] for g in c["goals"])
    for reason in ("TIMEOUT", "STALL", "SAFETY_TIMEOUT", "MISALIGNED"):
        assert reason in reasons
    assert len(CASES["controller"]) >= 5 and len(CASES["filter"]) >= 10
    assert len(CASES["timer"]) >= 4
