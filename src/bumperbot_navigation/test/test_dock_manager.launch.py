"""
AC5-AC13 (and the runtime part of AC20): dock_manager (C++ and Python) against MockDockWorld.

Every test method runs for both implementations (impl = cpp | py) with the same expectations. The
node under test is started by the test as a plain process (own process group), because several
scenarios need different parameters (detection_mode) or a node that starts before any TF exists.
Nav2, the simulator and twist_mux are replaced by MockDockWorld.

  ROS_DOMAIN_ID=90 ROS_LOCALHOST_ONLY=1 launch_test test/test_dock_manager.launch.py
"""
import math
import os
import shutil
import sys
import tempfile
import time
import unittest

os.environ.setdefault("ROS_DOMAIN_ID", "90")
os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")

import launch  # noqa: E402
import launch_testing  # noqa: E402
import launch_testing.actions  # noqa: E402
import launch_testing.markers  # noqa: E402
import pytest  # noqa: E402
from action_msgs.msg import GoalStatus  # noqa: E402
from bumperbot_msgs.action import DockRobot  # noqa: E402
from rcl_interfaces.srv import GetParameters, ListParameters  # noqa: E402
from sensor_msgs.msg import BatteryState  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from dock_harness import DockProcess, Rig, wait_until  # noqa: E402
from mock_dock_world import compose, inverse  # noqa: E402

SUCCEEDED = GoalStatus.STATUS_SUCCEEDED
CANCELED = GoalStatus.STATUS_CANCELED
ABORTED = GoalStatus.STATUS_ABORTED
R = DockRobot.Result

STAGING = (0.0, 0.0, 0.0)
CHARGER = (-0.5, 0.0, 0.0)
NEAR_START = (0.01, -0.01, 0.02)     # inside the align tolerances of the pre-dock pose
FAR_START = (1.5, 1.4, 0.3)          # about 2 m from the staging pose
POS_TOL, YAW_TOL = 0.03, 0.05
LOC_ERROR = (0.08, -0.05, 0.04)     # AC7 localization error map -> odom
EXIT_CODES = []                       # (impl, exit code, log tail) of every stopped node


@pytest.mark.launch_test
@launch_testing.parametrize("impl", ["cpp", "py"])
@launch_testing.markers.keep_alive
def generate_test_description(impl):
    return launch.LaunchDescription([launch_testing.actions.ReadyToTest()]), {}


def start_with_perceived(perceived):
    """Return the true start pose for which the node perceives ``perceived`` in map."""
    return compose(inverse(LOC_ERROR), perceived)


def moving(cmd):
    return cmd["v"] != 0.0 or cmd["w"] != 0.0


class DockTestCase(unittest.TestCase):
    rig = None
    node_key = None
    node_impl = None
    tmp = None

    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp(prefix="dock_test_")
        cls.rig = Rig()
        cls.node_key = None

    @classmethod
    def tearDownClass(cls):
        cls.stop_node()
        cls.rig.close()
        shutil.rmtree(cls.tmp, ignore_errors=True)

    # ------------------------------------------------------------------ node control
    @classmethod
    def stop_node(cls):
        if cls.rig.proc is not None:
            log = cls.rig.proc.log()
            code = cls.rig.stop_node()
            EXIT_CODES.append((cls.node_impl, code, log[-2000:]))
        cls.node_key = None

    @classmethod
    def use_node(cls, impl, params=None, fresh=False, tf_enabled=True):
        key = (impl, tuple(sorted((params or {}).items())))
        proc = cls.rig.proc
        if not fresh and proc is not None and proc.alive() and cls.node_key == key:
            return
        cls.stop_node()
        cls.rig.world.reset(tf_enabled=tf_enabled)
        cls.rig.start(impl, cls.tmp, params, tag="node")
        cls.node_key = key
        cls.node_impl = impl
        if not cls.rig.wait_ready(30.0):
            raise AssertionError("dock_manager did not start\n" + cls.rig.proc.log())

    @property
    def world(self):
        return self.rig.world

    def log(self):
        return self.rig.proc.log() if self.rig.proc is not None else ""

    def prepare(self, impl, params=None, fresh=False, **world_kwargs):
        """Make sure the node runs with the parameters, then reset the world and settle."""
        self.use_node(impl, params, fresh, world_kwargs.get("tf_enabled", True))
        w = self.world
        if self.nav_server_missing():
            w.start_nav_server()
        w.reset(**world_kwargs)
        w.set_mission("IDLE")
        if world_kwargs.get("tf_enabled", True):
            self.assertTrue(wait_until(
                lambda: (w.last_status() or {}).get("docked") is False, 5.0), "robot still docked")
        time.sleep(0.6)

    def nav_server_missing(self):
        return self.world.nav_server is None

    # ------------------------------------------------------------------ goal helpers
    def start_goal(self, command=DockRobot.Goal.COMMAND_DOCK):
        run = self.world.send_goal(command)
        self.assertTrue(self.world.wait_decision(run, 5.0), "no goal response\n" + self.log())
        return run

    def accepted_goal(self, command=DockRobot.Goal.COMMAND_DOCK):
        run = self.start_goal(command)
        self.assertTrue(run.accepted, "goal rejected\n" + self.log())
        return run

    def finish(self, run, timeout=90.0):
        self.assertTrue(self.world.wait_result(run, timeout), "no result\n" + self.log())
        time.sleep(0.4)     # the final zero commands and the idle status arrive after the result
        return run.result.status, run.result.result

    def dock(self, timeout=90.0):
        run = self.accepted_goal()
        status, result = self.finish(run, timeout)
        return run, status, result

    def wait_state(self, state, timeout=30.0, moving_cmd=False):
        w = self.world

        def reached():
            if w.state != state:
                return False
            if not moving_cmd:
                return True
            return any(c["state"] == state and moving(c) for c in w.commands_since(0.0))

        self.assertTrue(wait_until(reached, timeout, 0.01),
                        "state {} not reached (last {})\n{}".format(state, w.state, self.log()))

    def assert_no_motion(self, since=0.0):
        bad = [c for c in self.world.commands_since(since) if moving(c)]
        self.assertFalse(bad, "non-zero commands: {}".format(bad[:3]))

    def assert_last_command_zero(self):
        cmds = self.world.commands_since(0.0)
        self.assertTrue(cmds, "no dock_vel message at all")
        self.assertEqual((cmds[-1]["v"], cmds[-1]["w"]), (0.0, 0.0))

    def assert_docked_pose(self):
        pos, yaw = self.world.charger_error()
        self.assertLessEqual(pos, POS_TOL, "position error {:.4f} m".format(pos))
        self.assertLessEqual(yaw, YAW_TOL, "yaw error {:.4f} rad".format(yaw))

    def assert_idle_state(self, state="UNDOCKED"):
        self.assertTrue(wait_until(lambda: self.world.state == state, 3.0),
                        "state {} (expected {})".format(self.world.state, state))

    def assert_fresh_goal_possible(self):
        """After an error / cancel path a new DOCK goal is accepted and starts at attempt 1."""
        w = self.world
        time.sleep(0.3)
        if self.nav_server_missing():
            w.start_nav_server()
        w.odom_tf_enabled = True
        w.stall = False
        w.safety_lock = False
        w.nav_script = {}
        w.teleport(NEAR_START)
        time.sleep(0.6)
        run = self.start_goal()
        self.assertTrue(run.accepted, "a new goal was rejected after the previous one ended")
        self.assertTrue(wait_until(lambda: len(run.feedback) > 0, 5.0), "no first feedback")
        self.assertEqual(run.feedback[0][1].attempt, 1)
        w.cancel(run)
        status, _ = self.finish(run, 15.0)
        self.assertEqual(status, CANCELED)

    def commands_between(self, t0, t1):
        return [c for c in self.world.commands_since(t0) if c["t"] <= t1]

    def assert_stop_after(self, t_event, limit, what):
        """Expect a zero command within ``limit`` s after t_event and no motion afterwards."""
        cmds = self.world.commands_since(t_event)
        zero = next((c for c in cmds if not moving(c)), None)
        self.assertIsNotNone(zero, what + ": no zero command after the event")
        delay = zero["t"] - t_event
        self.assertLessEqual(delay, limit, "{}: first zero command after {:.3f} s".format(
            what, delay))
        later = [c for c in cmds if c["t"] > zero["t"] and moving(c)]
        self.assertFalse(later, what + ": non-zero commands after the stop")


# ====================================================================== AC5, AC6
class TestDockSuccess(DockTestCase):

    def test_dock_success(self, impl):
        """AC5: Nav2 to staging (+0.08, -0.06, +0.2 rad), align, back in, verify."""
        self.prepare(impl, pose=FAR_START, staging_error=(0.08, -0.06, 0.2))
        run, status, result = self.dock()
        w = self.world
        self.assertEqual(status, SUCCEEDED, result.message)
        self.assertEqual(result.outcome, R.OUTCOME_DOCKED)
        self.assertTrue(result.docked)
        self.assertEqual(result.attempts, 1)
        self.assertEqual(len(w.nav_goals), 1, "exactly one Nav2 goal expected")
        goal = w.nav_goals[0]
        self.assertEqual(goal["frame_id"], "map")
        self.assertAlmostEqual(goal["x"], STAGING[0], delta=1e-6)
        self.assertAlmostEqual(goal["y"], STAGING[1], delta=1e-6)
        self.assertAlmostEqual(math.remainder(goal["yaw"] - STAGING[2], 2 * math.pi), 0.0,
                               delta=1e-6)
        self.assert_docked_pose()
        states = w.states()
        order = [states.index(s) for s in (
            "NAVIGATING_TO_STAGING", "ALIGNING", "DOCKING", "DOCKED")]
        self.assertEqual(order, sorted(order), states)
        self.assertEqual(states.count("DOCKED"), 1, states)
        cmds = w.commands_since(0.0)
        self.assertTrue(cmds)
        for c in cmds:
            self.assertLessEqual(abs(c["v"]), 0.15 + 1e-9)
            self.assertLessEqual(abs(c["w"]), 0.6 + 1e-9)
            if c["state"] == "DOCKING":
                self.assertLessEqual(c["v"], 0.0, "forward command while DOCKING")
        self.assert_last_command_zero()

    def test_skip_staging(self, impl):
        """AC6: a robot within staging_tolerance gets no Nav2 goal and docks."""
        self.prepare(impl, pose=(0.1, 0.1, 0.1))
        run, status, result = self.dock()
        self.assertEqual(status, SUCCEEDED, result.message)
        self.assertEqual(result.outcome, R.OUTCOME_DOCKED)
        self.assertEqual(len(self.world.nav_goals), 0, "Nav2 was used although near staging")
        self.assertNotIn("NAVIGATING_TO_STAGING", self.world.states())
        self.assert_docked_pose()
        self.assert_last_command_zero()


# ====================================================================== AC7
class TestDetectionRefinement(DockTestCase):

    def test_detection_refinement(self, impl):
        """AC7a and AC7c: detections remove the localization bias, bad messages are ignored."""
        for inject in (False, True):
            with self.subTest(inject=inject):
                self.prepare(impl, localization_error=LOC_ERROR,
                             pose=start_with_perceived(NEAR_START))
                self.world.inject_active = inject
                run = self.accepted_goal()
                status, result = self.finish(run)
                self.assertEqual(status, SUCCEEDED, result.message)
                self.assertEqual(result.outcome, R.OUTCOME_DOCKED)
                self.assert_docked_pose()
                last = self.world.last_status()
                used = any(s["detection_used"] for s in self.world.status_log)
                self.assertTrue(used or last["detection_used"], "detection_used never set")
                self.assertTrue(any(f.detection_used for _, f in run.feedback),
                                "detection_used never set in the feedback")
                self.assertTrue(self.rig.proc.alive(), "the node died\n" + self.log())
                self.assertNotIn("Traceback", self.log())

    def test_map_only_keeps_the_bias(self, impl):
        """AC7b: with detection_mode none the true error stays at the localization bias."""
        self.prepare(impl, params={"detection_mode": "none"}, localization_error=LOC_ERROR,
                     pose=start_with_perceived(NEAR_START))
        run, status, result = self.dock()
        self.assertEqual(status, SUCCEEDED, result.message)
        self.assertEqual(result.outcome, R.OUTCOME_DOCKED)
        pos, _ = self.world.charger_error()
        self.assertGreaterEqual(
            pos, 0.06, "map-only docking is unexpectedly precise: {:.3f}".format(pos))
        self.assertNotIn("WAITING_FOR_DETECTION", self.world.states())
        self.assertFalse(any(s["detection_used"] for s in self.world.status_log))

    def test_detection_required_missing(self, impl):
        """AC7d: required without detections ends DETECTION_FAILED, never in DOCKING."""
        self.prepare(impl, params={"detection_mode": "required"}, detections=False,
                     pose=NEAR_START)
        run = self.accepted_goal()
        status, result = self.finish(run, 30.0)
        elapsed = run.t_result - run.t_decision
        self.assertEqual(status, ABORTED)
        self.assertEqual(result.outcome, R.OUTCOME_DETECTION_FAILED)
        self.assertFalse(result.docked)
        self.assertLessEqual(elapsed, 2.0 + 2.0, "took {:.1f} s".format(elapsed))
        states = self.world.states()
        self.assertNotIn("DOCKING", states)
        self.assertNotIn("ALIGNING", states)
        self.assertIn("WAITING_FOR_DETECTION", states)
        self.assert_no_motion()
        self.assert_last_command_zero()


# ====================================================================== AC8
class TestRetriesAndFailures(DockTestCase):

    def test_retry_on_slip(self, impl):
        """AC8a: an unobserved 0.06 m slip in the first insertion causes a second attempt."""
        self.prepare(impl, pose=NEAR_START, slip_first_insert=True)
        run, status, result = self.dock(120.0)
        w = self.world
        self.assertTrue(w.slip_applied, "the mock slip never happened")
        self.assertEqual(status, SUCCEEDED, result.message)
        self.assertEqual(result.outcome, R.OUTCOME_DOCKED)
        self.assertEqual(result.attempts, 2)
        states = w.states()
        self.assertGreaterEqual(states.count("ALIGNING"), 2, states)
        self.assertGreaterEqual(states.count("DOCKING"), 2, states)
        self.assert_docked_pose()

    def test_stall_fails_safe(self, impl):
        """AC8b: commands are ignored, max_retries + 1 attempts, then DOCKING_FAILED."""
        self.prepare(impl, pose=(0.1, 0.0, 0.0), stall=True)
        run = self.accepted_goal()
        status, result = self.finish(run, 60.0)
        elapsed = run.t_result - run.t_decision
        self.assertEqual(status, ABORTED)
        self.assertEqual(result.outcome, R.OUTCOME_DOCKING_FAILED)
        self.assertFalse(result.docked)
        self.assertEqual(result.attempts, 3)
        self.assertLessEqual(elapsed, 3 * (2.0 + 2.0), "took {:.1f} s".format(elapsed))
        self.assert_idle_state("FAILED")
        self.assert_last_command_zero()
        attempts = {f.attempt for _, f in run.feedback}
        self.assertEqual(attempts, {1, 2, 3})

    def test_staging_failed(self, impl):
        """AC8c: Nav2 always aborts: nav_max_retries + 1 goals, then STAGING_FAILED."""
        self.prepare(impl, pose=FAR_START, nav_script={"always_abort": True, "duration": 0.2})
        run = self.accepted_goal()
        status, result = self.finish(run, 60.0)
        self.assertEqual(status, ABORTED)
        self.assertEqual(result.outcome, R.OUTCOME_STAGING_FAILED)
        self.assertFalse(result.docked)
        self.assertEqual(len(self.world.nav_goals), 3)
        self.assert_no_motion()
        self.assert_idle_state("FAILED")


# ====================================================================== AC9
class TestSafetyLock(DockTestCase):

    def test_safety_lock(self, impl):
        """AC9a: a 2 s lock in the middle of DOCKING pauses the robot, docking completes."""
        self.prepare(impl, pose=NEAR_START)
        w = self.world
        run = self.accepted_goal()
        self.wait_state("DOCKING", 40.0, moving_cmd=True)
        time.sleep(0.5)
        t_lock = time.monotonic()
        w.safety_lock = True
        time.sleep(2.0)
        t_unlock = time.monotonic()
        w.safety_lock = False
        status, result = self.finish(run)
        self.assertEqual(status, SUCCEEDED, result.message)
        self.assertEqual(result.outcome, R.OUTCOME_DOCKED)
        locked = self.commands_between(t_lock + 0.15, t_unlock)
        self.assertTrue(locked, "no command while locked")
        self.assertFalse([c for c in locked if moving(c)], "motion command while locked")
        self.assert_docked_pose()
        # the node never publishes on safety_stop
        publishers = w.publishers_of("/safety_stop")
        self.assertIn("mock_dock_world", publishers)
        self.assertNotIn("dock_manager", publishers)
        self.assertIn("dock_manager", w.publishers_of("/dock_vel"))

    def test_safety_lock_timeout(self, impl):
        """AC9b: a lock held longer than safety_stop_timeout fails every attempt."""
        self.prepare(impl, pose=NEAR_START)
        w = self.world
        run = self.accepted_goal()
        self.wait_state("DOCKING", 40.0, moving_cmd=True)
        t_lock = time.monotonic()
        w.safety_lock = True
        status, result = self.finish(run, 90.0)
        w.safety_lock = False
        self.assertEqual(status, ABORTED)
        self.assertEqual(result.outcome, R.OUTCOME_DOCKING_FAILED)
        self.assertFalse(result.docked)
        self.assertEqual(result.attempts, 3)
        self.assertFalse([c for c in self.commands_between(t_lock + 0.15, time.monotonic())
                          if moving(c)], "motion while locked")
        self.assert_last_command_zero()
        self.assert_idle_state("FAILED")


# ====================================================================== AC10
class TestCancelAndErrors(DockTestCase):

    def test_cancel_paths(self, impl):
        """AC10a-d: cancel in NAVIGATING, ALIGNING, DOCKING and right after acceptance."""
        # (a) NAVIGATING_TO_STAGING
        self.prepare(impl, pose=FAR_START, nav_script={"duration": 30.0})
        w = self.world
        run = self.accepted_goal()
        self.wait_state("NAVIGATING_TO_STAGING", 10.0)
        self.assertTrue(wait_until(lambda: len(w.nav_goals) == 1, 5.0))
        w.cancel(run)
        status, result = self.finish(run, 15.0)
        self.assertEqual(status, CANCELED)
        self.assertEqual(result.outcome, R.OUTCOME_CANCELED)
        self.assertFalse(result.docked)
        self.assertTrue(wait_until(lambda: len(w.nav_cancels) >= 1, 3.0),
                        "the Nav2 goal was not canceled")
        self.assert_no_motion()
        self.assert_fresh_goal_possible()

        # (b) ALIGNING
        self.prepare(impl, pose=(0.2, 0.1, 0.3))
        run = self.accepted_goal()
        self.wait_state("ALIGNING", 20.0, moving_cmd=True)
        t_cancel = time.monotonic()
        w.cancel(run)
        status, result = self.finish(run, 15.0)
        self.assertEqual(status, CANCELED)
        self.assertEqual(result.outcome, R.OUTCOME_CANCELED)
        self.assertFalse(result.docked)
        self.assert_stop_after(t_cancel, 0.2, "cancel in ALIGNING")
        self.assert_idle_state("UNDOCKED")
        self.assertFalse(w.last_status()["docked"])
        self.assert_fresh_goal_possible()

        # (c) DOCKING
        self.prepare(impl, pose=NEAR_START)
        run = self.accepted_goal()
        self.wait_state("DOCKING", 30.0, moving_cmd=True)
        t_cancel = time.monotonic()
        w.cancel(run)
        status, result = self.finish(run, 15.0)
        self.assertEqual(status, CANCELED)
        self.assertEqual(result.outcome, R.OUTCOME_CANCELED)
        self.assertFalse(result.docked)
        self.assert_stop_after(t_cancel, 0.2, "cancel in DOCKING")
        self.assert_idle_state("UNDOCKED")
        self.assertFalse(w.last_status()["docked"])
        self.assert_fresh_goal_possible()

        # (d) race: cancel within 50 ms of the acceptance (with and without Nav2 first)
        for pose, script in ((FAR_START, {"duration": 5.0}), (NEAR_START, {})):
            self.prepare(impl, pose=pose, nav_script=script)
            run = self.start_goal()
            self.assertTrue(run.accepted)
            w.cancel(run)
            self.assertLessEqual(run.t_cancel - run.t_decision, 0.05, "test timing broke")
            status, result = self.finish(run, 10.0)
            self.assertEqual(status, CANCELED)
            self.assertLessEqual(run.t_result - run.t_cancel, 1.0)
            self.assertEqual(result.outcome, R.OUTCOME_CANCELED)
            self.assert_no_motion()
            self.assert_fresh_goal_possible()

    def test_error_paths(self, impl):
        """AC10e and AC10f: lost odom TF and a vanished Nav2 server end safely."""
        # (e) odom -> base_footprint stops during DOCKING
        self.prepare(impl, pose=NEAR_START)
        w = self.world
        run = self.accepted_goal()
        self.wait_state("DOCKING", 30.0, moving_cmd=True)
        t_stop = time.monotonic()
        w.odom_tf_enabled = False
        status, result = self.finish(run, 15.0)
        self.assertEqual(status, ABORTED)
        self.assertEqual(result.outcome, R.OUTCOME_ERROR)
        self.assertFalse(result.docked)
        self.assertLessEqual(run.t_result - t_stop, 1.0 + 1.0, "ERROR came too late")
        self.assert_stop_after(t_stop, 0.5 + 0.2, "odom TF lost")
        self.assert_idle_state("FAILED")
        self.assert_last_command_zero()
        self.assert_fresh_goal_possible()

        # (f) the Nav2 server disappears while a goal is active
        self.prepare(impl, pose=FAR_START, nav_script={"duration": 30.0})
        run = self.accepted_goal()
        self.wait_state("NAVIGATING_TO_STAGING", 10.0)
        self.assertTrue(wait_until(lambda: len(w.nav_goals) == 1, 5.0))
        time.sleep(0.5)
        t_gone = time.monotonic()
        w.stop_nav_server()
        status, result = self.finish(run, 30.0)
        self.assertEqual(status, ABORTED)
        self.assertIn(result.outcome, (R.OUTCOME_STAGING_FAILED, R.OUTCOME_ERROR))
        self.assertLessEqual(run.t_result - t_gone, 3.0 + 2.0)
        self.assert_no_motion()
        if w.commands_since(0.0):
            self.assert_last_command_zero()
        self.assert_fresh_goal_possible()


# ====================================================================== AC11
class TestDockedFlag(DockTestCase):

    def test_docked_flag_reset(self, impl):
        """AC11: ALREADY_DOCKED, leaving the dock clears the flag, CHARGING follows the battery."""
        self.prepare(impl, pose=NEAR_START)
        w = self.world
        run, status, result = self.dock()
        self.assertEqual(result.outcome, R.OUTCOME_DOCKED, result.message)
        self.assertTrue(w.last_status()["docked"])

        # DOCK while docked: no motion
        t0 = time.monotonic()
        run2, status, result = self.dock(20.0)
        self.assertEqual(status, SUCCEEDED)
        self.assertEqual(result.outcome, R.OUTCOME_ALREADY_DOCKED)
        self.assertTrue(result.docked)
        self.assert_no_motion(t0)

        # battery: CHARGING while docked, FULL back to DOCKED
        w.set_battery(0.5, BatteryState.POWER_SUPPLY_STATUS_CHARGING)
        self.assertTrue(wait_until(
            lambda: (w.last_status() or {}).get("state") == "CHARGING"
            and w.last_status()["charging"], 4.0), "no CHARGING state: {}".format(w.last_status()))
        w.set_battery(1.0, BatteryState.POWER_SUPPLY_STATUS_FULL)
        self.assertTrue(wait_until(
            lambda: (w.last_status() or {}).get("state") == "DOCKED"
            and not w.last_status()["charging"], 4.0),
            "no DOCKED state: {}".format(w.last_status()))
        self.assertTrue(w.last_status()["docked"])

        # the robot is moved 0.15 m away: UNDOCKED within 1 s
        x, y, yaw = w.truth()
        t_move = time.monotonic()
        w.teleport((x + 0.15, y, yaw))
        self.assertTrue(wait_until(
            lambda: (w.last_status() or {}).get("state") == "UNDOCKED"
            and not w.last_status()["docked"], 1.0, 0.01),
            "status after the move: {}".format(w.last_status()))
        self.assertLessEqual(time.monotonic() - t_move, 1.0 + 0.05)
        time.sleep(0.3)

        # the next DOCK is a full docking again
        w.status_log.clear()
        run3, status, result = self.dock()
        self.assertEqual(status, SUCCEEDED, result.message)
        self.assertEqual(result.outcome, R.OUTCOME_DOCKED)
        self.assertIn("ALIGNING", w.states())
        self.assert_docked_pose()


# ====================================================================== AC12
class TestGoalRejection(DockTestCase):

    def assert_rejected(self, run, what):
        self.assertFalse(run.accepted, what + " was accepted")
        self.assertLessEqual(run.t_decision - run.t_sent, 1.0, what + ": decision took too long")

    def test_goal_rejection(self, impl):
        """AC12: rejections decided within 1 s and recovery after the cause is gone."""
        # second goal while one is active
        self.prepare(impl, pose=NEAR_START)
        w = self.world
        first = self.accepted_goal()
        second = self.start_goal()
        self.assert_rejected(second, "a second goal")
        status, result = self.finish(first)
        self.assertEqual(status, SUCCEEDED, result.message)
        self.assertEqual(result.outcome, R.OUTCOME_DOCKED)

        # UNDOCK while UNDOCKED
        self.prepare(impl, pose=NEAR_START)
        self.assert_rejected(self.start_goal(DockRobot.Goal.COMMAND_UNDOCK),
                             "UNDOCK while undocked")

        # unknown command
        self.assert_rejected(self.start_goal(7), "command 7")

        # DOCK while a mission is active
        w.set_mission("NAVIGATING")
        time.sleep(0.6)
        self.assert_rejected(self.start_goal(), "DOCK during a mission")
        w.set_mission("COMPLETED")
        time.sleep(0.6)
        run = self.start_goal()
        self.assertTrue(run.accepted, "DOCK after the mission ended was rejected")
        w.cancel(run)
        self.finish(run, 15.0)

        # Nav2 server absent while staging is needed
        self.prepare(impl, pose=FAR_START)
        w.stop_nav_server()
        time.sleep(2.0)
        run = self.start_goal()
        self.assert_rejected(run, "DOCK needing Nav2 without the server")
        self.assertEqual(len(w.nav_goals), 0)
        self.assertFalse(w.commands_since(0.0))
        w.start_nav_server()
        accepted = []

        def retry():
            run = self.start_goal()
            if run.accepted:
                accepted.append(run)
            return bool(accepted)

        self.assertTrue(wait_until(retry, 10.0, 0.3), "never accepted after the server appeared")
        w.cancel(accepted[0])
        self.finish(accepted[0], 15.0)

    def test_goal_rejection_without_tf(self, impl):
        """AC12: no TF map -> base_footprint: rejected, accepted within 2 s once TF exists."""
        self.prepare(impl, fresh=True, tf_enabled=False, pose=NEAR_START)
        w = self.world
        run = self.start_goal()
        self.assert_rejected(run, "DOCK without TF")
        self.assertEqual(len(w.nav_goals), 0)
        self.assertFalse(w.commands_since(0.0), "dock_vel message without TF")
        self.assertEqual(set(w.states()) - {"UNDOCKED"}, set())
        t_tf = time.monotonic()
        w.tf_enabled = True
        accepted = []

        def retry():
            run = self.start_goal()
            if run.accepted:
                accepted.append(run)
            return bool(accepted)

        self.assertTrue(wait_until(retry, 2.0, 0.1), "not accepted within 2 s of the TF")
        self.assertLessEqual(time.monotonic() - t_tf, 2.5)
        w.cancel(accepted[0])
        self.finish(accepted[0], 15.0)


# ====================================================================== AC13
class TestUndock(DockTestCase):

    def test_undock(self, impl):
        """AC13: UNDOCK drives straight out to staging; a cancel stops it."""
        self.prepare(impl, pose=NEAR_START)
        w = self.world
        run, status, result = self.dock()
        self.assertEqual(result.outcome, R.OUTCOME_DOCKED, result.message)
        t0 = time.monotonic()
        run = self.accepted_goal(DockRobot.Goal.COMMAND_UNDOCK)
        status, result = self.finish(run, 60.0)
        self.assertEqual(status, SUCCEEDED, result.message)
        self.assertEqual(result.outcome, R.OUTCOME_UNDOCKED)
        self.assertFalse(result.docked)
        self.assert_idle_state("UNDOCKED")
        self.assertFalse(w.last_status()["docked"])
        pos, yaw = w.pose_error(STAGING)
        self.assertLessEqual(pos, 0.05)
        self.assertLessEqual(yaw, 0.1)
        cmds = w.commands_since(t0)
        self.assertTrue([c for c in cmds if moving(c)], "the robot never moved")
        for c in cmds:
            self.assertGreaterEqual(c["v"], -1e-9, "backwards command while undocking")
        self.assertEqual((cmds[-1]["v"], cmds[-1]["w"]), (0.0, 0.0))

        # dock again and cancel the undocking
        w.teleport(NEAR_START)
        time.sleep(0.6)
        run, status, result = self.dock()
        self.assertEqual(result.outcome, R.OUTCOME_DOCKED, result.message)
        run = self.accepted_goal(DockRobot.Goal.COMMAND_UNDOCK)
        self.wait_state("UNDOCKING", 10.0, moving_cmd=True)
        t_cancel = time.monotonic()
        w.cancel(run)
        status, result = self.finish(run, 15.0)
        self.assertEqual(status, CANCELED)
        self.assertEqual(result.outcome, R.OUTCOME_CANCELED)
        self.assertFalse(result.docked)
        self.assert_stop_after(t_cancel, 0.2, "cancel in UNDOCKING")
        self.assert_idle_state("UNDOCKED")


# ====================================================================== AC20 (runtime)
class TestParameterParity(DockTestCase):

    def fetch(self, name):
        node = self.world
        lister = node.create_client(ListParameters, name + "/list_parameters")
        getter = node.create_client(GetParameters, name + "/get_parameters")
        self.assertTrue(lister.wait_for_service(20.0), name + " has no list_parameters")
        self.assertTrue(getter.wait_for_service(20.0))
        future = lister.call_async(ListParameters.Request())
        self.assertTrue(wait_until(future.done, 10.0))
        names = sorted(future.result().result.names)
        request = GetParameters.Request()
        request.names = names
        future = getter.call_async(request)
        self.assertTrue(wait_until(future.done, 10.0))
        out = {}
        for key, value in zip(names, future.result().values):
            out[key] = (value.type, {1: value.bool_value, 2: value.integer_value,
                                     3: value.double_value, 4: value.string_value}.get(value.type))
        return out

    def test_parameter_parity_runtime(self, impl):
        """AC20 runtime: both running nodes expose identical parameter names and values."""
        self.stop_node()
        procs = {}
        codes = {}
        try:
            for which in ("cpp", "py"):
                procs[which] = DockProcess(which, self.tmp, namespace="/" + which, tag="param")
            params = {which: self.fetch("/{}/dock_manager".format(which)) for which in procs}
        finally:
            for which, proc in procs.items():
                codes[which] = proc.stop()
        self.assertEqual(codes, {"cpp": 0, "py": 0})
        # rclcpp declares qos_overrides.* parameters for the /parameter_events publisher of the
        # node itself; they come from the client library, not from dock_manager
        for which in params:
            params[which] = {k: v for k, v in params[which].items()
                             if not k.startswith("qos_overrides.")}
        only_cpp = sorted(set(params["cpp"]) - set(params["py"]))
        only_py = sorted(set(params["py"]) - set(params["cpp"]))
        self.assertEqual((only_cpp, only_py), ([], []))
        different = {k: (params["cpp"][k], params["py"][k]) for k in params["cpp"]
                     if params["cpp"][k] != params["py"][k]}
        self.assertFalse(different, different)
        for key in ("detection_mode", "staging_distance", "use_sim_time", "max_retries",
                    "cmd_vel_topic"):
            self.assertIn(key, params["cpp"])
        self.assertGreaterEqual(len(params["cpp"]), 51)


@launch_testing.post_shutdown_test()
class TestNodesExitCleanly(unittest.TestCase):

    def test_exit_codes(self):
        self.assertTrue(EXIT_CODES, "no node was ever stopped")
        bad = [(impl, code, log) for impl, code, log in EXIT_CODES if code != 0]
        self.assertFalse(bad, bad)
