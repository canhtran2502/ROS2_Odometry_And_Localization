# Test report: iteration 1
Verdict: PASS   (26/26 items: AC1-AC25 + full-suite gate)

Environment: full rebuild without --symlink-install, `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`,
`ROS_DOMAIN_ID=90` (MuJoCo launch tests run alone on 91), `ROS_LOCALHOST_ONLY=1`.
All pytest/gtest/launch tests are registered by the product CMakeLists.txt (confirmed in the colcon test output).
Test-side fixes made this session (tests only, no product code changed):
- `src/bumperbot_bringup/test/test_dock_sim_mujoco.launch.py`: publisher node name is `mujoco_sim` (the executable is `mujoco_sim_node`); the test asserted the executable name, which failed.
- `src/bumperbot_mujoco/test/test_dock_precision_sim.py`: `pytest.importorskip("mujoco")` replaced by a plain import (a test that cannot run must fail, not skip).

| AC | Test | Result | Evidence |
|---|---|---|---|
| AC1 | navigation/test_dock_static.py (interfaces) | PASS | 19 passed |
| AC2 | test_dock_logic.py (129 cases) + test_dock_logic_cpp (gtest) over shared dock_cases.yaml | PASS | 129 passed; gtest passed |
| AC3 | test_dock_robustness.py (first status parity x4 configs, 11 invalid stations x2 impls, 5 invalid configs x2) | PASS | 51 passed in 119 s |
| AC4 | test_dock_geometry_on_map.py | PASS | 4 passed |
| AC5 | test_dock_manager.launch.py::test_dock_success [cpp, py] | PASS | 18.9 s / 19.3 s |
| AC6 | ::test_skip_staging [cpp, py] | PASS | |
| AC7 | ::test_detection_refinement, test_map_only_keeps_the_bias, test_detection_required_missing | PASS | both impls |
| AC8 | ::test_retry_on_slip, test_stall_fails_safe, test_staging_failed | PASS | both impls |
| AC9 | ::test_safety_lock, test_safety_lock_timeout (incl. publisher check: no dock_manager on /safety_stop) | PASS | both impls |
| AC10 | ::test_cancel_paths, test_error_paths | PASS | both impls |
| AC11 | ::test_docked_flag_reset | PASS | both impls |
| AC12 | ::test_goal_rejection, test_goal_rejection_without_tf | PASS | both impls |
| AC13 | ::test_undock | PASS | both impls |
| AC14 | test_dock_robustness.py (signals during docking/navigation/idle, stalled clock, running sim clock) | PASS | 51 passed |
| AC15 | test_battery_logic.py | PASS | 58 passed |
| AC16 | test_battery_nodes.launch.py (6 tests) | PASS | 11 passed, 55.8 s |
| AC17 | test_battery_nodes.launch.py (4 tests) + static grep | PASS | |
| AC18 | bringup/test_dock_twist_mux.launch.py (real twist_mux) | PASS | 11 passed, 22 s |
| AC19 | bringup/test_dock_launch_config.py | PASS | 14 passed |
| AC20 | test_dock_static.py + test_parameter_parity_runtime [cpp, py] | PASS | |
| AC21 | mujoco/test_dock_precision_sim.py (12 runs + undock) | PASS | 13 passed |
| AC22 | mujoco/test_dock_marker_sim.py + ground-truth check in the AC24 launch test | PASS | |
| AC23 | test_dock_docs.py | PASS | 17 passed |
| AC24 | test_dock_sim_mujoco.launch.py::test_stack_up (domain 91) | PASS | OK after 1.0-2.5 s (limit 150 s) |
| AC25 | same file, BUMPERBOT_DOCK_E2E=1 (domain 91) | PASS | 50 s of 900 s: mission CANCELED, DOCK outcome 0, UNDOCK outcome 1 (logs), ground-truth tolerances asserted by the test |
| Gate | colcon test msgs/navigation/mujoco/bringup + test-result | PASS | 648 tests, 0 errors, 0 failures, 14 skipped (cppcheck, pre-existing opt-in mission/slam MuJoCo tests, AC25 which was run separately) |

Launch-test durations vs registered TIMEOUT: test_dock_manager 439 s / 900; test_battery_nodes 57 s / 400;
test_dock_robustness 119 s / 600; test_dock_twist_mux 23 s / 180; test_dock_sim_mujoco 4 s (AC25: 50 s) / 1200.
Lint (flake8, pep257, uncrustify, cppcheck, lint_cmake, xmllint) all passed.

## Failures
None remaining. Observations (not AC failures): on shutdown, `mujoco_sim_node`, `dock_marker_sim` and `joy_teleop` print an
`rcl_shutdown already called` traceback and exit 1 when stopped by launch SIGINT (pre-existing pattern in `mujoco_sim_node.py` main;
`dock_marker_sim.py:148` repeats it). Cosmetic, outside the ACs.

---

# Test report: iteration 2 (review fixes M1-M4, L2, L3, docs)
Verdict: FAIL   (25/27 items: AC1-AC25 PASS; full-suite gate FAIL; review-fix checks FAIL)

The only failing test is a genuine product defect in the M4 fix (below). Nothing else fails; lint is clean.

Environment: full rebuild without --symlink-install (`--cmake-force-configure`), `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`,
`ROS_DOMAIN_ID=90`, `ROS_LOCALHOST_ONLY=1` (MuJoCo launch test AC24/AC25 alone on 91, shutdown tests on 93).
Gate: `colcon test --packages-select bumperbot_msgs bumperbot_navigation bumperbot_mujoco bumperbot_bringup`
-> `colcon test-result`: 737 tests, 0 errors, 2 failures (the same one test, reported by xunit and by ctest), 14 skipped (cppcheck + opt-in MuJoCo tests).

## Is the C++ dock_manager really exercised? (explicit check, requested)
Yes. `test_dock_manager.launch.py` runs as `launch_tests[cpp]` and `launch_tests[py]` (log: "Starting test run ...launch_tests[cpp]",
"Ran 28 tests" for each). `test_dock_robustness.py` is parametrized by the `impl` fixture (ids `[cpp]`, `[py]`, 65 tests; the 14 new M1 tests
ran as `[cpp-0.5] [cpp-2.0] [py-0.5] ... [cpp-SIGINT] ...`). Because the launch parametrization only injects a name, a new guard test
`TestImplementationUnderTest.test_the_requested_implementation_is_running[cpp|py]` checks `/proc/<pid>/exe` and the command line of the node
under test: for `cpp` the process is an ELF binary `lib/bumperbot_navigation/dock_manager` (no `dock_manager.py` in the command line); for `py` a python
interpreter running `dock_manager.py`. Both pass. No test bug found; the coder simply saw only the first parametrization in the console tail.

## Review-fix tests added (tests only; no product file touched)
| Fix | Test | Impl | Result | Evidence |
|---|---|---|---|---|
| M1 cancel | test_dock_robustness.py::test_m1_cancel_before_nav2_answers_cancels_the_late_goal (delay 0.5 s and 2.0 s) | cpp, py | PASS | goal CANCELED within 2.5 s; mock Nav2 got exactly 1 cancel request for the late-accepted goal, still 1 after 1.5 s |
| M1 shutdown | ::test_m1_shutdown_before_nav2_answers_cancels_the_late_goal (SIGINT, SIGTERM) | cpp, py | PASS | exit 0, no traceback, ABORTED/ERROR, exactly 1 late cancel |
| M1 clock stall | ::test_m1_clock_stall_before_nav2_answers_cancels_the_late_goal | cpp, py | PASS | ABORTED/ERROR within 6 s, exactly 1 late cancel, node alive |
| M1 answer timeout | ::test_m1_answer_timeout_cancels_every_late_goal | cpp, py | PASS | cancels == Nav2 goals (each exactly once) |
| M1 control | ::test_m1_goal_answered_in_time_is_not_canceled_early | cpp, py | PASS | no cancel, dock succeeds |
| M2 dock_manager | test_dock_manager.launch.py::TestMissionInterlock (NAVIGATING, RETURNING_TO_DOCK, ALIGNING+DWELLING, DOCKING+NAVIGATING) | cpp, py | PASS | ABORTED, OUTCOME_ERROR, message contains "a mission became active during the dock goal", ended <= 2 s, Nav2 goal canceled, zero cmd_vel, state FAILED, fresh goal accepted afterwards; control: IDLE/COMPLETED/CANCELED/ABORTED/DOCK_FAILED do not abort |
| M2 battery_monitor | battery_cases.yaml (cancel_repeats_*, cancel_cadence_*, mission_started_during_docking_*, mission_started_during_charging_*, mission_active_when_dock_request_begins_*, mission_during_charging_does_not_undock_or_dock, cancel_is_not_repeated_without_an_active_mission) + test_battery_nodes::test_mission_started_during_docking_is_canceled, ::test_mission_started_during_charging_is_canceled | - | PASS | cancel arrives within 2 s, repeated >= 3 times, stops after the mission ends, no extra dock goal, state unchanged |
| M3 logic | battery_cases.yaml (unanswered_dock_request_*, ..._server_gone_*, accepted_dock_goal_*_never_timed_out, server_lost_during_an_accepted_*, server_flapping_*, abandoned_dock_requests_end_in_error_*, undock variants) | - | PASS | ABANDON_DOCK at exactly dock_request_timeout, attempts counted, retry/ERROR as specified, accepted goals with server up never time out (t=5000 s) |
| M3 node | test_battery_nodes::test_unanswered_dock_request_is_dropped_counted_and_retried, ::test_dock_manager_disappearing_during_a_goal_is_recovered, ::test_accepted_goal_is_not_timed_out, ::test_unanswered_undock_request_ends_in_error, ::test_dock_manager_disappearing_during_undock_ends_in_error, ::test_accepted_undock_goal_is_not_timed_out | - | PASS | 3 DOCK goals then ERROR, every late-accepted goal canceled once (3 cancels), recovery after the server returns, no timeout of running goals |
| M4 cancel retry | test_battery_nodes::test_cancel_is_repeated_while_the_mission_stays_active, ::test_cancel_service_not_ready_is_retried, ::test_refused_cancel_response_is_checked_and_retried, ::test_mission_cancel_timeout_still_sends_dock_with_repeated_cancels | - | PASS | gaps 0.4-1.5 s, warning "cancel service is not available", "refused" logged, DOCK still sent after mission_cancel_timeout |
| M4 stale, dock_manager | TestMissionInterlock::test_stale_latched_mission_is_ignored_at_acceptance (+ control: live publisher still rejects DOCK) | cpp, py | PASS | accepted once the mission publisher is gone |
| M4 stale, battery_monitor | test_battery_nodes::test_latched_mission_without_publisher_is_ignored_after_5_s | - | **FAIL** | see Failures |
| M4 stale control | ::test_latched_mission_with_a_publisher_is_not_stale | - | PASS | 7 s, still CANCELING_MISSION, no "stale" |
| L2 | bumperbot_mujoco/test/test_shutdown_signals.py (dock_marker_sim idle/publishing, mujoco_sim_node headless `viewer:=false` idle/driving, each SIGINT and SIGTERM, + static pattern check) | - | PASS | 10 passed in 17.6 s: exit 0, none of "Traceback", "rcl_shutdown already called", "RCLError". Headless launch is feasible. Sanity: the iteration-1 `dock_marker_sim.py` exits 1 with the traceback under the same SIGINT. The AC25 launch log now shows `dock_marker_sim` and `mujoco_sim_node` "process has finished cleanly" |
| L3 | TestCancelAfterDecision: ::test_cancel_right_after_already_docked_decision, ::test_cancel_right_after_verify_dock_and_undock, ::test_cancel_during_motion_phases_still_cancels | cpp, py | PASS | goal ends SUCCEEDED (ALREADY_DOCKED / DOCKED / UNDOCKED, docked flag correct); evidence that the window is really hit: "10 of 10 cancels reached the running goal" (cancel return code ERROR_NONE) for both impls; cancel during ALIGNING/DOCKING still CANCELED with zero cmd_vel within 0.2 s (also AC10 test_cancel_paths) |
| Docs | test_dock_docs.py: 10 new tests (27 total) | - | PASS | interlock section (all three parts, exact message, OUTCOME_ERROR), cancel-after-decision paragraph, README line, doc/03 `Service client: execute_mission/_action/cancel_goal (action_msgs/srv/CancelGoal)` and no "Action clients", "real_robot.launch.py does not start safety_stop" in README/doc10/doc12 and the fact itself (the launch file has no `safety_stop`), doc/12 limitations (a)-(d) with their key terms; plus a check that battery_monitor.py has exactly one ActionClient (DockRobot) and one service client |

Test-infrastructure additions (tests only): `mock_dock_world.py` got `nav_accept_delay` (delays the Nav2 goal response), `nav_cancel_requests`,
`drop_mission_publisher()`, `cancel_on_state()` and the cancel response code; the battery MockWorld got `dock_delay`, `cancel_mode`,
`drop_dock_server/cancel_service/mission_publisher`.

## Changed expectation (intended M4 behavior, NOT a regression)
`mission_cancel_timeout_sends_dock_anyway` in `src/bumperbot_navigation/test/data/battery_cases.yaml` failed after the iteration-2 changes at t=10.9 with
`['CANCEL_MISSION'] != []`. This is due to the intended M4 behavior: the mission cancel is now repeated every `dock_retry_delay` (5 s) while the
mission stays active, so the update at t=10.9 (9.9 s after the cancel at t=1) correctly emits `CANCEL_MISSION`. The expected sequence was updated to
t=1 CANCEL, t=5 none, t=10.9 CANCEL, t=11 SEND_DOCK (the dock-anyway timeout itself is unchanged), with an explanatory comment in the YAML. New cases cover the retry cadence
(`cancel_repeats_every_retry_delay_...`, `cancel_cadence_follows_dock_retry_delay`, `mission_cancel_timeout_with_updates_every_second`) and the timeout paths.
`run_monitor` now also feeds `accepted` and `server` event keys (`update(now, server_ready)`); old cases are unchanged otherwise. No assertion was weakened.

## Failures
### Review-fix M4 (battery_monitor stale mission): UnboundLocalError kills the node's spin thread
- command: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ROS_DOMAIN_ID=90 launch_test src/bumperbot_navigation/test/test_battery_nodes.launch.py`
  (test `TestMissionInterlockAndWatchdogs.test_latched_mission_without_publisher_is_ignored_after_5_s`)
- output (trimmed):
  ```
  AssertionError: state CANCELING_MISSION not reached: ['WAITING_FOR_BATTERY', 'MONITORING']
  Exception in thread monitor_spin:
    File ".../battery_monitor.py", line 187, in update
      self.check_stale_mission()
    File ".../battery_monitor.py", line 180, in check_stale_mission
      if stale:
  UnboundLocalError: local variable 'stale' referenced before assignment
  ```
- root cause: `src/bumperbot_navigation/bumperbot_navigation/battery_monitor.py:164-182`. `stale` is assigned only in the `elif ... >= MISSION_STALE_TIMEOUT` branch.
  When the publisher has just disappeared (`mission_gone_since is None` branch, line 172) the `with` block ends without a `return` and `if stale:` (line 180) raises.
  The exception propagates out of the timer callback and terminates the `monitor_spin` executor thread, so `battery_monitor` stops updating completely
  (no more state machine, no docking) the first time `mission_manager` stops publishing after a status was seen (crash, restart, normal shutdown). This is exactly the scenario the M4 fix is for.
  The existing tests did not catch it because a `/mission/status` publisher never disappeared in them.
- suggested fix: set `stale = False` before the `with self.lock:` block (or return after `self.mission_gone_since = now`).
  The new test passes once the warning "treated as stale" is logged and DOCK is sent about 5 s after the publisher is gone.
- Reproducible: yes, 2 of 2 full runs (gate 1 and gate 2).

## Observations (not failures)
- `test_parameter_parity_runtime` (AC20) failed once per full run in two earlier full runs (a `list_parameters` reply not received within 10 s, once as [cpp], once as [py]); it passed in isolation (8/8) and in the final gate. Pattern: a request lost while the service reply path was still being discovered (the old test sent the request once). The test now re-sends the call up to 3 times (8 s each); in the final gate no retry was needed. This is a test-side robustness fix; no product cause was found.
- `joy_teleop` (third-party) still prints `rcl_shutdown already called` and exits 1 on launch shutdown (also in AC25). Not part of this feature.
- New tests are deterministic and headless; the MuJoCo shutdown tests use a private domain 93.

## Regression and gate
| Item | Result | Evidence |
|---|---|---|
| AC1-AC23 | PASS | all previous tests pass again (test_dock_logic 129, gtest 5, test_dock_static 19, test_battery_logic 77, test_dock_docs 27, test_dock_robustness 65, test_dock_manager 28 per impl, test_dock_twist_mux, test_dock_launch_config) |
| AC16/AC17 | PASS | the 10 original battery node tests pass; only the new stale-mission test fails |
| AC24 | PASS | test_dock_sim_mujoco.launch.py::test_stack_up |
| AC25 (BUMPERBOT_DOCK_E2E=1, domain 91) | PASS | 3 tests in 50.8 s: dock outcome 0 (docked), undock outcome 1 (undocked), mujoco_sim_node and dock_marker_sim exit cleanly |
| Full-suite gate | FAIL | 737 tests, 2 failures (= the M4 stale-mission test); flake8, pep257, uncrustify, cppcheck, lint_cmake, xmllint all pass |

Launch-test durations vs registered TIMEOUT (ctest): test_dock_manager 621 s / 900 (margin 31 %; was 439 s before 14 new launch tests, no change needed, but it is above the 25 % line only narrowly: a further 40 s of tests would need a larger TIMEOUT);
test_battery_nodes 150 s / 400 (62 %); test_dock_robustness 218 s / 600 (64 %); test_dock_twist_mux 23 s / 180; test_dock_sim_mujoco 6 s (AC25: 51 s) / 1200.
No TIMEOUT changed.
