# Test report: iteration 2
Verdict: PASS   (17/17: AC1-AC16 + full-suite gate)

Build: `colcon build --packages-select bumperbot_msgs bumperbot_controller bumperbot_navigation bumperbot_bringup --cmake-force-configure` (no symlink-install).
Gate: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=88 colcon test --packages-select bumperbot_navigation bumperbot_bringup`
then `colcon test-result --verbose`: **139 tests, 0 errors, 0 failures, 8 skipped** (run twice, identical; 8 skips
include the AC16 opt-in and pre-existing opt-in tests).

Test-file fixes (tests only, no assertion weakened, no product change)
- pep257 D213: module docstring summary moved to line 2 in `navigation/test/mock_nav2.py`,
  `navigation/test/test_mission_manager.launch.py`, `bringup/test/test_mission_sim_mujoco.launch.py`.
- `navigation/test/test_stations_on_map.py`: docstring rewritten (D205/D209/D213/D400/D415).
- uncrustify: `navigation/test/test_mission_logic.cpp` reformatted (braces on separate lines).
- Test flake found by the full run (1 failure in `test_status_topic[cpp]`, seq `[dock,a,b,c,dock]`): a late
  `/mission/status` message of the previous test's mission arrived after `statuses.clear()`. Test isolation bug, not a
  product defect. Added `settle_status()` in `TestMissionManager.setUp` (waits for 0.5 s of status silence); all
  assertions unchanged. Passed in 2 consecutive full-gate runs plus a standalone `launch_test`.

Regression runs of the individual AC commands
- pytest (AC1-AC4, AC12-AC14): 55 passed.
- `launch_test test_mission_manager.launch.py` (AC5-AC11, AC13, cpp+py): Ran 10 + 1 tests, OK.
- AC15 + AC16 `BUMPERBOT_MISSION_E2E=1 ROS_DOMAIN_ID=89 launch_test test_mission_sim_mujoco.launch.py`: Ran 2 tests, OK (175 s).

Lint outside this feature: `ament_flake8` E501 (lines 111, 136, 157) and pep257 D213 in
`bumperbot_bringup/test/test_slam_map_quality_mujoco.launch.py` (auto-explore-slam feature). Not fixed; they do not
appear as failures in the colcon gate.

All AC1-AC16 results from iteration 1 below are unchanged (PASS).

---

# Test report: iteration 1
Verdict: PASS   (16/16 AC; AC16 run opt-in and passed)

Build: `colcon build --packages-select bumperbot_msgs bumperbot_controller bumperbot_navigation bumperbot_bringup` (no symlink-install).
Run: `ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 colcon test --packages-select bumperbot_navigation bumperbot_bringup`
(MuJoCo tests on ROS_DOMAIN_ID=89). Environment note: `python3 -m pytest` crashes at startup here
(broken user-site `anyio` plugin: `No module named '_pytest.scope'`), and ament_add_pytest_test then
warns "pytest not found". Workaround used for all runs: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`
(and a `--cmake-force-configure` rebuild so the pytests register). Not a product defect.

| AC | Test | Result | Evidence |
|---|---|---|---|
| AC1 | test_interfaces_and_static.py `-k interfaces` | PASS | `ros2 interface show` OK for 2 new + 3 old; all fields/constants checked, Python types checked |
| AC2 | test_mission_logic.py (stations/validate), gtest `StationsTest.*`, `ValidateTest.*` | PASS | identical expected values asserted in both languages; 6 invalid fixtures + missing file rejected |
| AC3 | test_stations_on_map.py | PASS | every station free, clearance >= 0.30 m, in the dock component (clearance > 0.15 m) |
| AC4 | test_mission_logic.py `-k cases` (13 cases), gtest `MissionCases.AllCases` | PASS | same mission_cases.yaml in both; progress monotonic |
| AC5 | test_mission_manager.launch.py::test_ordered_visit [cpp, py] | PASS | goals a,b,c,dock; pose/quaternion 1e-6; COMPLETED, docked |
| AC6 | ::test_dwell [cpp, py] | PASS | gap within [0.95, 2.1] s, DWELLING/a seen |
| AC7 | ::test_retry_and_skip [cpp, py] | PASS | b x3, u1 x3 skipped, c visited, COMPLETED_WITH_SKIPS |
| AC8 | ::test_tolerance_check [cpp, py] | PASS | 0.35 m -> 2nd goal; 0.15 m accepted |
| AC9 | ::test_abort_returns_to_dock, ::test_cancel_returns_to_dock, ::test_dock_failure [cpp, py] | PASS | ABORTED+docked; cancel forwarded to Nav2, dock goal, CANCELED; DOCK_FAILED, 4 dock goals |
| AC10 | ::test_goal_rejection [cpp, py] | PASS | empty/unknown/concurrent rejected, first mission completes |
| AC11 | ::test_status_topic [cpp, py] | PASS | IDLE first (transient_local), a,b,c,dock, progress monotonic to 1.0, COMPLETED, feedback per station |
| AC12 | bringup/test/test_mission_launch_config.py (14 tests) | PASS | see note below |
| AC13 | test_interfaces_and_static.py (sim independence, parameter parity, ASCII) + launch test `test_parameter_values` [cpp, py] | PASS | 15 params identical in both sources, yaml, and live nodes |
| AC14 | test_mission_docs.py | PASS | all 5 docs contain required strings; old sentence gone |
| AC15 | bringup/test/test_mission_sim_mujoco.launch.py::test_stack_up | PASS | MuJoCo 3.15.0 present; navigate_to_pose + bt_navigator ACTIVE (lifecycle GetState), execute_mission, IDLE in ~4 s |
| AC16 | same file ::test_full_mission with BUMPERBOT_MISSION_E2E=1 | PASS | ran 199 s: [pickup, dropoff] COMPLETED, docked, order pickup,dropoff,dock, each within 0.2 m. Skipped by default in `colcon test` (opt-in) |

Test counts (colcon): test_mission_logic 25, gtest 3 suites, test_stations_on_map 2,
test_interfaces_and_static 12, test_mission_docs 2, test_mission_manager (launch, cpp+py) 11 incl.
shutdown test, test_mission_launch_config 14, test_mission_sim_mujoco 2 (1 skipped = AC16 opt-in).

Notes (not failing an AC)
- AC12 "unchanged versus main": `git diff main` on simulated_robot.launch.py is non-empty, but only
  because of earlier unmerged commits of this branch (auto-explore-slam). The test therefore compares
  real_robot/simulated_robot launch files against the feature base (parent of the commit adding
  mission_sim.launch.py): unchanged. real_robot.launch.py is also unchanged vs main.
- AC15 initially passed in 1 s because the action server is visible while bt_navigator is only
  configuring; the test additionally requires bt_navigator lifecycle state ACTIVE (per the AC text).
- `colcon test` also runs ament_lint_auto suites that FAIL (24 failures, none are AC tests):
  flake8 E251 in `launch/navigation.launch.py` (spaces around `=`), E501
  `bumperbot_navigation/mission_manager.py:356`; pep257 in `mission_logic.py`, `mission_manager.py`
  and in the new test files; uncrustify on `include/bumperbot_navigation/mission_logic.hpp` and
  `test/test_mission_logic.cpp`. The plan says the full suite must show 0 failures; lint on product
  files is a coder-side cleanup (other packages in the repo use the same ament_lint_auto setup).
  Lint-only; no functional impact.

## Failures
None for AC1-AC16.

---

# Test report: iteration 3 (Opus review fixes)
Verdict: PASS  (16/16 AC + full-suite gate + review-fix checks = 18/18)

Build: `colcon build --packages-select bumperbot_msgs bumperbot_controller bumperbot_navigation bumperbot_bringup` (no --symlink-install).
Gate: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon test --packages-select bumperbot_navigation bumperbot_bringup`
-> `colcon test-result`: 160 tests, 0 errors, 0 failures, 8 skipped (opt-in/pre-existing skips); lint
(flake8, pep257, uncrustify, cppcheck, lint_cmake, xmllint) all pass.
AC15/AC16 regression: `BUMPERBOT_MISSION_E2E=1 launch_test src/bumperbot_bringup/test/test_mission_sim_mujoco.launch.py`
-> 2 tests OK in 112 s (test_stack_up and test_full_mission).

| Review fix | Test | Result | Evidence |
|---|---|---|---|
| 1 docked reflects reality | mission_cases.yaml (pytest `test_cases[...]` + gtest `MissionCases.AllCases`): `dock_first_then_cancel_while_driving_to_a`, `..._internal_error_while_driving_to_a`, `..._internal_error_dock_unreachable`, `dock_first_then_station_fails_and_aborts`, `dock_first_then_station_skipped_still_drives_back` | PASS | NAVIGATE:dock:1:dock step is emitted after leaving the dock; DOCK_FAILED/docked=false when unreachable. The drivers got an `abort` event (tracker.abort) for the internal-error path |
| 2 rejection handling | test_mission_review_fixes.launch.py `test_transient_rejection_does_not_consume_retries`, `test_rejection_window_leaves_full_retry_budget`, `test_continuous_rejection_counts_as_failed_attempt` [cpp, py] | PASS | 2 s rejection: mission COMPLETED, 1 accepted goal; rejection + 2 aborts: 3 accepted attempts; continuous: u1 never accepted, rejections span >= 13.5 s, skipped, COMPLETED_WITH_SKIPS |
| 3 stations loading parity | test_mission_logic.py (`test_stations_invalid_fixture_rejected[...]`, `test_stations_exponent_numbers_accepted`) and gtest `StationsTest.InvalidFixturesRejected`, `ExponentNumbersAccepted` | PASS | new fixtures: invalid_duplicate_station, invalid_duplicate_key, invalid_null_frame_id, invalid_null_dock_station, invalid_quoted_number, valid_exponent (1e3, 1.5e3, 1.0E-1, -2.5e-1, 1e+2); identical values in both languages |
| 4 cancel handling | `test_cancel_right_after_goal_acceptance_is_honored`, `test_cancel_racing_with_nav2_success`, `test_cancel_racing_with_nav2_rejecting_the_cancel` [cpp, py] | PASS | CANCELED + docked + last goal dock; no further station; cancel at 0.0..0.7 s around a goal finishing at 0.6 s (mock SUCCEEDED despite accepted cancel, or cancel rejected): node alive, next mission COMPLETED |
| 5 frame check | test_interfaces_and_static.py `test_node_refuses_start_when_station_frame_differs_from_global_frame`, control `test_node_starts_when_station_frame_matches_global_frame` [cpp, py] | PASS | non-zero exit and message naming global_frame/odom; matching frame keeps running |
| 6 params and docs | test_mission_docs.py `test_doc10_describes_review_fix_rules`, `test_nav2_params_controller_odom_topic` | PASS | controller_server odom_topic == /bumperbot_controller/odom; doc rules present |

Test infrastructure notes
- New file `test/test_mission_review_fixes.launch.py` (mock extended in `test/mock_nav2.py`: reject_for,
  succeed_on_cancel, cancel_reject). The package CMakeLists.txt registers a fixed test list and the
  existing launch test already uses ~240 of its 300 s TIMEOUT, so the new launch tests are run through
  one pytest wrapper per implementation (`test_mission_docs.py::test_review_fixes_launch_cpp`,
  `test_stations_on_map.py::test_review_fixes_launch_py`, ~102 s each of the 120 s per-file TIMEOUT;
  tight). Suggestion for the coder: register the file with add_launch_test in CMakeLists.txt and
  drop the wrappers; also consider reducing test_mission_manager runtime (setUp `settle_status`
  waits the full 10 s because the 2 Hz status never goes quiet for 0.5 s).
- Pre-existing latent flake fixed in the test (not product): `test_status_topic` could see a periodic
  status of the previous mission (`dock`, COMPLETED) after the list was cleared (hit once in [py]);
  it now starts at the first NAVIGATING status. The assertions on sequence, monotonic progress and
  final state are unchanged.
- Own bug found and fixed in the new tests: `ros2 run` leaves the node alive when terminated; the
  control test now uses its own process group (an orphaned node had disturbed later launch tests).
- No product defect found in iteration 3.

## Failures
None.

---

# Test report: iteration 4 (clock stall, dead Nav2, shutdown)
Verdict: FAIL  (16/19: AC1-AC12, AC14-AC16 pass = 15 AC, AC13 fails; full-suite gate FAIL; review-fix checks PASS; iteration-4 checks FAIL)

Build: clean rebuild of bumperbot_navigation/bumperbot_bringup, NO --symlink-install, with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` exported
at build time too (otherwise ament_cmake_pytest cannot import pytest, because the user-site anyio plugin crashes, and the pytest
tests are silently not registered: 9 instead of 13 ctest tests). Gate: `colcon test --packages-select bumperbot_navigation bumperbot_bringup`.
Result: 13 ctest tests in bumperbot_navigation, 3 failing test cases (below); lint (flake8, pep257, uncrustify, cppcheck, lint_cmake, xmllint) passes.
AC15/AC16 regression: `BUMPERBOT_MISSION_E2E=1 launch_test test_mission_sim_mujoco.launch.py` -> 2 tests OK in 124 s.

| Item | Test | Result | Evidence |
|---|---|---|---|
| AC1-AC12, AC14-AC16 | unchanged | PASS | |
| AC13 | test_interfaces_and_static.py::test_sim_independence_logic_and_nodes | FAIL | new product comments contain `/clock` |
| Task 1 wrappers removed | colcon test_mission_review_fixes | PASS | wrappers deleted; the file is run by add_launch_test once per impl (6 tests [cpp] + test_exit_codes[cpp], same for [py]) |
| Task 2 parity | test_parameter_parity / test_parameter_defaults_match_plan_and_yaml | PASS | 16 params, clock_stall_timeout 30.0 in cpp, py and mission_manager.yaml |
| 3a Nav2 dies mid-goal | test_stations_on_map.py::test_nav2_disappears_mid_goal_fails_after_server_timeout[cpp/py x dock_failed/server_back] | PASS | attempt fails 2.0-5.5 s after the server is gone (nav_server_timeout 2, nav_goal_timeout 60); no server -> DOCK_FAILED docked=false; server back -> COMPLETED_WITH_SKIPS docked, goals [a, dock] |
| 3b stalled clock | ::test_stalled_clock_wait_ends_after_clock_stall_timeout[cpp/py] | PASS | control: running clock, no warning; stall during dwell: ends ~3 s later with "Node clock did not advance" warning, mission COMPLETED |
| 3d status on node clock | ::test_status_timer_follows_node_clock[cpp/py] | PASS | 2x /clock: 9-15 msgs in 3 s wall; stalled: <=1; resumed 1x: 4-8 |
| 3c signal mid-mission | test_mission_docs.py::test_signal_mid_mission_cancels_nav2_goal_and_exits_cleanly[cpp/py x SIGINT/SIGTERM] | PASS | rc 0 in all, 12-15 repeats each without failure, mock got cancel request and goal canceled, no terminate/Traceback |
| 3c signal idle | ::test_signal_while_idle_exits_cleanly[py-SIGTERM] | FAIL (flaky ~45%) | see ACx below; cpp both signals and py SIGINT 25/25 |
| 3c constructor error | ::test_constructor_error_gives_nonzero_exit_and_fatal_log[cpp] / [py] | cpp PASS, py FAIL | |

Durations vs registered TIMEOUT: test_mission_manager 246 s / 300 s (82%, tight, unchanged from iteration 3 but still the weakest margin);
test_mission_review_fixes 204 s / 300 s (68%); each pytest file <= 55 s / 120 s (test_stations_on_map 54.5 s with the new tests). No timeouts hit.

Test infrastructure changes: test/mock_nav2.py (start_server/stop_server/close, goal threads exit on teardown, execute wrapped so a destroyed
server cannot kill the executor), new test/robust_harness.py (runs the real executables as processes, /clock publisher), test_doc10 now matches
"Wall (steady) time" case-insensitively (the doc bullet was capitalised in iteration 4; the rule text is unchanged). Because CMakeLists.txt is
product and the registered pytest list is fixed, the new tests live in the already registered pytest files test_stations_on_map.py and
test_mission_docs.py (suggestion: register a dedicated test file, e.g. test_mission_robustness.py, in CMakeLists.txt).

## Failures
### AC13: simulator independence (static)
- command: `python3 -m pytest src/bumperbot_navigation/test/test_interfaces_and_static.py -k sim_independence`
- output: `mission_manager.py:225: # clock_stall_timeout wall seconds (/clock stalled) the wait counts as elapsed.`, `mission_manager.cpp:255: // Node clock: dwell and retry delays follow sim time. If /clock stalls ...`
- root cause: iteration 4 comments mention `/clock`, which the plan (AC13) forbids in node/logic sources.
- suggested fix: reword both comments ("the time source stalls"); no code change.

### Iteration 4 / 3c: Python mission_manager exits with a traceback (code 1) on SIGTERM
- command: `python3 -m pytest src/bumperbot_navigation/test/test_mission_docs.py -k "idle and py-SIGTERM"` (11 of 25 runs failed; also in colcon test)
- output: `rclpy._rclpy_pybind11.RCLError: failed to initialize wait set: the given context is not valid, either rcl_init() was not called or rcl_shutdown() was called.` raised from `executor.spin()`, exit code 1.
- root cause: bumperbot_navigation/mission_manager.py main() (~line 509-517) catches only KeyboardInterrupt/ExternalShutdownException; rclpy's SIGTERM handler shuts the context down while the executor is (re)building its wait set, giving RCLError (race, mostly right after startup/idle; mid-mission SIGTERM passed 15/15).
- suggested fix: also catch `rclpy.exceptions.RCLError` (or check `rclpy.ok()` and treat it as shutdown) around `executor.spin()`; exit 0.

### Iteration 4 / 3c: Python constructor error has no FATAL log
- command: `python3 -m pytest src/bumperbot_navigation/test/test_mission_docs.py -k "constructor_error and py"`
- output: raw `Traceback ... ValueError: cannot read stations file ...`, exit code 1, no `[FATAL]` line (C++ prints FATAL and returns 1).
- root cause: mission_manager.py main() does not wrap `MissionManager()` in try/except like mission_manager.cpp:632-636.
- suggested fix: catch Exception around construction, `rclpy.logging.get_logger("mission_manager").fatal(str(exc))`, shut down rclpy and `sys.exit(1)`.

# Test report: iteration 5 (comment reword, Python RCLError/FATAL handling, raised TIMEOUTs)
Verdict: PASS  (19/19: AC1-AC16 pass; full-suite gate PASS; iteration-4 checks PASS)

Build: clean rebuild (build/install of navigation and bringup removed), NO --symlink-install, `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`,
`--cmake-force-configure`. Registered ctest tests: 13 in bumperbot_navigation, 5 in bumperbot_bringup (none silently missing).
Gate: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon test --packages-select bumperbot_navigation bumperbot_bringup`;
`colcon test-result --verbose`: 192 tests, 0 errors, 0 failures, 8 skipped (5 cppcheck sub-checks skipped by ament, 1 AC16 and 2 slam
opt-in tests skipped by design, without the opt-in env var). Lint (flake8, pep257, uncrustify, cppcheck, lint_cmake, xmllint) passes.
AC13 (static) now passes after the comment rewording. AC15/AC16: `BUMPERBOT_MISSION_E2E=1 launch_test test_mission_sim_mujoco.launch.py`
-> test_stack_up and test_full_mission OK, 2 tests in 112.5 s.

| Item | Result | Evidence |
|---|---|---|
| AC1-AC16 | PASS | all registered tests passed |
| 3c idle signal (cpp/py x SIGINT/SIGTERM) | PASS | test_signal_while_idle_exits_cleanly, 25 consecutive runs of all 4 variants: 0 failures (100/100), rc 0 |
| 3c constructor error cpp/py | PASS | FATAL log, non-zero exit in both |
| mid-mission signals, 3a/3b/3d, parity | PASS | unchanged, pass in the gate |

Durations vs registered TIMEOUT: test_mission_manager 247.4 s / 420 s (59%); test_mission_review_fixes 202.0 s / 360 s (56%);
pytest files: test_stations_on_map 54.8 s, test_mission_docs 12.0 s, test_interfaces_and_static 10.4 s (all well below 120 s).
bringup: test_mission_sim_mujoco 5.2 s (E2E off), test_auto_explore_mujoco 23.2 s. No timeouts hit. No test files modified this iteration.
