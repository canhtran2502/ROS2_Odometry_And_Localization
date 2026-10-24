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
