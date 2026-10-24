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
