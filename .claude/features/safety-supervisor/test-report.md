# Test report: iteration 1

Verdict: FAIL   (17/20: AC1-AC19 plus the full-suite gate as one item; AC4, AC10 and the gate fail)

Build: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon build --packages-up-to bumperbot_bringup bumperbot_mujoco bumperbot_utils --cmake-force-configure`
(no `--symlink-install`). All new tests are registered (`ctest -N`): utils 6 + lint, bringup 11, mujoco pytest 74.
Ports: node/robustness/mux tests ROS_DOMAIN_ID=92, MuJoCo 93, ROS_LOCALHOST_ONLY=1.
Not committed.

## Earlier-feature fix (reviewed, kept)
`src/bumperbot_bringup/test/test_dock_launch_config.py` (`feature_end()`, base..last auto-docking commit 7ea98db) and
`test_mission_launch_config.py` (`feature_end_commit()`, base..last waypoint-mission-manager commit 2b49fdf) compare
base..end commit instead of base..worktree. Asserted files and `--quiet` diff are unchanged, only the right-hand
side changed, the intent ("that feature did not change the launch files") is kept. Both pass.

## Results
| AC | Test | Result | Evidence |
|---|---|---|---|
| AC1 | `bumperbot_utils/test/test_safety_static.py -k interfaces` | PASS | `ros2 interface show` OK for SafetyStatus + 7 old interfaces; 20 fields and 11 constants exact |
| AC2 | `test_safety_logic.py` (172 cases) + gtest `test_safety_logic_cpp` | PASS | shared table `data/safety_cases.yaml`, both registered and passed |
| AC3 | `test_safety_robustness.py` (valid x5, invalid x15, duplicate key) x cpp/py | PASS | every invalid file: exit 1 <= 5 s, `safety_stop: true` published, no release, nothing published on cmd_vel_out; duplicate key: same outcome; valid files: first status INITIALIZING locked |
| AC4 | `bumperbot_mujoco/test/test_safety_supervisor_sim.py::test_footprint_*`, laser tests | FAIL | wheel collision geoms exceed the footprint (see Failures); laser pose and mock/sim agreement PASS |
| AC5 | `test_safety_supervisor.launch.py::test_startup_failsafe[cpp,py]` | PASS | reasons `waiting for scan` / `waiting for TF base_footprint <- laser_link` / `waiting for odom`, zero output >5 s, status >= 4 Hz, ACTIVE <= 1 s; exe path + startup log prove the impl |
| AC6 | `test_stop_zone[cpp,py]` | PASS | stop, brake lock, STOP within 0.25 s; release; forward blocked, -0.1 passes, 0.3 back <= 0.55 s |
| AC7 | `test_slowdown_direction[cpp,py]` | PASS | output = speed_limit(d) within 1e-3 for 0.60...0.06, monotonic; rear/side cases; angular pass-through and clamp |
| AC8 | `test_watchdog`, `_invalid_scans`, `_latch_faults`, `_tf_late` [cpp,py] | PASS | (a)-(g) incl. nan 0.80 / empty ranges / angle_increment 0 / frame changed / empty frame_id |
| AC9 | `test_estop_latch_service_and_input`, `_reset_refusals`, `_contact` [cpp,py] | PASS | trigger 0.15 s, persists 5 s, reset rules (`estop input`, `fault`, `contact zone (front)`), contact latch only when moving |
| AC10 | `test_dock_profile_*` [cpp,py] | FAIL | (b) fallbacks, (c) time limit/re-arm, (d) latency 0.036-0.043 s PASS; (a) with the plan's literal 0.03 m rear obstacle FAIL |
| AC11 | `test_lock_only[cpp,py]` | PASS | no publisher on cmd_vel_unstamped, no subscription on safety/cmd_vel_in (graph), lock on forward obstacle, none for rear obstacle, `max_forward_speed` = speed_limit |
| AC12 | `test_safety_robustness.py` AC12 cases x cpp/py | PASS | SIGINT/SIGTERM: exit 0, last lock true/out 0/status SHUTDOWN, no Traceback; tick exceptions recover; stalled /clock keeps status >= 2 Hz, lock republished, FAULT <= 0.75 s; repeated signals exit 0 |
| AC13 | `bumperbot_bringup/test/test_safety_twist_mux.launch.py` x cpp/py | PASS | real twist_mux, (a)-(f) and locks file unchanged vs main |
| AC14 | `test_safety_launch_config.py` (13 tests) | PASS | real launch expansion: one C++ supervisor (use_sim_time true, enforcement filter), none for legacy, mux remap, unchanged files |
| AC15 | `test_safety_static.py` + `test_parameter_parity_runtime[cpp,py]` | PASS | no sim tokens, ASCII, defaults py == C++ == YAML == plan; runtime get_parameters identical (rclcpp `qos_overrides.*` excluded: framework-declared) |
| AC16 | `test_safety_supervisor_sim.py::test_a..test_e` | PASS | (a) min clearance > 0.03 and no contact, (b) travel <= 0.20 m, (c) forward after rear STOP, (d) FAULT <= 0.7 s, (e) latch and reset |
| AC17 | regression: all existing explore/mission/dock/navigation/mujoco/mapping-functional tests | PASS | see Full suite |
| AC18 | `test_safety_sim_mujoco.launch.py`, smoke and `BUMPERBOT_SAFETY_E2E=1` | PASS | smoke ACTIVE after 5-10 s, one lock publisher, one C++ node, no safety_stop; E2E stop at 0.0488 m clearance, back-off, e-stop/reset (40 s) |
| AC19 | `test_safety_docs.py` + `test_dock_docs.py` | PASS | 52 passed |
| Gate | `colcon test` of msgs, utils, controller, navigation, mapping, mujoco, bringup | FAIL | failures: AC4 (2 cases), AC10 (2 cases); pre-existing lint failures separately below |

## Full suite
`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon test --executor sequential --packages-select ...` took 36 min (navigation 26 min).
`colcon test-result`: 1264 tests, 0 errors, 101 failures, 23 skipped. The 101 failures are:
- 2 x AC4 (mujoco), 2 x AC10 (utils): real defects below.
- Pre-existing lint, not part of this feature (established with git): `bumperbot_utils` flake8 (12 errors, all in
  `bumperbot_utils/safety_stop.py`), uncrustify (`src/safety_stop.cpp`), lint_cmake (`CMakeLists.txt:54`
  `install (PROGRAMS`, blame 38e0e7b9 from 2026-10-01); the legacy node is unchanged
  (`git diff HEAD` empty, asserted by `test_static_safety_stop_legacy_unchanged_by_the_feature`).
  `bumperbot_mapping` flake8 (48), pep257 (20), uncrustify (2): package untouched by the feature
  (`git diff 7ea98db -- src/bumperbot_mapping` empty).
- All new product and test files are lint-clean (flake8, pep257 D213, uncrustify, cppcheck).
- 23 skipped = opt-in E2Es (dock, mission, SLAM, explore, safety; the safety E2E was run separately with
  `BUMPERBOT_SAFETY_E2E=1` and passed twice).
- bumperbot_navigation, bumperbot_msgs, bumperbot_controller, all bringup tests: 0 failures.

## Durations vs registered TIMEOUT
| Test | Duration | TIMEOUT |
|---|---|---|
| test_safety_logic_cpp / test_safety_logic / static / docs | 0.06 / 0.3 / 1.7 / 0.2 s | 120 |
| test_safety_robustness | 83 s | 600 |
| test_safety_supervisor (launch, 17 x 2) | 237 s | 900 |
| test_safety_launch_config | 5 s | 180 |
| test_safety_twist_mux | 52 s | 300 |
| test_safety_sim_mujoco (smoke) | 6 s (E2E run 40 s) | 900 |
| test_auto_explore_mujoco / test_dock_sim_mujoco / test_mission_sim_mujoco | 24 / 6.5 / 6 s | 180 / 1200 / 900 |

## Failures
### AC4: default footprint does not contain the wheel collision geoms
- command: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/bumperbot_mujoco/test/test_safety_supervisor_sim.py -k geom_aabb`
- output (trimmed): `wheel_left_link_collision: AABB x [-0.0330, 0.0330] y [0.0521, 0.1181] exceeds the footprint x [-0.10, 0.10] y +-0.09; corners outside [(-0.033, 0.1181), (0.033, 0.1181)]`, same for the right wheel (y -0.1181).
  base_link (real mesh vertices, +-0.068), casters, IMU and laser geoms are inside.
- root cause: `safety_supervisor.yaml` / `Config.footprint` (`safety_logic.py:~63`, `safety_logic.hpp`) use half width 0.09 m.
  The wheel collision geoms (`robot.xml` sphere r 0.033 at y +-0.0851, same in the URDF) reach y +-0.118; even a real
  0.03 m wide wheel reaches +-0.100. The robot is wider than the footprint at the wheels. Corridors still add
  `lateral_margin`, but `nearest_distance`, contact and clearance are measured from a polygon that is 0.01-0.03 m too narrow.
- suggested fix: widen the default footprint half width to >= 0.12 (and mirror it in the C++ default, the YAML and the case
  table), or state in the plan that wheels are excluded from AC4.

### AC10: dock profile does not give -0.06 for a rear obstacle at 0.03 m (plan scenario)
- command: `launch_test src/bumperbot_utils/test/test_safety_supervisor.launch.py` (`test_dock_profile_plan_scenario[cpp]` and `[py]`)
- output: `rear obstacle at 0.03 m: output [-0.0]`, status `DOCK_APPROACH`, `rear_zone='STOP'`, `max_reverse_speed=0.0`,
  `rear_clearance=0.0299999975`.
- root cause: the rear zone keeps its previous STOP state when the profile switches on. With the normal margin 0.05 a
  0.03 m obstacle is STOP, and the hysteresis then holds STOP while `d <= dock_stop_margin + stop_release_hysteresis`
  = 0.01 + 0.02 = 0.03 (`safety_logic.py:304`, `safety_logic.cpp:341`, state carried by `prev_rear` at
  `safety_logic.py:669`). The plan's example (0.03 m, input -0.15, output -0.06) therefore only works when the profile is
  already active before the obstacle is first seen (run order dependent; it passed for py once in the exploratory run).
  The profile is used when approaching a dock from outside STOP, but a robot already within 0.03 m of the charger
  (or any obstacle within 0.03 m at engage time) cannot reverse until it is farther than 0.03 m.
- suggested fix: reset `prev_rear` (or ignore hysteresis) in the tick where the dock profile switches on, or
  use `dock_stop_margin` as hysteresis base when the profile is active; or change the plan example to > 0.03 m.
- the other AC10 checks use 0.04 m (outside the hysteresis band) and pass for both implementations.
