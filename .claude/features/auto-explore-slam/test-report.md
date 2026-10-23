# Test report: iteration 2
Verdict: PASS   (8/8 AC)

| AC | Test | Result | Evidence |
|---|---|---|---|
| AC1 | `src/bumperbot_mapping/test/test_frontier_exploration.py` | PASS | file: 29/29 (also ctest `test_frontier_exploration`) |
| AC2 | same file | PASS | included |
| AC3 | same file, now incl. 4 new `side_repulsion` tests (steer away both sides, offset 0 and pi; inactive when clear/front/empty; depth scaling, rear ignored for steering, inf beams ignored) | PASS | 29/29 (25 old + 4 new) |
| AC4 | `test_exploration_synthetic.py` | PASS | colcon ctest passed (11.7 s) |
| AC5 | `src/bumperbot_mujoco/test/test_auto_explore_sim.py` | PASS | `COMPLETE sim_time=736.1s wall=47.8s coverage=0.828 min_range=0.209 max_lock_s=0.0 lock_events=0`; mujoco test dir 7/7 (incl. test_sim_core) |
| AC6 | `test_frontier_explorer_node.launch.py` | PASS | colcon ctest passed (6.8 s) |
| AC7 | `src/bumperbot_bringup/test/test_launch_config.py` | PASS | 11/11 (doc/03:132 now mentions auto_explore.launch.py); colcon ctest passed |
| AC8 | `test_auto_explore_mujoco.launch.py` | PASS | `ROS_DOMAIN_ID=87 timeout 200 launch_test`: OK, `moved 0.34 m after 21.5 s`; also colcon ctest 26 s |

## Failures
None.

## Notes
- Added tests (test addition only): `test_ac3_side_repulsion_*` appended to `test_frontier_exploration.py`; `side_repulsion`/`side_clearance` were not covered before.
- Workaround used: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`, `colcon build --cmake-force-configure`. AC5 is run with direct pytest (ament_python package, not registered in ctest).
- colcon test run with lint excluded (`-E flake8|pep257|uncrustify|xmllint|copyright|lint_cmake`): mapping 3/3 functional + cppcheck, bringup 2/2 pass. Lint tests of `bumperbot_mapping` still fail (pre-existing files plus new ones); not part of any AC.
- The AC4 closed-loop run and AC5 now keep min lidar range >= 0.2 m (AC5: 0.209, zero lock events); iteration-1 deadlock is resolved.
- Teardown of the full stack still logs rclpy shutdown races (mujoco_sim_node, joy_teleop); harmless.
- Process hygiene: AC8 processes ended on their own. PID 15921 (`mujoco_sim_node` from an earlier unrelated `simulated_robot.launch.py`) is still running; I did not start it and did not kill it.
