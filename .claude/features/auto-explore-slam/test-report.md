# Test report: iteration 1
Verdict: FAIL   (6/8 AC)

| AC | Test | Result | Evidence |
|---|---|---|---|
| AC1 | `src/bumperbot_mapping/test/test_frontier_exploration.py` (import without rclpy, detect_frontiers vs brute force, clustering, min size) | PASS | 25/25 tests of the file pass (also via `colcon test`, ctest `test_frontier_exploration`) |
| AC2 | same file (costmap inflation exact, A* doorway/no corner cutting/closed room None, start snap, select_goal blacklist/unreachable) | PASS | included in the 25/25 |
| AC3 | same file (PurePursuit limits/rotate/done, scan_safety offset pi, rear obstacle, escape_dir, CompletionMonitor) | PASS | included in the 25/25 |
| AC4 | `src/bumperbot_mapping/test/test_exploration_synthetic.py` | PASS | `state=COMPLETE sim_time=76.1s coverage=1.000 collided=False min_range=0.188 final_cmd=(0.0, 0.0)` |
| AC5 | `src/bumperbot_mujoco/test/test_auto_explore_sim.py` | FAIL | COMPLETE at sim 902 s, wall 55 s, coverage 0.826, min_range 0.157, but emulated safety lock active 220.5 s in a row (limit 5 s), 23 lock events |
| AC6 | `src/bumperbot_mapping/test/test_frontier_explorer_node.launch.py` | PASS | launch_test OK (6-7 s); EXPLORING, non-zero twist, path, COMPLETE, zero twist, save request with `map_url == map_save_path`, MAP_SAVED; also via ctest |
| AC7 | `src/bumperbot_bringup/test/test_launch_config.py` | FAIL | 10/11 pass; `doc/03_ros2_nodes_topics_services.md` has no `auto_explore.launch.py` |
| AC8 | `src/bumperbot_bringup/test/test_auto_explore_mujoco.launch.py` | PASS | map received, EXPLORING, moved 0.31-0.32 m after about 21.5 s (ROS_DOMAIN_ID isolated); ctest 24-28 s |

## Failures

### AC5: safety-lock deadlock in the MuJoCo small_house run
- Command: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/bumperbot_mapping:src/bumperbot_mujoco python3 -m pytest src/bumperbot_mujoco/test/test_auto_explore_sim.py -q -s`
- Output (trimmed): `AssertionError: safety lock active for 220.5 s in a row (deadlock)`.
  `AC5: state=COMPLETE sim_time=902.1s wall=54.2s coverage=0.826 min_range=0.157 max_lock_s=220.5 lock_events=23`.
  Lock starts: t=54 (min 0.197 m, base angle -1.66), t=114-117 (five times at (2.69,-0.88), angle +1.5), t=194-204 (nine times at (-3.7,-1.3), angle +1.3..+1.9), t=487 at (3.43, 2.82, yaw -1.72) with 0.191 m at base angle +1.09..+1.34, then locked until 708 s, and lock again from 708.9 s until the end.
- This is the exact risk named in the plan and by the implementer: the lidar min of about 0.19-0.20 m is below or at safety_stop's `<= 0.2` lock threshold. Coverage (0.826 >= 0.80), COMPLETE (902 s < 1800 s), wall time (55 s < 180 s) and min range (0.157 >= 0.15) pass; only the no-deadlock criterion fails.
- Root cause: the obstacles that trigger the lock are on the robot's SIDE (base angle about +-1.1 to +-1.9 rad), outside the front sector (+-0.6 rad) that `scan_safety` uses for blocking, so the explorer's own safety layer does not react. `src/bumperbot_mapping/bumperbot_mapping/frontier_exploration.py:406-412` (blocked only from `front_min`) and `:663-692` (recovery only when `blocked`). The path planner keeps 0.30 m inflation from the map, but the robot start/snap, pure-pursuit corner cutting and the rotate-in-place turn bring the side to <= 0.2 m. Once the lock is active the explorer's commands (including the escape) are dropped by twist_mux, so the robot never leaves the zone.
- Suggested fix: make the reactive layer use `min_all` (all directions): slow down and steer away when any beam is < about 0.28 m (above the 0.2 m lock), and add a sideways escape (rotate/reverse away from the nearest side obstacle) before the lock engages. Increase `inflation_radius` or add a path-clearance cost. Make `safety_stop` danger distance configurable per launch only if needed (plan risk section). Do not weaken safety_stop in general.

### AC7: doc/03 does not mention `auto_explore.launch.py`
- Command: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/bumperbot_bringup/test/test_launch_config.py -q`
- Output: `AssertionError: doc/03_ros2_nodes_topics_services.md lacks auto_explore.launch.py`. `frontier_explorer` is present (doc/03:123), the launch file name is not.
- Passing sub-checks: `--show-args` of auto_explore (5 args listed), real_robot and simulated_robot exit 0; `simulated_robot` defaults unchanged (use_rviz/viewer true); auto_explore description contains the `frontier_explorer.py` node of `bumperbot_mapping`; no `/home` or `/opt` literals in any launch file; twist_mux explore 10 < keyboard 90 < joystick 99 and safety_stop lock 255; `git diff main --stat` empty for `real_robot.launch.py` and `src/bumperbot_firmware`; README, doc/08 and doc/10 mention both names.
- Suggested fix: add a line with `ros2 launch bumperbot_bringup auto_explore.launch.py ...` to `doc/03_ros2_nodes_topics_services.md` (for example in the `frontier_explorer` entry at line 123).

## Notes and observations (not counted as AC failures)
- Environment: `python3 -m pytest` fails here with `ModuleNotFoundError: No module named '_pytest.scope'` because of `~/.local/.../anyio/pytest_plugin.py` (user-site anyio is newer than the system pytest 6.2.5). It also makes CMake's `ament_has_pytest` fail, so the pytest tests are silently not registered under `colcon build/test`. The plan's literal commands do not run until this is worked around: `export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` (or `-p no:anyio`) and `colcon build --cmake-force-configure`. With the workaround colcon registers and passes `test_frontier_exploration`, `test_exploration_synthetic`, `test_frontier_explorer_node`, `test_auto_explore_mujoco`; only `test_launch_config` fails (the doc/03 item above).
- AC5 is not registered in colcon (bumperbot_mujoco is ament_python), so it is run directly with pytest. `test_sim_core.py` regression: 6/6 pass.
- AC6 deviation: the test sets `initial_spin: False`. With a static TF the 360 degree spin never completes (it ends after the 40 s `SPIN_TIMEOUT`), so with the default `initial_spin: true` the path stays empty and the run fails (verified: `paths=[0, 0, 0]` after 15 s). This is inherent to the AC as written (static TF), not a product bug.
- AC8 was first run while an unrelated earlier stack (pids 15913/15921, `simulated_robot.launch.py simulator:=mujoco use_slam:=true`, not started by me) was running in the same ROS domain; the result was repeated with `ROS_DOMAIN_ID=77`/`78` and passed. I did not kill that stack.
- On SIGINT teardown of the full stack, `frontier_explorer.py` and `mujoco_sim_node` exit with code 1 (rclpy `RuntimeError: Unable to convert call argument to Python object` in `_take_subscription`, a shutdown race). `joy_teleop` also dies (no joystick). Harmless for the smoke test, which asserts no exit codes.
- Lint tests (flake8/pep257/uncrustify/xmllint) of `bumperbot_mapping` fail, mostly on pre-existing files; new files add a few pep257 findings (for example `frontier_exploration.py:497,611`). They are not part of any AC.
- AC4 synthetic run: min lidar range 0.188 m (below safety_stop's 0.2 m), consistent with the AC5 problem.
