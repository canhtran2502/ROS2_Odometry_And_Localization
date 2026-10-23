# Test report: iteration 3
Verdict: FAIL   (8/10 AC)

| AC | Test | Result | Evidence |
|---|---|---|---|
| AC1 | test_frontier_exploration.py | PASS | 29 tests (AC1-3) pass |
| AC2 | test_frontier_exploration.py | PASS | same file |
| AC3 | test_frontier_exploration.py | PASS | same file |
| AC4 | test_exploration_synthetic.py | PASS | 1/1 |
| AC5 | test_auto_explore_sim.py | PASS | passes with the modified robot.xml |
| AC6 | test_frontier_explorer_node.launch.py | PASS | 2/2 (domain 88) |
| AC7 | test_launch_config.py | PASS | 11/11 |
| AC8 | test_auto_explore_mujoco.launch.py | FAIL | robot never moves: moved=0.000 m in 120 s (statuses WAITING_FOR_MAP, EXPLORING; map received) |
| AC9 | test_map_compare.py (5 tests) PASS; test_slam_map_quality_mujoco.launch.py FAIL | FAIL | E2E: explorer ended in 274 s sim with 36 occupied cells, see below |
| AC10 | test_odometry_drift.py + test_sim_core.py | PASS | square 0.68 deg / 0.017 m; 4 turns +-0.8: 2.73 deg each; straight 3 m: 0.0018 m; casters dim == 1; 11/11 with sim_core |

New tests: src/bumperbot_mapping/test/test_map_compare.py (+ test/data/warped_small_house.{pgm,yaml}),
src/bumperbot_mujoco/test/test_odometry_drift.py, src/bumperbot_bringup/test/test_slam_map_quality_mujoco.launch.py.
Helper results (map_compare, each compare_maps about 1.3 s):
- perturbed reference (7 deg, shift, 30 % dropout, 3 cm noise): PASS, rotation 7.0 deg, inlier 1.00, tile rot 0.5.
- synthetic seam: fails, max_tile_rotation 7.0 deg (> 4), inlier 0.67, tile shift 0.67.
- warped fixture: fails on inlier 0.62, tile rot 16.0, tile shift 0.57, min tile inlier 0.27.
map_compare.py itself looks correct (no FAIL found against it).

AC9 E2E run (ROS_DOMAIN_ID=87, ROS_LOCALHOST_ONLY=1, BUMPERBOT_SLAM_E2E=1, timeout 2000):
- /clock publishers 1, /scan publishers 1 (isolation OK).
- Status: EXPLORING at sim 4.5 s, COMPLETE at sim 278 s (natural, "no reachable frontier left"), MAP_SAVED at 286 s.
  Exploration time 274 s sim / 274 s wall. nav2 map_saver "service not available", fallback slam_toolbox/save_map worked.
- MapQuality: n_occupied=36, extent 4.85 x 3.3 m, inlier_fraction 0.94 (meaningless, 36 cells), global_rotation -1.1 deg,
  dominant_residual 3.75 deg, n_tiles 0 (so max tile rotation / shift = 0 are vacuous), min_tile_inlier 0.0.
- Failed checks: n_occupied 36 < 2000, extent < 15 x 9, dominant_residual 3.75 > 2.0, n_tiles 0 < 1, min_tile_inlier 0.00 < 0.50.
- Map (saved): /tmp/claude-1000/-home-canhtran-Working-04-Investigating-udemy-ros2-map-localization-ws-bumperbot-ws/80aba5a0-ee04-509b-a599-e53f27f8f360/scratchpad/art/slam_quality_map.yaml / .pgm
- PNG vs reference: /tmp/claude-1000/-home-canhtran-Working-04-Investigating-udemy-ros2-map-localization-ws-bumperbot-ws/80aba5a0-ee04-509b-a599-e53f27f8f360/scratchpad/art/slam_quality_map_vs_reference.png
  The map shows only a free-space blob of radius about 1.3 m around the origin with almost no wall cells.

## Failures
### AC8 / AC9: robot does not move in the full ROS stack
- AC8: `ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 timeout 180 launch_test src/bumperbot_bringup/test/test_auto_explore_mujoco.launch.py`
  -> `AssertionError: within 120.0s: map_received=True, statuses=['WAITING_FOR_MAP','EXPLORING'], first_tf=(8e-10, 5e-12), moved=0.000 m`.
  Also failed under colcon test (same result).
- Diagnostics (domain 89): explore_vel and /bumperbot_controller/cmd_vel carry non-zero twists (v 0.145, w 0.18-0.5), but
  /joint_states wheel positions stay at about 5e-8 and odom stays at 0; no message could be echoed on /mujoco/joint_commands
  (also with best_effort). So commands are produced but the wheels do not turn in the ros2_control -> mujoco_sim_node path.
- Headless BumperbotSim with the same small_house.xml and robot.xml moves fine (AC5, AC10, sim_core pass), so the MJCF model is not the cause.
- Not confirmed: whether this is a regression from iteration 3 or an environment effect. A stale pre-existing mujoco_sim_node
  (pid 15921, default domain) exists, which I did not start and did not kill; all my runs used isolated domains (87/88/89,
  localhost only) and AC9 confirmed exactly one /clock and one /scan publisher.
  Suspects: bumperbot_controller / topic_based_ros2_control hardware write path (not publishing /mujoco/joint_commands), or the
  controller_manager clock handling with use_sim_time. The explorer's own early COMPLETE is a consequence: the map has only a small free blob, no frontiers remain.
- Suggested next step: run `ros2 topic hz /mujoco/joint_commands /clock` on the isolated domain during auto_explore, and
  re-run AC8 on the iteration-2 commit (3b28c20) in the same environment to see if it is a regression.

### Other notes
- colcon test also runs ament lint (pep257, flake8, uncrustify, xmllint): many pre-existing style failures (and new E501/D205 in map_compare.py and test_map_compare.py); not counted as AC.
- test_map_compare was only registered in ctest after `--cmake-force-configure` (the test file was new); it passes via pytest (5/5).
- test_slam_map_quality_mujoco is skipped by ctest without BUMPERBOT_SLAM_E2E=1 (by design); it was run separately with the env set.
