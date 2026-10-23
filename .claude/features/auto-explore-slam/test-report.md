# Test report: iteration 4
Verdict: PASS   (11/11 AC)

| AC | Test | Result | Evidence |
|---|---|---|---|
| AC1 | test_frontier_exploration.py | PASS | 29 passed (AC1-3) |
| AC2 | test_frontier_exploration.py | PASS | same file |
| AC3 | test_frontier_exploration.py | PASS | same file |
| AC4 | test_exploration_synthetic.py | PASS | 1 passed, 10 s |
| AC5 | test_auto_explore_sim.py | PASS | 1 passed, 62 s wall |
| AC6 | test_frontier_explorer_node.launch.py | PASS | colcon ctest, domain 97 |
| AC7 | test_launch_config.py | PASS | 11 passed |
| AC8 | test_auto_explore_mujoco.launch.py | PASS | colcon ctest on ROS_DOMAIN_ID=97, 21 s |
| AC9 | test_map_compare.py (5) + test_slam_map_quality_mujoco.launch.py | PASS | E2E on domain 98: 2 tests OK, 625 s |
| AC10 | test_odometry_drift.py + test_sim_core.py | PASS | 5 + 6 passed |
| AC11 | test_world_fidelity.py (NEW) | PASS | 2 passed, 4.4 s |

## AC9 E2E metrics (ROS_DOMAIN_ID=98, ROS_LOCALHOST_ONLY=1)
- /clock=1, /scan=1 publishers. EXPLORING at sim 4.4 s, COMPLETE (natural) at sim 614 s, MAP_SAVED at 622 s. Exploration 610 s sim / 610 s wall.
- MapQuality: n_occupied=3621, extent 19.95 x 12.60 m, global_rotation 0.1 deg, shift 0.04 m, inlier_fraction 0.945,
  median_distance 0.05, dominant_residual 0.0 deg, max_tile_rotation 1.25 deg, max_tile_shift 0.10 m,
  min_tile_inlier 0.73, n_tiles 44. (iteration 3: 36 cells, FAIL.)
- Map: /tmp/claude-1000/-home-canhtran-Working-04-Investigating-udemy-ros2-map-localization-ws-bumperbot-ws/80aba5a0-ee04-509b-a599-e53f27f8f360/scratchpad/art/it4/slam_quality_map.pgm (+ .yaml)
- PNG vs reference: /tmp/claude-1000/-home-canhtran-Working-04-Investigating-udemy-ros2-map-localization-ws-bumperbot-ws/80aba5a0-ee04-509b-a599-e53f27f8f360/scratchpad/art/it4/it4_map_vs_reference.png

## AC11 metrics (src/bumperbot_mujoco/test/test_world_fidelity.py)
- Ground truth: horizontal rays at the laser_site height read from the model (0.155 m), 721 free origins on a 0.4 m grid
  (reference-free, reachable from the start, >= 0.3 m from walls), 1440 rays each, group 4 only, robot excluded, range 30 m.
- MapQuality vs map.pgm: n_occupied=3262, inlier_fraction 0.990 (>= 0.85), max_tile_rotation 1.25 deg (<= 2),
  max_tile_shift 0.05 m (<= 0.20), n_tiles 33, dominant_residual -0.25 deg.
- Hull/box check: 105 wall collision geoms, none with XY area > 4 m2 and wider than 0.6 m both ways (largest area 4.07 m2, thin).
- Asserts n_occupied >= 2000 and n_tiles >= 1 so the check cannot pass vacuously.

## Notes (not AC failures)
- colcon test --packages-select bumperbot_mujoco/mapping/bumperbot/controller: only the CMake-registered ctest items ran. The
  `ament_add_pytest_test` registrations (test_frontier_exploration, test_exploration_synthetic, test_map_compare,
  test_launch_config) are silently NOT registered: `python3-pytest_DIR-NOTFOUND` in build/bumperbot_mapping/CMakeCache.txt, ctest -N
  lists only 7 tests. I ran those pytest files directly (all pass). The bumperbot_mujoco package registers no tests with colcon at all.
  Suggested fix: make CMake find pytest (or register the tests through add_test with python3 -m pytest).
- Lint ctests (flake8, pep257, uncrustify) fail in bumperbot_mapping (style issues in map_compare.py, tests, and old C++ files).
  These are not AC tests; xmllint, cppcheck, lint_cmake pass.
- --cmake-force-configure belongs to `colcon build`, not `colcon test`; used on build.
- A pre-existing mujoco_sim_node (pid 15921, ROS_DOMAIN_ID unset = 0) was not started by me and was left running; the runs used domains 97/98 with localhost only.

## Failures
None.
