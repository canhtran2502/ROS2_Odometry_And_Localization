# Review: safety-supervisor

Base: 7ea98db..HEAD (228e80a). Scope: `git diff 7ea98db..HEAD -- src doc README.md`, plan.md, CLAUDE.md.
Verified by reading both implementations and installed twist_mux headers. The numbers come from scripts that call the
shipped `bumperbot_utils/safety_logic.py`, which matches the C++ line by line.

REVIEW: 1 high, 6 medium, 6 low

## Verdict

C++/Python parity is very good: the logic, defaults, reason strings, float32 to float64 handling, NaN/inf/0 gating,
the hysteresis state and the brake timers are the same in both. Startup, shutdown, tick-exception and stalled-clock
handling are solid. The serious defect is in the zone geometry plus the reset rule: an ordinary teleop situation
(driving alongside a wall with less than 3 cm side gap) latches an e-stop that cannot be reset. Several other findings
concern `lock_only` (the documented hardware recipe), which is much weaker than the docs suggest. Fix H1 before merge.
The medium items should be fixed or documented explicitly.

## High

### H1. A wall beside the robot within `lateral_margin` gives front AND rear CONTACT, which latches an e-stop that cannot be reset
- `src/bumperbot_utils/src/safety_logic.cpp:296-306` (py `safety_logic.py:269-275`): any point with `|y| <= half_width +
  lateral_margin` (0.155 m) and `x >= x_front` counts as front clearance `x - x_front`. A point beside the robot just
  past the front-edge line, e.g. (0.101, 0.145), gives clearance about 0, even though it is outside the swept path
  (|y| > 0.125).
- Then `safety_logic.cpp:748-750` (py 715-717) latches ESTOP when moving. `request_reset`
  `safety_logic.cpp:581-586` (py 570-573) then refuses while `contact_front`/`contact_rear` holds.
- Scenario (verified with the real logic): a wall at y = 0.145 (2 cm gap to the wheel side), robot driving forward at
  0.2 m/s by joystick or keyboard.
  - `evaluate_points` gives `front_clearance = 0`, `rear_clearance = 0`, contact on both sides.
  - The state becomes `ESTOP "contact zone (front)"`, and `request_reset()` returns
    `(False, "obstacle in contact zone (front)")`.
  - ESTOP blocks every motion, so the doc/12 advice "back away manually first" is impossible. The robot is stuck until
    the supervisor is restarted or the robot is pushed.
  - Even without the latch (stationary), both directions are STOP. The only motion left is rotation, which swings the
    corners (radius 0.16 m) into the wall.
- This breaks the plan goal "the robot is never stuck forever next to a wall". The iteration-2 footprint widening
  (corridor 0.12 m to 0.155 m) makes it more likely.
- Second path to the same state: the dock profile ends mid-insertion (`dock_approach_max_time`, `/dock/status` stale
  for more than 2 s, a TF loss) while reversing within 0.015 m of a charger. The rear zone is then re-evaluated as
  CONTACT (`safety_logic.cpp:690`), which latches, and the reset is refused because `last_dock_active_` is now false.
- Minimal fix (both languages and the case table):
  - compute `contact_front/rear` (and preferably the STOP clearance) only from points with `|y| <= half_width`, i.e.
    the swept path, and keep the `lateral_margin` band for the slowdown;
  - and/or let `request_reset` succeed with an obstacle in contact, because the filter already blocks motion toward it
    (`max_forward`/`max_reverse` = 0). Keep refusing only for the e-stop input and faults.
  - Add a case-table entry for a parallel wall.

## Medium

### M1. `lock_only` cannot keep the robot out of an occupied STOP zone: it moves, brakes, releases and repeats (hardware-only)
- `safety_logic.cpp:726` (py 699): with odom present, `moving` uses only `|v_odom| > stopped_speed`. The observed
  command is ignored, so no brake starts while the robot is still at rest.
- In `lock_only` there is no filter, so after each brake pulse (`brake_hold_time` then release) `twist_mux` forwards
  the held command again.
- Verified: obstacle at 0.03 m ahead, odom 0, observed command 0.3. The result is `ACTIVE/STOP, locked=False`. Once odom
  exceeds 0.02 the lock comes on, and it is released 0.5 s after stopping. The next command is unlocked again.
- On hardware the robot jerks toward the obstacle on every cycle. It ends either in the blind zone (M3, no stop at all)
  or in a front contact latch that cannot be reset (H1).
- The docs (README "it only drives the lock", doc/07 "brake on motion toward an occupied stop zone") do not warn about
  this.
- Fix: in `lock_only`, keep the lock while a fresh observed command or the odom points toward a STOP/CONTACT zone, or
  latch the brake until the obstacle leaves the zone. Document that `lock_only` cannot hold a robot out of a stop zone
  and recommend the filter wiring on hardware.

### M2. A supervisor crash leaves `/safety_stop` false; in `lock_only` the robot is then unprotected, and the docs say the lock stays engaged (hardware-only)
- In `/opt/ros/humble/include/twist_mux/topic_handle.hpp:102-106,227-230`, `isLocked = hasExpired() || data`, and
  `hasExpired` is always false with timeout 0.0 (`twist_mux_locks.yaml`).
- `finalize()` (`safety_supervisor.cpp:544`) runs only on SIGINT/SIGTERM. On SIGKILL, a segfault or an OOM the last
  published value is `false` (ACTIVE).
- In `filter` mode this is safe, because nothing forwards `safety/cmd_vel_in`. In `lock_only` (the hardware recipe)
  `twist_mux` drives `cmd_vel_unstamped` directly, so the robot runs with no protection and no signal.
- doc/02:155 and doc/07 say the lock "stays true when the supervisor exits", which holds only for a clean exit.
- Fix: for the `lock_only` recipe, give a separate locks file with a heartbeat timeout (for example 0.3 s; the
  supervisor republishes at 20 Hz), so a dead supervisor locks. Correct the docs to say "on a clean exit".

### M3. LiDAR blind zone and "no return" beams are treated as free space; an obstacle that moves into the blind zone turns the zone CLEAR (0.7 m/s allowed)
- `safety_logic.cpp:260-266` (py 232-236) skip beams with `r < range_min`, `inf` and NaN. The laser sits at (-0.005,
  -0.002) in `base_footprint` (URDF `laser_joint`), and `range_min` is 0.12 in both MuJoCo
  (`mujoco_sim_node.py:28`, sub-min set to inf in `sim_core.py:107`) and Gazebo.
- Front: on the centre line a clearance below 0.015 m is invisible. The whole front contact zone is blind for |y| up to
  about 0.058 m.
- Rear: clearances below 0.025 m are invisible, so `dock_stop_margin` (0.01) can never trigger for an obstacle centred
  behind the robot.
- Scenario: a robot stopped at the front STOP edge, and a person or object moves up to the bumper. The point drops
  below `range_min` and `front_clearance` becomes inf. `speed_limit(inf)` gives 0.7 m/s forward, so the protection
  inverts.
- Related gaps:
  - a scan that is 100 % inf is "valid" and CLEAR;
  - up to 50 % NaN beams (for example a blocked front half) are silently dropped, even when they are all in the
    corridor.
- doc/12:117 states only that such an obstacle "can be missed". It does not say that the limit returns to full speed,
  or that the contact latch is dead on the centre line.
- Fix (both languages):
  - treat finite returns in (0, `range_min`) as obstacles at `range_min`;
  - keep a blocking zone "sticky" after its points disappear until the robot has moved away (odom) or the hold time
    has passed;
  - count NaN/inf beams whose ray crosses the stop region as unknown or blocking;
  - add a `validate_config` warning when `stop_margin` (or `contact_margin`, `dock_stop_margin`) lies inside the
    `range_min` circle.

### M4. The dock profile reverse cap ignores the braking distance; at 0.06 m/s the robot cannot stop within `dock_stop_margin`
- `safety_logic.cpp:702-704` (py 672-675): `rev_limit = dock_max_speed` for every rear clearance above
  `dock_stop_margin`.
- The stopping distance at the cap is `0.06*0.3 + 0.06^2/(2*0.5) = 0.0216 m`, which is more than 2x the 0.01 m margin.
  The normal formula would allow only `speed_limit(0.02, 0.01) = 0.02 m/s`.
- The plan says the limit "never exceeds the braking-distance limit". The profile drops that limit, and with M3 the
  0.01 m rear stop is blind anyway. During DOCKING the robot therefore reverses at 0.06 m/s into anything within
  0.8 m of the dock pose (a person, a box) and relies on contact.
- The profile does not check that the rear obstacle is where the charger should be.
- Currently latent: the sim has no physical charger, and the hardware recipe (`lock_only`) disables the profile.
- Fix: `rev_limit = min(dock_max_speed, speed_limit(d, dock_stop_margin, cfg))`. Optionally require the rear clearance
  to agree with the expected distance to `dock_pose` within a tolerance.

### M5. The e-stop input is missed after a restart, and a supervisor restart silently clears a latched e-stop
- `safety_supervisor.cpp:177-178` / py 187: `safety/estop` is subscribed reliable + volatile (depth 10), and
  `estop_input_` starts false.
- An e-stop source that publishes on change (button node, `transient_local` publisher, or one `ros2 topic pub --once`
  as in doc/11:194) is not replayed to a restarted supervisor. The new node goes to ACTIVE although the e-stop input is
  still true.
- The ESTOP latch lives only in memory, so restarting the node (or a respawn) also clears an operator e-stop without
  the reset rules.
- Fix:
  - subscribe `safety/estop` with `transient_local` (it stays compatible with volatile publishers);
  - document that e-stop sources must be latched or periodic;
  - consider starting in ESTOP (reset required) when a parameter like `start_latched` is set for hardware.

### M6. The supervisor is now the sim default, but no closed-loop dock, mission or explore E2E ran with it, and the noise behaviour is untested
- test-report iteration 1 and 2: the 21-23 skips are the opt-in E2Es (dock, mission, SLAM, explore). Only smoke tests
  of `auto_explore`, `mission_sim` and `dock_sim` ran. `simulated_robot.launch.py:107,152` now routes every one of these
  stacks through the filter, which reduces Nav2 speed near walls, caps the dock reverse speed and adds brake pulses.
  The dock-manager interaction (STALL instead of SAFETY_TIMEOUT, insertion at 0.06 m/s, `dock_approach_max_time` 30 s
  vs `docking_timeout` 40 s) was never exercised end to end.
- Gazebo is the default `simulator` (`simulated_robot.launch.py:18`), and no supervisor test runs on Gazebo.
- Both simulators add `σ = 0.01` range noise (`mujoco_sim_node.py:30`, `bumperbot_gazebo.xacro` noise). The zones take
  the minimum over about 90 corridor beams, which biases clearance low by about 2.5σ.
  - A quick Monte Carlo with the shipped logic, a wall at a true 0.035 m (the E2E rest distance), shows
    `contact_front` in about 60 % of scans.
  - So `~/reset` after an e-stop at a wall is refused about 60 % of the time (flaky).
  - A robot still above 0.02 m/s at a true clearance of about 0.04 m has about a 13 % per-scan chance of a contact
    latch.
  - AC16 runs with noise 0.
- Fix:
  - run the dock/mission/explore E2Es with `BUMPERBOT_SAFETY_E2E`-style opt-ins before merging;
  - add a noisy-scan AC16 case;
  - require contact on 2+ adjacent beams or 2 consecutive scans before latching or refusing the reset.

## Low

### L1. The docs misstate the WARNING semantics and the brake release
- README:279, doc/07:105 and doc/12:111 say "WARNING means the speed limit is below `max_linear_speed`".
- The code (`safety_logic.cpp:344`, py 306) sets WARNING by distance only (`d < margin + slowdown_distance`). Example:
  `speed_limit(0.60) = 0.607` while the zone is CLEAR and `reason` is empty, because the brake term limits up to about
  0.75 m.
- The table's "(0.35 m)" is `slowdown_distance`, not the 0.40 m threshold.
- doc/07 says the brake is released after odom reports stopped "for at least `brake_hold_time`". The code needs
  `brake_hold_time` since the brake started plus "stopped now" (`safety_logic.cpp:742-746`).
- Fix: either make `zone_for` return WARNING when `speed_limit < max_linear_speed` (the plan's definition), or reword
  the three docs.

### L2. A reset bypasses `recovery_time`
- `request_reset` refuses only while `last_problem_` is non-empty. During the recovery window (`fault_active_` true,
  inputs healthy again) the reset is accepted and clears `fault_active_` (`safety_logic.cpp:578-592`, py 568-579).
- With `latch_faults: true`, a reset right after scans resume returns to ACTIVE immediately, without the 0.5 s of
  healthy data.
- Fix: also refuse while `fault_active_`.

### L3. `limit_command` clamps `v` but passes `w`, so the commanded path curvature changes
- `safety_logic.cpp:605-606` (py 588-589). Nav2/dock arc commands (v=0.3, w=0.5) become sharper turns or spins in
  place in WARNING/STOP. The front corners then swing toward the obstacle, and `dock_manager` line tracking runs at
  2.5x curvature under the 0.06 cap.
- In-place rotation is documented (doc/12:117), but the curvature change is not.
- Fix: scale `w` by `v_out/v_in` when `|v_in| > stopped_speed`, or document it.

### L4. The `safety` launch arg is not validated
- `simulated_robot.launch.py:107,120,152,159`: a typo (`safety:=Supervisor`) matches neither condition. The stack then
  starts without `twist_mux`, `twist_relay`, joystick or any lock, and gives no error.
- Safe (nothing moves) but confusing. Fix: add a `LogInfo`/`Shutdown` on an unknown value, or use `choices=`.

### L5. `/dock/status` is trusted without corroboration
- Any publisher of `DockStatus{state: DOCKING, frame_id, dock_pose = current pose}` turns on the rear profile anywhere
  (margin 0.01, contact latch suspended).
- Retries that leave and re-enter DOCKING re-arm the 30 s timer.
- Bounded by the 0.06 m/s cap, but see M4. Fix: optionally check `dock_pose` against the configured stations file or
  a radius from a fixed parameter.

### L6. The legacy files and earlier-feature tests changed
- `safety_stop.cpp` / `safety_stop.py` were reformatted, although the plan says "file unchanged". The change is
  whitespace only; the static test was relaxed to compare ignoring whitespace or the AST.
- `test_dock_launch_config.py` / `test_mission_launch_config.py` now diff `base..<last commit touching
  .claude/features/<old>>`. This is history-only, so they can no longer fail on new changes. If anything later commits
  into those old feature directories, the end commit moves.
- Acceptable, but it should be stated in the docs or the commit message.

## Areas with nothing significant
- C++ vs Python parity: no behavioural difference found in the logic, defaults, reason strings, float32 to float64
  scan handling, angle math, hysteresis state, brake timers or the odom-less direction logic.
- Startup and config errors: the fail-safe lock publish and exit code 1 are correct. The type checks match.
- Tick and callback exceptions, the stalled `/clock` (steady timer, steady ages), SIGINT/SIGTERM shutdown ordering,
  and a cmd before the first tick (default `Outputs.locked = true`): all fine.
- Command filtering: there is no sign change or amplification. NaN/inf commands give zero, and a stale command gives a
  zero each tick.
- Filter-mode supervisor death: the robot stops, because nothing forwards `twist_mux` and diff_drive
  `cmd_vel_timeout` is 0.5 s.
- Sim/real separation and simulator independence of the node sources: `real_robot.launch.py` is untouched, and there
  are no sim tokens or absolute paths.
- Footprint +-0.125: it does not change the x extents, so the rear clearance at the dock pose (-0.5, 0, 0) is
  unchanged. There is no physical charger in the sim worlds, so docking is not blocked geometrically (but see M6:
  untested end to end).
