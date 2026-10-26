# Review: auto-docking (base 26a8a37..HEAD)

Scope: `git diff 26a8a37..HEAD -- src doc README.md`, checked against plan.md and CLAUDE.md.
Every finding below was traced in code. The tests pass, so this covers what they do not exercise.

## Findings (most severe first)

### M1. A cancel or shutdown while the Nav2 goal request is in flight leaves an orphan Nav2 goal running (medium)
- `src/bumperbot_navigation/bumperbot_navigation/dock_manager.py:828-835`, `src/bumperbot_navigation/src/dock_manager.cpp:970-975`
- Scenario: a DOCK goal starts in NAVIGATING_TO_STAGING and `send_goal_async` / `async_send_goal` is sent. A DockRobot cancel
  (or SIGINT/SIGTERM, or a clock stall) arrives before Nav2 answers. `check_interrupts()` returns true and `navigate()`
  returns `None` / `-1`. `nav_handle` was never set, so neither `navigate()` nor `finish_goal()` -> `cancel_nav_goal()`
  cancels anything. Nav2 then accepts the goal and drives the robot to staging, while the dock goal has already ended
  CANCELED/ERROR "robot stopped". In the shutdown case the process exits and nobody owns the Nav2 goal any more. Both
  languages behave the same way. The race window is the Nav2 goal-response latency, which AC10d (cancel within 50 ms)
  can hit against a real bt_navigator.
- Fix: when interrupted during the response wait, keep the future and wait a bounded time (e.g. up to 1 s) for the
  answer. If the goal was accepted, cancel it. Alternatively, attach a done-callback that cancels any handle accepted
  after the interrupt. Mirror the fix in both languages.

### M2. No mission/dock interlock after acceptance: a mission sent during docking or charging fights `dock_manager` for Nav2 and pulls the robot off the charger (medium)
- `src/bumperbot_navigation/bumperbot_navigation/dock_manager.py:496` / `dock_manager.cpp:545` (mission state is checked
  only in `goal_callback`); `mission_manager.py:127-138` (unchanged, accepts missions at any dock state);
  `battery_logic.py:259-273` (DOCKING / CHARGING never re-cancel)
- Scenario A: battery_monitor sends DOCK, and dock_manager is in NAVIGATING_TO_STAGING. A user sends
  `ExecuteMission`. mission_manager sends its own NavigateToPose, Nav2 preempts dock_manager's goal (aborted), and
  dock_manager counts a failed Nav2 attempt and re-sends, which preempts the mission. The two clients ping-pong until one
  runs out of retries.
- Scenario B: the mission arrives during ALIGN/INSERT. `dock_vel` (30) masks `nav_vel` (20) while docking runs. When the
  goal ends DOCKED and `dock_vel` times out (0.5 s), Nav2 drives the robot straight out of the charger with a low battery.
  battery_monitor then goes CHARGING -> MONITORING -> cancels the mission again.
- The plan lists only the "mission sent while DOCKED" case as an open question. The case during an active dock is not
  covered and is not documented.
- Fix (minimal): in `dock_manager`, abort the running goal with a clear message (or cancel Nav2 and stop) when
  `mission_active` turns true during a goal, in both languages. In `battery_monitor`, re-send the cancel-all when a
  mission becomes active while it is in DOCKING/CHARGING. Document the remaining "mission while charging" behavior in
  doc/10.

### M3. battery_monitor can hang forever in DOCKING/UNDOCKING when dock_manager dies or never answers (medium)
- `src/bumperbot_navigation/bumperbot_navigation/battery_logic.py:259-265, 274-280`; `battery_monitor.py:201, 216`
- Scenario: `server_is_ready()` is true and DOCK is sent. dock_manager then crashes or is restarted, either before the
  goal response or during the goal. The rclpy futures for the goal response and the result never complete. `_goal_active`
  stays true, and `dock_request_timeout` is only evaluated when `not _goal_active`. So battery_monitor stays in DOCKING
  indefinitely. It never retries, never reaches ERROR, and does not recover after dock_manager restarts. The battery keeps
  draining with no automatic dock.
- Fix: add a watchdog in the logic. With a goal active for longer than an upper bound (e.g. `dock_request_timeout` plus
  the worst-case goal time, or a new `dock_goal_timeout`), or with the server not ready for N s while a goal is active,
  treat it as RESULT_FAILED and drop the handle. Add a `battery_cases.yaml` case for it.

### M4. The low-battery mission cancel is one-shot and can end in ERROR while the mission keeps running (medium)
- `src/bumperbot_navigation/bumperbot_navigation/battery_monitor.py:184-190`; `battery_logic.py:250-258`;
  `dock_manager.py:346-347` / `dock_manager.cpp:136-140`
- Scenario 1: `execute_mission/_action/cancel_goal` is not ready, or the service response is lost. `cancel_missions()`
  logs a warning and returns. ACTION_CANCEL_MISSION is emitted only on entry to CANCELING_MISSION, so it is never
  retried. The CancelGoal response (`return_code`, `goals_canceling`) is never checked either. The mission keeps running
  on a low battery for `mission_cancel_timeout` (180 s). Then every DOCK is rejected ("mission active") for
  `dock_request_timeout` (120 s), and the monitor ends in ERROR. Nothing protects the battery after that.
- Scenario 2: mission_manager crashes while NAVIGATING. `/mission/status` is transient_local, so the last
  NAVIGATING sample stays latched in both battery_monitor and dock_manager. `mission_active` is then true forever, every
  DOCK is rejected, and the result is the same ERROR even though no mission is running.
- Fix: re-send the cancel periodically (e.g. every `dock_retry_delay`) while in CANCELING_MISSION, and check the
  response. For a stale status, treat `/mission/status` as inactive when `execute_mission` has no server
  (`server_is_ready()` false) or when the status is older than a timeout. Apply this in battery_monitor and in both
  dock_manager versions.

### M5. Detection gating is too permissive for a real detector: stamp 0 skips the age check, no yaw gate, and a walking gate stays active during insertion (medium, hardware-only)
- `dock_manager.py:380-384, 754` / `dock_manager.cpp:377-383, 875`; `dock_logic.py:303-313` / `dock_logic.cpp:239-253`
- A detection with `header.stamp == 0` is never age-checked and is transformed with the latest TF. The jump gate is
  relative to the current EMA estimate, not to the map prior. Only position is gated: a 180 deg yaw flip (a common
  AprilTag pose ambiguity) is accepted and moves the estimate yaw by alpha*pi, about 0.94 rad, in one message.
  `accepting` stays true through ALIGN and INSERT.
- Scenario: a real detector re-publishes its last pose in a robot-attached frame (`camera_link`) with stamp 0, or
  re-stamps it with "now". Each sample puts the "charger" at a fixed offset behind the current robot pose, so the estimate
  follows the robot. In INSERT `x` never reaches 0 and `|x| > slow_down_distance` holds, so the robot backs straight at
  `max_linear_speed` until `docking_timeout` (40 s, up to about 6 m). The only backstop is the safety_stop lock, and
  `real_robot.launch.py` does not start `safety_stop`. A yaw flip, in contrast, only makes the attempt fail MISALIGNED,
  which is safe but costs retries.
- Fix: reject stamp 0 (or require `detection_max_age` on a real stamp). Bound the total correction relative to the
  map-derived prior (`|estimate - prior| <= detection_max_jump`). Gate yaw (e.g. reject `|dyaw| > max_insert_yaw_error`).
  Optionally freeze the estimate during the last `slow_down_distance`. Add parity cases to `dock_cases.yaml`.

### L1. A constant odom TF lag in (tf_stale_timeout, tf_timeout] makes a control phase loop forever and bypasses its timers (low, hardware-only)
- `dock_manager.py:697-720, 948-953, 1001-1005` / `dock_manager.cpp:795-824, 1133-1139, 1205-1210`
- Scenario: the odom TF stamp lags node time by 0.5-1.0 s, for example because the dock_manager PC and the robot clocks
  are offset without NTP. Each `lookup` succeeds (so `tf_fail_since` resets), `age > tf_stale_timeout` returns None, and
  `age > tf_timeout` is never true. `control_phase`/`run_verify` then `continue` before `timer.expired()`, so
  align/docking/stall/safety timeouts never fire. The goal publishes zero forever, stays active, and blocks battery_monitor
  (see M3). An offset of more than 1 s makes every goal ERROR immediately.
- Fix: evaluate the phase timer on the stale path too, or escalate to ERROR once the stale condition has lasted
  `tf_timeout` of wall time.

### L2. `dock_marker_sim` (new code) exits 1 with an `rcl_shutdown already called` traceback on launch SIGINT (low)
- `src/bumperbot_mujoco/bumperbot_mujoco/dock_marker_sim.py:140-148`
- It uses the default rclpy signal handler, which shuts the context down on SIGINT. Then `rclpy.shutdown()` at line 148
  raises RCLError, and `ExternalShutdownException` from `spin` is not caught either. This is the traceback the tester saw.
- Fix (same pattern as `battery_sim.py:138-144` / `mission_manager.py:520-536`): catch
  `(KeyboardInterrupt, ExternalShutdownException)` around `spin`, wrap `destroy_node()` and
  `if rclpy.ok(): rclpy.shutdown()` in `try/except RCLError`. `mujoco_sim_node.py:153-161` (pre-existing, but touched
  in this diff) has the same pattern and can get the same 4-line fix.

### L3. A cancel that arrives after a successful dock reports CANCELED for a robot that is docked (low)
- `dock_manager.py:597-598` / `dock_manager.cpp:664-665`
- Scenario: verify succeeds (outcome DOCKED, `docked=true`), then a cancel arrives before `finish_goal`. The goal ends
  CANCELED with `result.docked == true`, and `/dock/status` says DOCKED. battery_monitor maps any non-SUCCEEDED status to
  RESULT_FAILED and counts an attempt. The plan says "a cancel after the terminal decision is ignored", and rcl_action
  allows CANCELING -> SUCCEEDED, so the code comment "can only end CANCELED (or ABORTED)" is inaccurate.
- Fix: when the tracker already finished with DOCKED/UNDOCKED/ALREADY_DOCKED, call `succeed()` even if a cancel is
  pending. Change both languages.

### L4. dock_manager misses a `safety_stop` lock that was active before it subscribed (low)
- `dock_manager.py:173-174` / `dock_manager.cpp:125-127`; `bumperbot_utils/safety_stop.py:81-100` (publishes only on change, volatile)
- Scenario: dock_manager starts or restarts while the lock is already True. `safety_locked` stays False. twist_mux
  still blocks the motion, so this is safe, but the attempt fails as STALL after `stall_timeout` instead of pausing up to
  `safety_stop_timeout`. Logs and doc/12 then point at the wrong cause.
- Fix: document it, or have safety_stop publish periodically or latch (that is a change outside this feature).

### L5. A manual dock that runs longer than `dock_request_timeout` drives battery_monitor to ERROR (low)
- `battery_logic.py:205-207, 261-262`
- Scenario: a user DOCK is in progress (a long Nav2 leg plus retries, which can exceed 120 s) when the battery crosses
  `low_threshold`. Every monitor DOCK is rejected ("another dock goal is active") and does not count as an attempt. After
  120 s the monitor goes to ERROR and needs `~/reset`, even if the user's dock then succeeds.
- Fix: in DOCKING, when `/dock/status` shows an active dock state, extend `_request_start`, or treat `docked=true` as
  RESULT_ALREADY_DOCKED.

### L6. Docs accuracy (low)
- `doc/03_ros2_nodes_topics_services.md:162` says battery_monitor has the action clients `dock_robot` and
  `execute_mission`. The code has only the DockRobot action client plus a `CancelGoal` service client on
  `execute_mission/_action/cancel_goal`.
- README / doc/10 / doc/12 describe the safety_stop lock as overriding docking "on the real robot", but
  `real_robot.launch.py` starts no `safety_stop` node. Add a sentence that it has to be started there before docking is
  used on hardware.

## Areas with nothing significant
- C++ vs Python parity: `dock_logic.*` match line by line (geometry, Aligner hysteresis, insert/undock, filter,
  PhaseTimer, DockTracker transitions, outcome->state mapping). The nodes match in phase timing, retries, MIN_DETECTIONS,
  detection validation, status/feedback content, startup validation order, exit codes and finish_goal mapping. YAML
  parsing goes through the shared `load_stations`. I found no behavioral difference other than the ones shared by both
  languages above.
- Velocity bounds: every command goes through the clamping `publish_cmd`, and every phase return, cancel, abort and
  shutdown path publishes zero (3x in `finish_goal`). The docked flag is cleared by `next_step()` on motion and by the
  idle docked watch (0.10 m in odom), so the earlier-feature bug is not repeated. battery_monitor leaves CHARGING when
  `docked` drops.
- Sim/real separation: no mujoco/gazebo/ground_truth/clock tokens in navigation nodes, configs or launch.
  `real_robot.launch.py`, `simulated_robot.launch.py` and `mission_sim.launch.py` are unchanged. `battery_sim` and
  `dock_marker_sim` are started only by `dock_sim.launch.py`. No hardcoded absolute paths.
- twist_mux: docking at 30 sits between nav 20 and keyboard 90, and `twist_mux_locks.yaml` is unchanged.

## Verdict
No high-severity defect. The core docking controller and the C++/Python parity are solid. The remaining risk is in
the integration and failure paths: an orphan Nav2 goal after a cancel/shutdown race (M1), no mission/dock interlock after
acceptance (M2), battery_monitor having no watchdog for a dead dock_manager or a failed one-shot cancel (M3, M4), and
detection gating that a real detector could exploit (M5, hardware-only). I recommend fixing M1-M4 before merge. M5 and
the hardware-only lows can be deferred to the real-detector integration.
