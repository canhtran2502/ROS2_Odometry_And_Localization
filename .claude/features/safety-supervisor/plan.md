# AGV safety supervisor (direction-aware speed limiting, e-stop latch, sensor watchdog)

## Goal

Add a `safety_supervisor` node (C++ and Python versions with the same behavior, parameters and
interfaces) to `bumperbot_utils`. It protects the robot independent of Nav2, using only `/scan`,
odometry, TF and the velocity command stream:

1. **Direction-aware zones shaped around the footprint.** Each LiDAR point is transformed into
   `base_footprint`. Points inside the front corridor (ahead of the footprint front edge, footprint
   half width + `lateral_margin`) limit only forward motion. Points inside the rear corridor limit
   only reverse motion. Per direction:
   - **stop zone**: clearance `d <= stop_margin` is a hard stop in that direction;
   - **warning zone**: `stop_margin < d < stop_margin + slowdown_distance` gives a speed limit that
     never increases as `d` decreases (progressive slowdown). The limit also never exceeds the
     braking-distance limit (`response_time`, `max_decel`);
   - **contact zone**: `d <= contact_margin`. While the robot moves toward it, this latches the
     e-stop.
   An obstacle behind the robot never limits forward motion, and an obstacle ahead never limits
   reverse motion.
2. **Two enforcement channels:**
   - **Speed filter** (default, `enforcement: filter`). In the simulation stack the `twist_mux`
     output is remapped to `safety/cmd_vel_in`. The supervisor publishes the limited command on
     `bumperbot_controller/cmd_vel_unstamped`, which `twist_relay` consumes. This is the only way
     to slow down every source (joystick, keyboard, Nav2, docking, exploration) without Nav2.
   - **The existing `twist_mux` `safety_stop` lock (priority 255, file unchanged).** The supervisor
     is its only publisher in the default sim stack. The lock is `true` in INITIALIZING, FAULT,
     ESTOP and SHUTDOWN, and during a **brake pulse**: an obstacle enters the stop zone of the
     direction of travel while the robot moves toward it. The pulse is released once odometry
     reports the robot stopped for `brake_hold_time`. After that the filter keeps blocking motion
     toward the obstacle and lets motion away from it pass, so the robot can always back out. The
     lock is republished on every tick (20 Hz), not only on change.
3. **Latched emergency stop.** It is triggered by `~/trigger_estop` (`std_srvs/Trigger`), by the
   `safety/estop` topic (`std_msgs/Bool`, true) or by a contact-zone violation. It stays latched
   until `~/reset` (`std_srvs/Trigger`) succeeds. The reset is refused while:
   - the `safety/estop` input is still true;
   - any fault condition exists (stale or invalid scan, odom or TF);
   - an obstacle is inside the contact zone (footprint + `contact_margin`). The rear is exempt when
     the dock gate conditions hold.
   An obstacle in a stop zone (not contact) does not block the reset. After the reset the filter
   still blocks motion toward that obstacle. So a reset never enables motion into an obstacle, and
   the robot is never stuck forever next to a wall.
4. **Watchdog, fail-safe.**
   - No valid scan yet, TF `robot_frame <- scan frame` missing, or (with `use_odom`) no odometry
     yet gives INITIALIZING: locked, zero output, the reason names the missing input.
   - Scan older than `scan_timeout`, odom older than `odom_timeout`, an invalid scan (empty, NaN
     ratio > `max_invalid_beam_ratio`, `angle_increment` <= 0, frame changed) or an exception
     inside a tick gives FAULT: locked, zero output. It auto-recovers after `recovery_time` of
     healthy data, or latches the e-stop when `latch_faults: true`.
   - A stale or non-finite `cmd_vel_in` gives a zero output. This is not a fault and does not lock.
   - Ages are measured with the steady clock, and the protection loop runs on a wall/steady timer,
     so it keeps running when `/clock` stalls.
5. **`/safety/status`** (`bumperbot_msgs/msg/SafetyStatus`): state, active / front / rear zone,
   nearest obstacle distance and bearing, clearances, allowed speeds, lock, latch, reason and input
   ages. Published at `status_rate` and on every state or zone change.
6. **Bounded dock-approach profile.** It is active only when all of these hold:
   - `dock_approach_enabled`;
   - `enforcement == filter`;
   - `/dock/status` (`DockStatus`) is fresh (< `dock_status_timeout`) and in state `DOCKING`;
   - the robot is within `dock_approach_radius` of `DockStatus.dock_pose`, checked through TF
     `DockStatus.frame_id -> robot_frame`;
   - less than `dock_approach_max_time` has passed in this DOCKING episode.
   It changes **only the rear direction**: the rear stop margin becomes `dock_stop_margin`, the rear
   warning ramp is replaced by a hard reverse cap `dock_max_speed`, and the rear contact latch is
   suspended. Front, lock, e-stop and watchdog are unchanged. When any condition fails, normal
   protection returns in the same tick. The profile re-arms only after `/dock/status` leaves
   DOCKING, so no flag survives across episodes. The lock is never bypassed: a brake pulse or the
   e-stop still blocks docking.
7. **Integration without regressions.**
   - `simulated_robot.launch.py` starts the supervisor (C++ by default) instead of the legacy
     `safety_stop` node. A new arg `safety:=legacy` restores the old node and wiring.
   - `auto_explore`, `mission_sim` and `dock_sim` include `simulated_robot.launch.py`, so they get
     the supervisor without being modified.
   - `dock_manager` keeps reading `/safety_stop` as before.
   - The legacy `safety_stop` node stays in `bumperbot_utils`, unchanged.

### Real-robot decision (called out explicitly)

**`real_robot.launch.py` is NOT modified and does not start the supervisor.** Reasons:
- No LiDAR driver is started by `real_robot.launch.py` and none exists in the repo. A fail-safe
  supervisor would stay in INITIALIZING with the 255 lock engaged forever, and that would also
  disable the joystick. That is a silent, breaking behavior change on hardware.
- The filter channel needs the `twist_mux` output rewired, which is a real-robot wiring decision for
  the user.

What is provided instead:
- a simulator-independent `bumperbot_utils/launch/safety_supervisor.launch.py`;
- `enforcement:=lock_only` for hardware. In this mode the node does not subscribe to `cmd_vel_in`
  and never publishes velocity. It only drives the lock (brake on measured or observed motion toward
  an occupied stop zone, plus e-stop and watchdog). Slowdown and the dock profile are unavailable,
  and the status reports this;
- `joystick_teleop.launch.py` gets an optional `cmd_vel_out` arg whose default stays the current
  topic, so the real-robot path is byte-for-byte equivalent.

The docs state:
- `real_robot.launch.py` does not start `safety_stop` or `safety_supervisor`;
- on hardware the user starts a LiDAR driver plus `safety_supervisor.launch.py enforcement:=lock_only`
  separately;
- without that, nothing provides the lock (the same as today). The existing sentences required by
  `test_dock_docs.py` stay true and present.

### Virtual keepout / speed-limit polygons: decision

**Dropped (see Risks).** Map-frame polygons make the safety layer depend on AMCL. A safety layer
must stay valid when localization is lost, and Nav2 already offers keepout and speed costmap
filters. It would also add a third YAML parser that needs parity. The robot footprint is a polygon
parameter, so polygon validation exists and the extension point is ready.

### Package placement

- Nodes and the pure `safety_logic` libraries go into the existing `bumperbot_utils` (home of
  `safety_stop`).
- `SafetyStatus.msg` goes into `bumperbot_msgs`.
- The launch file and config go into `bumperbot_utils/launch` and `bumperbot_utils/config` (new
  install dirs).
- Sim wiring goes into `bumperbot_bringup/launch/simulated_robot.launch.py`.
- The headless MuJoCo closed-loop test goes into `bumperbot_mujoco/test`.
- No simulator code is added.

### C++ / Python parity strategy

- Pure logic without ROS in both languages, with the same snake_case API:
  `bumperbot_utils/safety_logic.py`, `include/bumperbot_utils/safety_logic.hpp` and
  `src/safety_logic.cpp`.
- One shared case table, `test/data/safety_cases.yaml`, is run by pytest and gtest (yaml-cpp, test
  only).
- Configuration is **ROS parameters only** (no custom YAML). Both languages get the values from the
  same rcl parameter parser (`--params-file`), so YAML handling of duplicate keys, quotes, null and
  exponents is identical by construction. Semantic validation is one shared function
  `validate_config`, in the case table. AC3 checks at node level that both executables give the
  same outcome for each malformed file.
- Node tests are parametrized over `impl in [cpp, py]`. The C++ process is proven by its startup
  log line `safety_supervisor started (C++)` and the executable path.

## Architecture

### Nodes

| Package | Language | Executable / node name | Role |
|---|---|---|---|
| `bumperbot_utils` | C++ (new) | `safety_supervisor` / `safety_supervisor` | Zones, filter, lock, e-stop, watchdog, dock profile, status |
| `bumperbot_utils` | Python (new) | `safety_supervisor.py` / `safety_supervisor` | Same as C++ (selected by `use_python`) |
| `bumperbot_utils` | C++ lib (new, no rclcpp) | `safety_logic` | Geometry, limits, zones, `SafetyCore` state machine, `DockGate` |
| `bumperbot_utils` | Python module (new, no rclpy) | `bumperbot_utils/safety_logic.py` | Same as C++ |
| `bumperbot_utils` | existing | `safety_stop` / `safety_stop.py` | Unchanged legacy node; started only with `safety:=legacy` |
| `twist_mux` | existing | `twist_mux` | Unchanged configs; output remapped to `safety/cmd_vel_in` in the sim |
| `bumperbot_controller` | existing | `twist_relay` | Unchanged; consumes the supervisor output |
| `bumperbot_navigation` | existing | `dock_manager` | Unchanged; publishes `/dock/status`, reads `/safety_stop` |

### Interface (new, `bumperbot_msgs/msg/SafetyStatus.msg`)

```
string STATE_INITIALIZING=INITIALIZING     # waiting for scan / TF / odom: locked
string STATE_ACTIVE=ACTIVE                 # protection running
string STATE_DOCK_APPROACH=DOCK_APPROACH   # ACTIVE with the bounded rear dock profile
string STATE_FAULT=FAULT                   # stale/invalid input or internal error: locked
string STATE_ESTOP=ESTOP                   # latched until ~/reset succeeds: locked
string STATE_SHUTDOWN=SHUTDOWN             # node exiting: locked
string ZONE_CLEAR=CLEAR
string ZONE_WARNING=WARNING
string ZONE_STOP=STOP
string ZONE_CONTACT=CONTACT
string ZONE_UNKNOWN=UNKNOWN                # no valid scan
builtin_interfaces/Time stamp
string state
string enforcement                         # filter | lock_only
string active_zone                         # zone of the travel direction; worst of both when stationary
string front_zone
string rear_zone
string reason                              # empty only in ACTIVE with both zones CLEAR
bool locked                                # last value published on the lock topic
bool braking                               # brake pulse active
bool estop_latched
bool dock_profile_active
float32 nearest_obstacle_distance          # m, footprint edge to nearest valid point; inf none; -1 unknown
float32 nearest_obstacle_bearing           # rad in robot_frame (0 when none/unknown)
float32 front_clearance                    # m in the front corridor; inf none; -1 unknown
float32 rear_clearance
float32 max_forward_speed                  # m/s allowed now (>= 0)
float32 max_reverse_speed                  # m/s magnitude allowed now (>= 0)
float32 scan_age                           # s (steady clock), -1 never received
float32 odom_age
float32 cmd_age
```
Add it to `bumperbot_msgs/CMakeLists.txt`. Its dependency is `builtin_interfaces`, which is already
there.

### `safety_supervisor` interfaces (identical in C++ and Python)

Subscriptions (all QoS reliable, depth 10, unless noted):
- `scan_topic` (`scan`, `sensor_msgs/LaserScan`, sensor-data QoS: best effort, depth 5, so it is
  compatible with both reliable and best-effort publishers);
- `odom_topic` (`bumperbot_controller/odom`, `nav_msgs/Odometry`) when `use_odom`;
- `cmd_vel_in_topic` (`safety/cmd_vel_in`, `geometry_msgs/Twist`) in filter mode. In lock_only mode
  the node subscribes to `cmd_vel_out_topic` read only (observed command, used for direction only);
- `estop_topic` (`safety/estop`, `std_msgs/Bool`);
- `dock_status_topic` (`dock/status`, `bumperbot_msgs/DockStatus`, reliable, transient_local,
  depth 1).

Publishers:
- `lock_topic` (`safety_stop`, `std_msgs/Bool`), on every tick and on every change;
- `cmd_vel_out_topic` (`bumperbot_controller/cmd_vel_unstamped`, `Twist`), **filter mode only**. It
  publishes the limited command on each input, and a zero on each tick while the input is stale or
  the node is locked;
- `status_topic` (`safety/status`, `SafetyStatus`, reliable, transient_local, depth 1).

Services: `~/reset` and `~/trigger_estop` (`std_srvs/Trigger`). Reset replies:
- not latched: `success=true`, message `not latched`;
- refused: `success=false`, and the message names the rule (`estop input active`,
  `fault: <reason>`, `obstacle in contact zone (front|rear)`);
- accepted: `success=true`, `reset`.

TF reads:
- `robot_frame <- scan.header.frame_id`, looked up with timeout 0 on each scan until found, then
  cached. It is static in the URDF. The MuJoCo laser is yawed by about pi, which is why the
  transform is required;
- `DockStatus.frame_id -> robot_frame`, only for the dock gate.
The node publishes no TF.

### Parameters (declared in code, same defaults in both languages, also in `config/safety_supervisor.yaml`)

| Name | Default | Meaning |
|---|---|---|
| `enforcement` | `filter` | `filter` or `lock_only` (other value: refuse to start) |
| `robot_frame` | `base_footprint` | non-empty |
| `scan_topic` / `odom_topic` / `cmd_vel_in_topic` / `cmd_vel_out_topic` | `scan` / `bumperbot_controller/odom` / `safety/cmd_vel_in` / `bumperbot_controller/cmd_vel_unstamped` | non-empty; in and out must differ |
| `lock_topic` / `status_topic` / `estop_topic` / `dock_status_topic` | `safety_stop` / `safety/status` / `safety/estop` / `dock/status` | non-empty |
| `use_odom` | true | odom required (INITIALIZING/FAULT without it) |
| `footprint` | `[0.10, 0.125, 0.10, -0.125, -0.10, -0.125, -0.10, 0.125]` | flat x,y list in `robot_frame`: even length, >= 3 points, finite, simple polygon, area > 0, contains (0, 0) |
| `lateral_margin` | 0.03 | m added to the footprint half width for the corridors |
| `stop_margin` | 0.05 | m from the footprint edge (> `contact_margin`) |
| `stop_release_hysteresis` | 0.02 | m, the STOP zone is left at `d > stop_margin + hysteresis` |
| `slowdown_distance` | 0.35 | m, warning ramp length (> 0) |
| `contact_margin` | 0.015 | m (>= 0) |
| `max_linear_speed` | 0.7 | m/s, hard clamp and top of the ramp (joystick turbo max) |
| `max_angular_speed` | 5.0 | rad/s, hard clamp only (joystick angular max) |
| `response_time` | 0.3 | s, scan period + tick + margin, for the braking limit |
| `max_decel` | 0.5 | m/s^2, for the braking limit |
| `stopped_speed` | 0.02 | m/s, "moving" threshold |
| `brake_hold_time` | 0.5 | s, minimum brake pulse |
| `scan_timeout` / `odom_timeout` / `cmd_timeout` | 0.6 / 0.5 / 0.5 | s (> 0); the MuJoCo scan is 5 Hz |
| `recovery_time` | 0.5 | s of healthy data before FAULT -> ACTIVE |
| `max_invalid_beam_ratio` | 0.5 | NaN beams / total; above it the scan is invalid (inf = no return, valid) |
| `latch_faults` | false | a FAULT latches the e-stop |
| `latch_on_contact` | true | contact-zone violation while moving toward it latches the e-stop |
| `rate` / `status_rate` | 20.0 / 5.0 | Hz, steady timers |
| `dock_approach_enabled` | true | |
| `dock_max_speed` | 0.06 | m/s reverse cap in the profile (<= `max_linear_speed`) |
| `dock_stop_margin` | 0.01 | m rear stop margin in the profile (<= `stop_margin`) |
| `dock_approach_radius` | 0.8 | m from `DockStatus.dock_pose` |
| `dock_approach_max_time` | 30.0 | s per DOCKING episode |
| `dock_status_timeout` | 2.0 | s (`dock_manager` publishes at 2 Hz) |
| `debug_inject_tick_exceptions` | 0 | test hook: the first N ticks raise on purpose (documented, default off) |
| `use_sim_time` | false (true from the sim launch) | |

**Startup errors** are a parameter type mismatch (for example a quoted number or an int for a
double) caught around `declare_parameter`, or a `validate_config` failure. On one:
- the node creates the lock publisher (the configured topic, or `safety_stop` if that parameter is
  itself invalid);
- it publishes `true` 3 times over 0.3 s and logs the reason;
- it exits with code 1 within 5 s and never publishes on `cmd_vel_out_topic`.
In filter mode a dead supervisor means nothing forwards the `twist_mux` output, so the robot stops
anyway (diff_drive `cmd_vel_timeout` 0.5 s).

### Logic (pure `safety_logic`, same in both languages)

- `Config` + `validate_config(cfg) -> str` ("" when valid).
- Geometry:
  - `polygon_is_simple`, `point_in_polygon`, `distance_to_polygon`;
  - `footprint_extents(fp) -> (x_front, x_rear, half_width)`, from the bounding box;
  - `scan_to_points(ranges, angle_min, angle_increment, range_min, range_max, tf_x, tf_y, tf_yaw)
    -> (points, invalid_ratio)`. NaN counts as invalid, inf and out-of-range are skipped, and points
    are in `robot_frame`.
- `evaluate_points(points, cfg)`:
  - returns `front_clearance` (min `x - x_front` over points with `x >= x_front` and
    `|y| <= half_width + lateral_margin`), `rear_clearance` (mirror), `nearest_distance` /
    `nearest_bearing` (to the footprint polygon), and `contact_front` / `contact_rear`;
  - points strictly inside the footprint count as self-hits and are ignored;
  - no point gives `inf`.
- `speed_limit(d, margin, cfg) -> v_max`:
  - 0 for `d <= margin`;
  - otherwise `min(max_linear_speed, ramp, brake)` with
    `ramp = max_linear_speed * (d - margin) / slowdown_distance` and
    `brake = max_decel * (-response_time + sqrt(response_time^2 + 2 (d - margin) / max_decel))`;
  - it is monotonic non-decreasing in `d`, and `inf` gives `max_linear_speed`.
- `zone_for(d, prev_zone, margin, cfg)`: CONTACT / STOP / WARNING / CLEAR, with hysteresis when
  leaving STOP. WARNING means `v_max < max_linear_speed`.
- `travel_direction(v_cmd, v_odom, stopped_speed) -> +1 | -1 | 0`. It uses the odometry sign when
  `|v_odom| > stopped_speed`, otherwise the command sign when `|v_cmd| > stopped_speed`, otherwise 0.
- `DockGate`: `update(now, enabled, status_state, status_age, distance_or_none) -> (active, reason)`.
  - The episode starts on the transition into DOCKING.
  - It expires after `dock_approach_max_time`, gives reasons `dock approach time limit` /
    `dock status stale` / `dock out of radius` / `dock TF unavailable`, and re-arms only after the
    state leaves DOCKING.
- `SafetyCore` (no clock, no threads, the node drives it):
  - `update(now, inputs) -> Outputs{state, locked, braking, estop_latched, front_zone, rear_zone,
    active_zone, max_forward, max_reverse, reason, dock_profile_active}`.
    `inputs = {scan_ok, scan_age, scan_invalid_reason, tf_ok, clearances, odom_age, v_odom,
    cmd_age, v_cmd, estop_input, dock: DockGate result, tick_exception}`.
  - `request_reset() -> (ok, message)`, `trigger_estop(reason)`, `shutdown()`.
  - `limit_command(v, w, outputs) -> (v, w)`:
    - zero when locked;
    - otherwise `v` is clamped to `[-max_reverse, +max_forward]` and `w` to `+-max_angular_speed`;
    - non-finite input gives zero.
  - Rules:
    - Priority order: SHUTDOWN > ESTOP > INITIALIZING > FAULT > DOCK_APPROACH/ACTIVE.
    - A brake pulse starts when the zone of `travel_direction` becomes STOP or CONTACT while moving
      toward it. It ends when `brake_hold_time` has elapsed and `|v_odom| < stopped_speed` (time
      only without odom). A reset, a fault entry and a shutdown clear it.
    - A contact violation (zone CONTACT in the travel direction while moving, the rear exempt in
      the profile) with `latch_on_contact` latches ESTOP.
    - Reset rules are as in the Goal. A successful reset clears the brake, the latch reason and the
      dock episode state. A new FAULT/ESTOP never inherits a previous reason.

### Node runtime (both languages)

- Single-threaded executor, one callback group, so no shared-state races. Callbacks only store the
  messages (scan -> points/clearances, odom, cmd, estop, dock status). The **steady** timer at
  `rate` runs the tick: build the inputs, `core.update`, publish the lock, the zero output if
  needed, and the status on change or at `status_rate`.
  - C++ uses `create_wall_timer`.
  - Python uses `create_timer(..., clock=Clock(clock_type=ClockType.STEADY_TIME))`.
- The `cmd_vel_in` callback publishes `limit_command(msg, last_outputs)` right away (latency <= one
  tick for a new scan).
- Every callback and the tick body are wrapped in try/catch-all. An exception is logged (throttled)
  and fed as `tick_exception`, which gives FAULT `internal error: <what>`, locked, zero output. The
  timer keeps running, and the node recovers after `recovery_time` of clean ticks (AC12).
- Signals:
  - C++: `rclcpp::init(..., SignalHandlerOptions::None)` plus `std::signal` setting an atomic flag;
    the loop is `spin_some(50 ms)` until the flag is set.
  - Python: `rclpy.init(signal_handler_options=SignalHandlerOptions.NO)` plus `signal.signal`, and
    a `spin_once(timeout_sec=0.05)` loop.
  - On exit:
    - `core.shutdown()`;
    - publish status SHUTDOWN, lock `true` 3x and a zero `cmd_vel_out` 3x (filter mode), 0.05 s
      apart;
    - destroy the node;
    - shutdown guarded by try/except (`RCLError` from `rclpy._rclpy_pybind11`, double shutdown);
    - exit code 0.
- **Lock state on shutdown: LOCKED.** `twist_mux` lock timeout 0.0 never expires, so the robot stays
  stopped until a new supervisor publishes `false`. This is documented, with the manual release
  command for an intentional removal.

### TF

```
map -(amcl)-> odom -(diff_drive)-> base_footprint -> base_link -> laser_link (yaw ~pi)
```
- Required: `base_footprint <- laser_link` (static, robot_state_publisher). Missing gives
  INITIALIZING with reason `waiting for TF base_footprint <- laser_link`.
- The scan `frame_id` must be non-empty and constant. A change gives FAULT `scan frame changed`.
- Optional: `map -> base_footprint`, only for the dock gate. When it is missing the profile is off
  and protection stays normal.
- `robot_frame` must equal Nav2 `robot_base_frame` and the `dock_manager` `robot_frame`
  (`base_footprint`), checked by the config test.

### Launch

- `bumperbot_utils/launch/safety_supervisor.launch.py` (new, simulator independent):
  - args `use_sim_time` (`false`), `use_python` (`false`), `enforcement` (`filter`) and `config`
    (default `<share bumperbot_utils>/config/safety_supervisor.yaml` via
    `get_package_share_directory`);
  - starts exactly one `safety_supervisor` (C++ unless `use_python`).
- `bumperbot_controller/launch/joystick_teleop.launch.py` (modify): new arg `cmd_vel_out`, default
  `bumperbot_controller/cmd_vel_unstamped` (identical to today), forwarded to `twist_mux_launch.py`.
- `bumperbot_bringup/launch/simulated_robot.launch.py` (modify): new args `safety` (`supervisor` |
  `legacy`, default `supervisor`) and `safety_use_python` (`false`).
  - `supervisor`: the joystick include gets `cmd_vel_out:=safety/cmd_vel_in`, and
    `safety_supervisor.launch.py` is included with `use_sim_time:=true`, `enforcement:=filter`.
  - `legacy`: the default `cmd_vel_out` and the old `safety_stop` node, exactly as before.
  - Other entities are unchanged.
- `real_robot.launch.py`, `mission_sim.launch.py`, `dock_sim.launch.py` and
  `auto_explore.launch.py` are **not modified**.

### Data flow (sim, `safety:=supervisor`)

```
 joy_vel(99) key_vel(90) dock_vel(30) nav_vel(20) explore_vel(10)
        \        |          |           |          /
         +---------------- twist_mux ------------+ <-- /safety_stop lock (255) <--+
                              | safety/cmd_vel_in (remapped output)                |
                              v                                                    |
  /scan (LaserScan) --> +------------------------------------+ --> /safety_stop ---+
  TF laser->base -----> | safety_supervisor (C++ | Python)   | --> /safety/status (SafetyStatus)
  /bumperbot_controller | SafetyCore, zones, DockGate        |
     /odom -----------> |   filter: limit_command()          | <-- ~/reset, ~/trigger_estop (Trigger)
  /safety/estop (Bool)->|                                    | <-- /dock/status (DockStatus, latched)
                        +------------------------------------+     + TF map->base_footprint
                              | bumperbot_controller/cmd_vel_unstamped (limited)
                              v
                         twist_relay -> /bumperbot_controller/cmd_vel -> diff_drive -> MuJoCo/Gazebo
```
In lock_only mode (hardware recipe) the `twist_mux` output goes straight to
`cmd_vel_unstamped`. The supervisor only observes it and drives `/safety_stop`.

In node tests `MockSafetyWorld` replaces everything around the node. It is one rclpy node in the
test process that:
- publishes synthetic scans from obstacle points given in `base_footprint` (converted into
  `laser_link` ranges with the URDF yaw 3.14, 360 beams, 5 Hz, optional NaN/empty/bad-frame
  scans);
- publishes static TF `base_footprint -> laser_link` (switchable) and optional TF
  `map -> base_footprint`;
- publishes odom (scripted v, or integrating `cmd_vel_out` with a first-order lag);
- publishes `cmd_vel_in` and `safety/estop`;
- publishes `/dock/status` (transient_local);
- optionally publishes `/clock`;
- records every lock, output and status message with wall time.

## Files

| Path | create/modify | Purpose |
|---|---|---|
| `src/bumperbot_msgs/msg/SafetyStatus.msg` | create | `/safety/status` message |
| `src/bumperbot_msgs/CMakeLists.txt` | modify | Generate `SafetyStatus.msg` |
| `src/bumperbot_utils/include/bumperbot_utils/safety_logic.hpp` | create | C++ pure logic API |
| `src/bumperbot_utils/src/safety_logic.cpp` | create | C++ pure logic |
| `src/bumperbot_utils/include/bumperbot_utils/safety_supervisor.hpp` | create | C++ node class |
| `src/bumperbot_utils/src/safety_supervisor.cpp` | create | C++ node (steady timer, signals, fail-safe exit) |
| `src/bumperbot_utils/bumperbot_utils/safety_logic.py` | create | Python pure logic |
| `src/bumperbot_utils/bumperbot_utils/safety_supervisor.py` | create | Python node (executable `safety_supervisor.py`) |
| `src/bumperbot_utils/config/safety_supervisor.yaml` | create | Default parameters (English comments) |
| `src/bumperbot_utils/launch/safety_supervisor.launch.py` | create | Simulator-independent launch |
| `src/bumperbot_utils/CMakeLists.txt` | modify | `safety_logic` lib, `safety_supervisor` exe, install py exe + `launch` + `config`, register tests (only if files exist) with TIMEOUTs |
| `src/bumperbot_utils/package.xml` | modify | `bumperbot_msgs`, `geometry_msgs`, `nav_msgs`, `std_srvs`, `tf2`, `tf2_ros` depend; test: `ament_cmake_gtest`, `ament_cmake_pytest`, `launch_testing_ament_cmake`, `launch_testing_ros`, `yaml_cpp_vendor`, `python3-yaml` |
| `src/bumperbot_utils/test/data/safety_cases.yaml` | create | Shared parity case table |
| `src/bumperbot_utils/test/data/invalid_*.yaml`, `valid_*.yaml` | create | ROS param files for AC3 (odd footprint, bowtie, origin outside, NaN, negative margin, zero timeout, bad enforcement, empty frame, in == out topic, dock_max_speed > max, quoted number, int for double, duplicate key) |
| `src/bumperbot_utils/test/test_safety_logic.py` | create | AC2 (Python) |
| `src/bumperbot_utils/test/test_safety_logic.cpp` | create | AC2 (gtest) |
| `src/bumperbot_utils/test/mock_safety_world.py` | create | `MockSafetyWorld` helper |
| `src/bumperbot_utils/test/safety_harness.py` | create | Process helpers (start exe, signals, exit codes, logs) |
| `src/bumperbot_utils/test/test_safety_supervisor.launch.py` | create | AC5-AC11 (+ runtime param parity), parametrized cpp/py |
| `src/bumperbot_utils/test/test_safety_robustness.py` | create | AC3, AC12 (process level, both impls) |
| `src/bumperbot_utils/test/test_safety_static.py` | create | AC1, AC15 |
| `src/bumperbot_utils/test/test_safety_docs.py` | create | AC19 |
| `src/bumperbot_controller/launch/joystick_teleop.launch.py` | modify | `cmd_vel_out` arg (default unchanged) |
| `src/bumperbot_bringup/launch/simulated_robot.launch.py` | modify | `safety` / `safety_use_python` args, supervisor wiring, legacy path |
| `src/bumperbot_bringup/test/test_safety_launch_config.py` | create | AC14 |
| `src/bumperbot_bringup/test/test_safety_twist_mux.launch.py` | create | AC13 (real twist_mux + supervisor, both impls) |
| `src/bumperbot_bringup/test/test_safety_sim_mujoco.launch.py` | create | AC18 smoke + opt-in E2E (`BUMPERBOT_SAFETY_E2E=1`) |
| `src/bumperbot_bringup/test/test_dock_launch_config.py` | modify | "unchanged by the auto-docking feature" diffs `base..<last auto-docking commit>` instead of `base..worktree` (the intent is kept, later features may change `simulated_robot`) |
| `src/bumperbot_bringup/test/test_mission_launch_config.py` | modify | Same fix for the mission feature (`base..<last waypoint-mission-manager commit>`) |
| `src/bumperbot_bringup/CMakeLists.txt` | modify | Register the 3 new tests |
| `src/bumperbot_bringup/package.xml` | modify | `std_srvs` test depend |
| `src/bumperbot_mujoco/test/test_safety_supervisor_sim.py` | create | AC4, AC16 (BumperbotSim closed loop, no ROS) |
| `src/bumperbot_mujoco/package.xml` | modify | `bumperbot_utils` test_depend |
| `README.md` | modify | Capabilities, safety supervisor quick start, real-robot recipe, e-stop/reset commands |
| `doc/07_lidar_and_sensor_processing.md` | modify | Supervisor design: zones, limits, brake pulse, e-stop, watchdog, dock profile; legacy node kept |
| `doc/03_ros2_nodes_topics_services.md` | modify | New node, topics, services, `SafetyStatus`, `safety/cmd_vel_in` |
| `doc/04_tf_and_coordinate_frames.md` | modify | `laser_link -> base_footprint` use, dock-gate TF |
| `doc/02_system_architecture.md` | modify | Filter after `twist_mux`, lock semantics |
| `doc/10_navigation.md` | modify | Dock-approach profile, interaction with `dock_manager` lock handling |
| `doc/12_troubleshooting.md` | modify | Locked at startup (INITIALIZING), stale scan FAULT, lock kept after supervisor exit, reset refusals, real-robot recipe; keep the existing required sentences |
| `doc/11_build_run_and_command_reference.md` | modify | Build / run / test commands |
| `doc/01_repository_structure.md` | modify | New files |

## Acceptance criteria

- AC1: `bumperbot_msgs` generates `SafetyStatus.msg` with exactly the fields and constants above.
  `ros2 interface show bumperbot_msgs/msg/SafetyStatus` succeeds, and `DockRobot`, `DockStatus`,
  `ExecuteMission`, `MissionStatus`, `Fibonacci`, `AddTwoInts` and `GetTransform` still show.
- AC2: Python and C++ `safety_logic` give identical results for every case in
  `test/data/safety_cases.yaml` (numbers within 1e-9, strings and states exact). The table covers at
  least:
  - geometry: simple vs self-intersecting polygon, point-in-polygon, distance-to-polygon, extents;
  - `scan_to_points` with a yaw of 3.14 and an offset TF, inf, NaN and out-of-range handling,
    invalid ratio;
  - `evaluate_points`: corridors, self-hits ignored, side points outside the corridor ignored;
  - `speed_limit` monotonic on a grid of 200 distances, 0 at the margin, max at inf;
  - `zone_for` hysteresis;
  - `travel_direction`;
  - `validate_config` (every invalid case listed in AC3 that reaches validation);
  - `DockGate`: enable, each fallback reason, time limit, no re-arm within the same episode, re-arm
    after leaving DOCKING;
  - `SafetyCore` step sequences:
    - INITIALIZING -> ACTIVE;
    - brake pulse start and release;
    - front STOP blocks only forward, rear STOP blocks only reverse;
    - contact latch while moving, no latch when stationary;
    - e-stop via request and input;
    - every reset refusal rule and acceptance;
    - FAULT on stale scan, odom and exception, and recovery after `recovery_time`;
    - `latch_faults`;
    - SHUTDOWN locks;
    - reuse after a reset carries no reason, brake or episode state.
- AC3: (process level, both executables) config handling.
  - The shipped `config/safety_supervisor.yaml` and `test/data/valid_*.yaml` start, and each reaches
    a first `/safety/status` with `state == INITIALIZING` and `locked == true`.
  - For every `test/data/invalid_*.yaml` each executable exits with code 1 within 5 s, publishes
    `safety_stop: true` at least once and publishes nothing on `cmd_vel_out_topic`.
  - For the duplicate-key file both executables give the same outcome: both exit 1, or both run with
    identical `get_parameters` values.
- AC4: (headless MuJoCo, no ROS) the default `footprint` contains the xy projection of every
  collision geom AABB of the robot body in `robot.xml` (in `base_footprint`). The laser site pose
  from MuJoCo matches the URDF `laser_joint` (xy 0.01 m, yaw 0.01 rad), so mock and sim
  `scan_to_points` agree.
- AC5: (node, `impl=cpp` and `impl=py`, `MockSafetyWorld`) fail-safe startup.
  - With no scan, no TF and no odom, status is `INITIALIZING`, `locked == true`, and every
    `cmd_vel_out` (fed `cmd_vel_in` 0.3 m/s) is zero for 5 s. The reason names the missing input
    (`scan`, then `TF`, then `odom` as each one is provided in turn).
  - Status arrives at >= 4 Hz throughout, and the node does not exit.
  - With all inputs present, `ACTIVE` and `locked == false` follow within 1 s, and the output
    equals the input (0.3).
  - The startup log line contains `(C++)` or `(Python)` matching the impl.
- AC6: (node, both impls) stop zone. The robot is commanded at 0.3 m/s and odom follows the output.
  - An obstacle appears 0.03 m ahead of the front edge.
  - Within 0.25 s of the first scan containing it, `cmd_vel_out.linear.x == 0`, `/safety_stop` is
    true (brake) and status `front_zone == STOP`.
  - After odom reports < `stopped_speed`, the lock is released within `brake_hold_time + 0.3` s.
  - Forward commands are still zeroed while the obstacle stays.
  - A -0.1 m/s command passes (|out| = 0.1).
  - After the obstacle is removed, 0.3 passes again within `scan period + 0.3` s.
- AC7: (node, both impls) direction-aware slowdown.
  - With `cmd_vel_in` = 0.7 and a front obstacle at d in {0.60, 0.40, 0.30, 0.20, 0.10, 0.06}:
    - the output speeds are monotonic non-increasing;
    - each equals `speed_limit(d)` within 1e-3;
    - each is < 0.7 for `d < stop_margin + slowdown_distance`;
    - `front_zone` is WARNING there and CLEAR at 0.60.
  - An obstacle 0.03 m behind the rear edge: a 0.3 forward command passes unchanged, and -0.1
    gives 0.
  - A point at (0.0, 0.25), beside the robot and outside both corridors, limits nothing.
  - Angular velocity passes unchanged in ACTIVE (clamped only at `max_angular_speed`).
- AC8: (node, both impls) watchdog.
  - (a) Scans stop: within `scan_timeout + 0.15` s the state is FAULT, `locked`, output zero, and
    the reason contains `scan`. Scans resume: ACTIVE within `recovery_time + 0.5` s.
  - (b) The same for odom with `use_odom: true`.
  - (c) `cmd_vel_in` stops: zero output within `cmd_timeout + 0.15` s, state stays ACTIVE, `locked`
    is false.
  - (d) Each invalid scan gives FAULT with a specific reason: NaN ratio 0.8, empty ranges,
    `angle_increment` 0, `frame_id` changed.
  - (e) A NaN `cmd_vel_in` gives a zero output.
  - (f) `latch_faults: true` with a scan dropout gives ESTOP, which stays after scans resume.
  - (g) TF removed before the first scan: INITIALIZING, no hang. Once TF appears, ACTIVE within 1 s.
- AC9: (node, both impls) e-stop latch.
  - (a) `~/trigger_estop` gives ESTOP, locked, output zero within 0.15 s, and `estop_latched`. It
    persists for >= 5 s with clear surroundings.
  - (b) `safety/estop` true triggers it too. While it stays true, `~/reset` returns
    `success == false` with `estop input`. After it turns false, reset returns `success == true`
    and the state is ACTIVE with `locked == false`.
  - (c) Reset during a scan dropout is refused (`fault`).
  - (d) Reset with a point 0.01 m ahead of the front edge is refused (`contact zone`). With the
    point at 0.04 m (STOP, not contact) it is accepted, forward commands stay zero and reverse
    passes.
  - (e) A point entering the front contact zone while odom is 0.1 m/s forward latches ESTOP. The
    same point with the robot stationary does not latch (only STOP).
  - (f) Reset while not latched gives `success == true`, `not latched`.
- AC10: (node, both impls) dock-approach profile. The mock has TF `map -> base_footprint` with the
  robot 0.4 m from `dock_pose`, `/dock/status` DOCKING at 2 Hz, and a rear obstacle 0.03 m behind
  the rear edge.
  - (a) Input -0.15 gives output -0.06 (`dock_max_speed`), `DOCK_APPROACH`, and
    `dock_profile_active`. A front obstacle at 0.03 m still zeros forward commands. A rear point
    at 0.005 m (inside `dock_stop_margin`) zeros reverse. A rear contact while moving does not
    latch ESTOP.
  - (b) Each of the following keeps normal protection (reverse output 0, status ACTIVE, reason
    names the cause):
    - no `/dock/status`;
    - state ALIGNING;
    - status stopped for `dock_status_timeout + 0.5` s;
    - robot 1.0 m from `dock_pose`;
    - no `map` TF;
    - `dock_approach_enabled: false`;
    - `enforcement: lock_only`.
  - (c) With `dock_approach_max_time: 3.0` the profile ends after 3 s (+-0.3), reverse goes to 0
    and the reason is `dock approach time limit`. It stays off while DOCKING continues. After
    UNDOCKED -> DOCKING it is active again.
  - (d) Each fallback takes effect within one tick (<= 0.1 s) of the condition change.
- AC11: (node, both impls) lock_only mode.
  - The node has no publisher on `bumperbot_controller/cmd_vel_unstamped`, and no subscription on
    `safety/cmd_vel_in` (graph check).
  - With observed command 0.2 and odom 0.2 forward, an obstacle 0.03 m ahead gives the lock true
    within 0.25 s. It is released after odom stops plus `brake_hold_time`.
  - The same obstacle behind the robot while moving forward gives no lock.
  - Status `enforcement == lock_only`, and `max_forward_speed` reports the computed limit.
- AC12: (process level, both impls) robustness.
  - (a) SIGINT and (b) SIGTERM in ACTIVE with the input flowing:
    - exit code 0 within 5 s, with no `Traceback` in stderr;
    - the last `/safety_stop` message is `true`;
    - the last `cmd_vel_out` is zero;
    - the last status is `SHUTDOWN`.
  - (c) `debug_inject_tick_exceptions: 3`: status FAULT with reason `internal error`, the process
    keeps running, and ACTIVE comes back within `recovery_time + 1` s.
  - (d) `use_sim_time: true` with a test `/clock` that stops after 2 s:
    - status keeps arriving at >= 2 Hz wall for 5 s;
    - a scan dropout during the stall still gives FAULT within `scan_timeout + 0.15` s;
    - the lock keeps being republished.
  - (e) A second SIGINT during shutdown does not produce a traceback or a non-zero exit.
- AC13: (`launch_testing`, real `twist_mux` with the repo's `twist_mux_topics.yaml` /
  `twist_mux_locks.yaml`, output remapped to `safety/cmd_vel_in` as in `simulated_robot`, plus the
  supervisor for both impls and the mock scan/TF/odom). Observed on
  `bumperbot_controller/cmd_vel_unstamped`:
  - (a) `key_vel` 0.7 with a front obstacle at 0.20 m gives output `speed_limit(0.20)` (1e-3).
  - (b) `dock_vel` -0.1 with a rear obstacle at 0.03 m gives 0, and with a front-only obstacle it
    gives -0.1.
  - (c) E-stop: `key_vel`, `dock_vel` and `nav_vel` all give 0 for 2 s.
  - (d) A brake pulse: no non-zero output while `/safety_stop` is true.
  - (e) SIGTERM to the supervisor, with `key_vel` still publishing: no non-zero output for 3 s,
    because the lock is latched and there is no filter.
  - (f) `/safety_stop` has exactly one publisher (the supervisor).
  - Statically, `twist_mux_locks.yaml` is unchanged vs `main`.
- AC14: launch and configuration.
  - `--show-args` succeeds for `bumperbot_utils safety_supervisor.launch.py`, `simulated_robot`,
    `real_robot`, `mission_sim`, `dock_sim` and `auto_explore`, and `joystick_teleop`. It lists the
    new args with their defaults (`safety` = `supervisor`, `safety_use_python` = `false`,
    `cmd_vel_out` = `bumperbot_controller/cmd_vel_unstamped`).
  - `simulated_robot` with the defaults gives exactly one `safety_supervisor` node (C++) with
    `use_sim_time` true and `enforcement` filter, zero `safety_stop` nodes, and joystick
    `cmd_vel_out == safety/cmd_vel_in`.
  - `simulated_robot safety:=legacy` gives one `safety_stop` node, no supervisor, and the default
    `cmd_vel_out`.
  - `real_robot.launch.py`, `mission_sim.launch.py`, `dock_sim.launch.py`, `auto_explore.launch.py`
    and `src/bumperbot_firmware` are unchanged vs the feature base, and `real_robot.launch.py` is
    unchanged vs `main`.
  - No launch string literal starts with `/home` or `/opt`.
  - The `robot_frame` values of the supervisor, `dock_manager` and Nav2 are all `base_footprint`.
- AC15: static checks.
  - `safety_supervisor.cpp/.hpp/.py` and `safety_logic.*` contain no `mujoco`, `gazebo`, `\bgz\b`
    or `/clock` tokens (case-insensitive).
  - All new and modified source, YAML and launch files have ASCII-only comments.
  - Both sources declare the same parameter names with the same defaults, equal to
    `config/safety_supervisor.yaml` and the table above.
  - At runtime (inside the AC5 launch test), `list_parameters` / `get_parameters` on both running
    nodes return identical sets. Name lookup is retried for 10 s against discovery races.
- AC16: (headless MuJoCo, pytest, no ROS) `BumperbotSim` on `small_house.xml` runs the Python
  `SafetyCore` closed loop. Scans run at 5 Hz through `scan_to_points` with the MuJoCo laser pose,
  the tick at 20 Hz, odom comes from the wheel velocities, and diff-drive inverse kinematics uses
  r = 0.033, b = 0.17.
  - (a) Approach a wall at a commanded 0.3 m/s (noise 0): the robot halts. The minimum ground-truth
    front clearance is >= `stop_margin - 0.02` and > 0 (no contact). The output speed is monotonic
    non-increasing (tolerance 1e-6) while the clearance decreases.
  - (b) At a steady 0.3 m/s in open space, a virtual point is injected 0.04 m ahead into the next
    scan:
    - the robot speed is < 0.02 m/s within 1.0 s of the injection;
    - the travel after the injection is <= 0.20 m;
    - the lock (brake) was true.
  - (c) Back toward a wall until the rear STOP, then command +0.2: the robot moves >= 0.10 m forward
    within 1.5 s (an obstacle behind does not stop forward motion).
  - (d) Stop feeding scans while moving at 0.3: FAULT and a zero command within
    `scan_timeout + 0.1` s, and the robot speed is < 0.02 within 1.0 s after that.
  - (e) Contact latch: with the injected point at 0.01 m while moving, ESTOP latches. Reset is
    refused until the point is removed, then accepted.
- AC17: no regression. These pass unchanged except for the two listed test fixes:
  - `bumperbot_bringup`: `test_launch_config.py`, `test_mission_launch_config.py`,
    `test_dock_launch_config.py`, `test_dock_twist_mux.launch.py`,
    `test_auto_explore_mujoco.launch.py`, `test_mission_sim_mujoco.launch.py` (smoke) and
    `test_dock_sim_mujoco.launch.py` (AC24 smoke);
  - every `bumperbot_navigation` test (including `test_dock_docs.py` and `test_dock_static.py`);
  - every `bumperbot_mujoco` test (including `test_auto_explore_sim.py` and
    `test_dock_precision_sim.py`);
  - the `bumperbot_mapping` tests.
  `git diff` of the two modified tests touches only the feature-base functions.
- AC18: headless MuJoCo stack `simulated_robot.launch.py simulator:=mujoco world_name:=small_house
  viewer:=false use_rviz:=false`.
  - Smoke (always): within 120 s, `/safety/status` is `ACTIVE` with `locked == false`, `/safety_stop`
    has exactly one publisher, and `cmd_vel_unstamped` has the supervisor as its publisher.
  - Opt-in E2E (`BUMPERBOT_SAFETY_E2E=1`), within 300 s:
    - the test picks a heading with a wall 1-3 m ahead (from `/scan` + TF), rotates to it with
      `key_vel`, then publishes `key_vel` 0.3. The robot stops, with a ground-truth
      (`/mujoco/ground_truth`) front clearance from the min `/scan` > 0.03 m, and `front_zone` STOP
      or WARNING with `max_forward_speed == 0` at rest;
    - `key_vel` -0.2 moves the robot away (>= 0.1 m);
    - `~/trigger_estop` gives no motion with `key_vel` 0.2 for 3 s, then `~/reset` succeeds and the
      robot moves again.
- AC19: docs.
  - `README.md`, `doc/07`, `doc/03`, `doc/02`, `doc/04`, `doc/10`, `doc/11`, `doc/12` and `doc/01`
    mention `safety_supervisor`, `/safety/status`, `SafetyStatus`, `safety/cmd_vel_in`,
    `~/reset` (as `/safety_supervisor/reset`), `trigger_estop`, `enforcement`, `lock_only`,
    `dock_max_speed` and `safety:=legacy`.
  - README, doc/10 and doc/12 state that `real_robot.launch.py` does not start `safety_supervisor`
    and give the hardware recipe.
  - doc/12 documents "lock stays engaged after the supervisor exits" and the reset refusal rules.
  - The existing `test_dock_docs.py` sentences stay intact.

## Test plan

Build once (no `--symlink-install`; `--packages-up-to` on a clean tree):
```
cd <ws> && colcon build --packages-up-to bumperbot_bringup bumperbot_utils bumperbot_mujoco && source install/setup.bash
```
All pytest commands are prefixed with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`. Node launch tests run
with `ROS_DOMAIN_ID=92 ROS_LOCALHOST_ONLY=1`, MuJoCo tests with 93. Every `add_launch_test` /
`ament_add_pytest_test` has a generous registered TIMEOUT. Tests wait for the first
transient_local status for up to 10 s, because late latched delivery is normal. Parameters are set
through the start-up params file, never through `ros2 param set` races.

| AC | Type | File | Command | Pass condition |
|---|---|---|---|---|
| AC1 | pytest (CLI) | `src/bumperbot_utils/test/test_safety_static.py -k interfaces` | `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/bumperbot_utils/test/test_safety_static.py -k interfaces -v` | `ros2 interface show` returns 0 for 1 new + 7 old interfaces; all fields/constants present |
| AC2 | pytest unit + gtest | `test/test_safety_logic.py`, `test/test_safety_logic.cpp` | `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/bumperbot_utils/test/test_safety_logic.py -v` and `./build/bumperbot_utils/test_safety_logic_cpp` | every case passes in both (TIMEOUT 120) |
| AC3, AC12 | pytest, process level, both impls | `src/bumperbot_utils/test/test_safety_robustness.py` | `ROS_DOMAIN_ID=92 ROS_LOCALHOST_ONLY=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 timeout 600 python3 -m pytest src/bumperbot_utils/test/test_safety_robustness.py -v` | all parametrized cases pass (TIMEOUT 600) |
| AC4, AC16 | pytest, headless MuJoCo, no ROS | `src/bumperbot_mujoco/test/test_safety_supervisor_sim.py` | `PYTHONPATH=src/bumperbot_utils:$PYTHONPATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/bumperbot_mujoco/test/test_safety_supervisor_sim.py -v` | footprint/laser checks and (a)-(e) hold (wall < 180 s) |
| AC5-AC11 (+ runtime part of AC15) | launch_testing, `@launch_testing.parametrize("impl", ["cpp", "py"])`, MockSafetyWorld | `src/bumperbot_utils/test/test_safety_supervisor.launch.py` | `ROS_DOMAIN_ID=92 ROS_LOCALHOST_ONLY=1 timeout 900 launch_test src/bumperbot_utils/test/test_safety_supervisor.launch.py` | methods `test_startup_failsafe`, `test_stop_zone`, `test_slowdown_direction`, `test_watchdog`, `test_estop_latch`, `test_dock_profile`, `test_lock_only`, `test_parameter_parity_runtime` pass for both impls (TIMEOUT 900; split into two files if tight) |
| AC13 | launch_testing (real twist_mux + supervisor) | `src/bumperbot_bringup/test/test_safety_twist_mux.launch.py` | `ROS_DOMAIN_ID=92 ROS_LOCALHOST_ONLY=1 timeout 300 launch_test src/bumperbot_bringup/test/test_safety_twist_mux.launch.py` | (a)-(f) pass for both impls; locks file `git diff --quiet main` (TIMEOUT 300) |
| AC14 | pytest (static + `--show-args` + launch description introspection) | `src/bumperbot_bringup/test/test_safety_launch_config.py` | `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/bumperbot_bringup/test/test_safety_launch_config.py -v` | all checks pass (TIMEOUT 180) |
| AC15 | pytest static | `src/bumperbot_utils/test/test_safety_static.py` | `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/bumperbot_utils/test/test_safety_static.py -v` | no forbidden tokens; ASCII comments; same parameter set/defaults in both sources + YAML |
| AC17 | colcon test (regression) | existing files listed in AC17 | `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon test --packages-select bumperbot_msgs bumperbot_utils bumperbot_controller bumperbot_navigation bumperbot_mapping bumperbot_mujoco bumperbot_bringup --event-handlers console_direct+ && colcon test-result --verbose` | 0 failures; opt-in E2Es reported as skipped |
| AC18 | launch_testing, headless MuJoCo (smoke always, E2E opt-in) | `src/bumperbot_bringup/test/test_safety_sim_mujoco.launch.py` | `ROS_DOMAIN_ID=93 ROS_LOCALHOST_ONLY=1 timeout 400 launch_test src/bumperbot_bringup/test/test_safety_sim_mujoco.launch.py`; E2E: `BUMPERBOT_SAFETY_E2E=1 ROS_DOMAIN_ID=93 ROS_LOCALHOST_ONLY=1 timeout 700 launch_test src/bumperbot_bringup/test/test_safety_sim_mujoco.launch.py` | smoke within 120 s; E2E steps within 300 s (TIMEOUT 900 registered) |
| AC19 | pytest (grep) | `src/bumperbot_utils/test/test_safety_docs.py` + existing `src/bumperbot_navigation/test/test_dock_docs.py` | `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/bumperbot_utils/test/test_safety_docs.py src/bumperbot_navigation/test/test_dock_docs.py -v` | all required strings found |

`MockSafetyWorld` (in `test/mock_safety_world.py`, own executor thread in the test process) has:
- `obstacles` (list of base_footprint points, settable at any time);
- `scan_mode` (`normal`, `stop`, `nan`, `empty`, `zero_increment`, `bad_frame`);
- `tf_laser` / `tf_map` on/off, with `robot_in_map`;
- `odom_mode` (`follow_output` with a 0.1 s lag, `scripted v`, `stop`);
- `cmd_in` (value or None);
- `estop` (None / bool);
- `dock_state` (None / string, 2 Hz, `dock_pose`);
- `clock` (off / running / stopped).
It records `(t_wall, value)` for the lock, the output and the status. The node under test runs with
`test/data/valid_test.yaml`, which is the shipped defaults except `dock_approach_max_time: 3.0` where
a case needs it.

## Implementation steps

1. `bumperbot_msgs`: add `SafetyStatus.msg`, update CMake, build, then `ros2 interface show` (AC1).
2. Python `safety_logic.py`: config + `validate_config`, geometry, `scan_to_points`,
   `evaluate_points`, `speed_limit`, `zone_for`, `travel_direction`, `DockGate`, `SafetyCore`,
   `limit_command`. Write `test/data/safety_cases.yaml` + `test_safety_logic.py` (AC2 Python).
3. C++ `safety_logic.hpp/.cpp` with the same API, plus the gtest reading the same YAML through
   `TEST_DATA_DIR` (AC2 C++). CMake: `safety_logic` library with no ROS deps; yaml-cpp only for the
   test target.
4. `bumperbot_mujoco/test/test_safety_supervisor_sim.py` (AC4, AC16). Tune the defaults
   (`stop_margin`, `slowdown_distance`, `response_time`, `max_decel`, footprint) until AC16 passes,
   and mirror any default change into the C++ code, the YAML and the case table.
5. Python node `safety_supervisor.py`:
   - parameter declaration in a try block, with the fail-safe exit path;
   - subscriptions/publishers per enforcement mode;
   - TF cache;
   - steady tick timer;
   - services;
   - status publishing;
   - exception wrapping;
   - signal handling and the final locked publish.
   Add `config/safety_supervisor.yaml` and `launch/safety_supervisor.launch.py`.
6. `test/mock_safety_world.py` + `test_safety_supervisor.launch.py` with `impl=py`. Make AC5-AC11
   pass.
7. C++ node `safety_supervisor.hpp/.cpp` mirroring step 5: same names, defaults and reasons, the
   startup log `(C++)`, `SignalHandlerOptions::None`, `create_wall_timer`. Enable `impl=cpp` and
   make AC5-AC11 pass.
8. `safety_harness.py` + `test_safety_robustness.py` + `test/data/invalid_*.yaml` (AC3, AC12) for
   both impls.
9. `bumperbot_utils/CMakeLists.txt` / `package.xml`: install targets, `launch` and `config`; test
   registration (files must exist, `APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:${CMAKE_CURRENT_SOURCE_DIR}/test`,
   TIMEOUTs as in the test plan).
10. `joystick_teleop.launch.py` `cmd_vel_out` arg (default unchanged), then the
    `simulated_robot.launch.py` `safety` / `safety_use_python` args and wiring with the legacy path
    kept.
11. Fix the two feature-base tests (`test_dock_launch_config.py`, `test_mission_launch_config.py`)
    so they compare `base..<last commit of that feature's .claude/features dir>`. Write
    `test_safety_launch_config.py` (AC14) and `test_safety_twist_mux.launch.py` (AC13), and
    register them in the bringup CMake.
12. `test_safety_static.py` (AC1, AC15).
13. `test_safety_sim_mujoco.launch.py`: smoke, then the E2E once with `BUMPERBOT_SAFETY_E2E=1`.
    Then run the existing MuJoCo launch tests (`auto_explore`, `mission_sim`, `dock_sim`) to confirm
    the supervisor does not block them (AC17).
14. Docs:
    - README, doc/07, doc/03, doc/02, doc/04, doc/10, doc/11, doc/12 and doc/01, with the commands:
      - `ros2 launch bumperbot_bringup simulated_robot.launch.py simulator:=mujoco`;
      - `ros2 topic echo /safety/status`;
      - `ros2 service call /safety_supervisor/trigger_estop std_srvs/srv/Trigger`;
      - `ros2 service call /safety_supervisor/reset std_srvs/srv/Trigger`;
      - `ros2 launch bumperbot_utils safety_supervisor.launch.py enforcement:=lock_only`;
      - the manual lock release `ros2 topic pub --once /safety_stop std_msgs/msg/Bool "{data: false}"`;
    - keep the `test_dock_docs.py` sentences (for example "real_robot.launch.py does not start
      safety_stop", "the user starts safety_stop ...") while adding the supervisor;
    - run `test_safety_docs.py` + `test_dock_docs.py` (AC19).
15. Full `colcon test` on all touched packages plus `bumperbot_navigation` and `bumperbot_mapping`
    (AC17).

## Risks and open questions

- **Virtual keepout / speed polygons dropped.** Map-frame zones need AMCL, so the safety layer
  would lose validity exactly when localization fails. Nav2 costmap filters already cover planning
  around keepouts. Possible follow-up: the supervisor reads polygons in `map` only as an *additional*
  speed cap, and falls back to the plain local limits when `map -> base_footprint` is stale.
- **Brake pulse vs `dock_manager`.** `dock_manager` treats `/safety_stop` true as a pause bounded by
  `safety_stop_timeout`. With the supervisor the lock is short (brake pulses). Afterwards the filter
  silently zeros motion toward the obstacle, so `dock_manager` sees no motion and fails the attempt
  by STALL (5 s) instead of SAFETY_TIMEOUT (10 s). That is safe but the outcome differs. Follow-up
  (open question): `dock_manager` could read `/safety/status` (`max_reverse_speed == 0`) and count
  it as a lock.
- **LiDAR blind zone.** Returns below `range_min` (0.12 m from the laser) are inf in MuJoCo and on
  many real sensors, so an obstacle hugging the front edge (0.10-0.12 m) can be invisible. The
  contact zone partly sits there. Bumpers or a lower range_min are a hardware fix; this is
  documented in doc/12.
- **In-place rotation.** Zones limit only linear motion, so a rectangular footprint corner can touch
  a side obstacle while rotating. A swept-circle angular guard is a follow-up (open question:
  default margin vs Nav2 narrow passages in `small_house`).
- **Footprint default.** `[+-0.10, +-0.125]` contains the MuJoCo collision geoms including the wheels (wheel extent 0.118 m, AC4). Iteration 1 used +-0.09, which the AC4 test showed to miss the wheels; corrected in iteration 2.
  If the coder finds the mesh AABB larger, the default grows and the existing MuJoCo
  explore/mission/dock smoke tests must be re-run (larger zones could slow Nav2 in doorways).
- **Lock left engaged after supervisor exit** (twist_mux lock timeout 0.0). This is intended
  fail-safe behavior, but it surprises users who stop only the supervisor. It is documented with the
  manual release command.
- **Real robot.** Without the user starting a LiDAR driver plus `safety_supervisor.launch.py
  enforcement:=lock_only`, nothing provides the lock on hardware. That is unchanged from today and
  now stated clearly. lock_only has no slowdown and no dock profile. Open question: should a future
  `real_robot.launch.py` gain an opt-in `use_safety_supervisor:=false` arg? That would require
  relaxing the "unchanged vs main" tests of earlier features, so it is the user's call.
- **Editing earlier features' invariant tests.** `test_dock_launch_config.py` and
  `test_mission_launch_config.py` compare `simulated_robot.launch.py` to their feature base against
  the work tree, so any later change fails them. The fix compares the base to the end commit of
  that feature, which keeps the original intent ("that feature did not change it"). This is a
  deliberate, reviewed change and listed in Files.
- **Moving obstacles in MuJoCo.** The LiDAR ray cast sees only static group-4 geoms, so a moving
  body is not used. AC16b injects a virtual point into the real scan instead, and AC16a uses real
  wall geometry.
- **Twist_mux behavior under a lock** (whether it publishes a zero or nothing) does not matter: the
  filter publishes zero on a stale input. AC13 checks the observable output only.
- **Scan header stamps are not age-checked** (receipt time on the steady clock is used) so that
  sim-time / real-time skew cannot cause false faults. A stamp-age check could be a follow-up
  parameter.
- `test_safety_supervisor.launch.py` with two impls and eight scenario methods may approach the
  900 s TIMEOUT. If so, split it into `..._zones` and `..._latch`.
