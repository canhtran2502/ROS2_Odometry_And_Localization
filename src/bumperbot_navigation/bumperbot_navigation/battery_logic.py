"""
Pure battery logic (no ROS dependency).

  * BatteryModel: the simulated battery of battery_sim (SIMULATION ONLY).
  * BatteryMonitorLogic: the state machine of battery_monitor (simulator independent). It turns
    battery, mission and dock information into requests (cancel mission, dock, undock); the node
    performs them and reports the results back.
  * FallbackClock: node time with a wall-clock fallback for a stalled time source.

Python only (the request asks for C++ / Python parity for dock_manager only).
"""
import math
from dataclasses import dataclass
from typing import List

# sensor_msgs/BatteryState power supply status values
STATUS_UNKNOWN = 0
STATUS_CHARGING = 1
STATUS_DISCHARGING = 2
STATUS_NOT_CHARGING = 3
STATUS_FULL = 4

# BatteryMonitorLogic states
MONITOR_WAITING_FOR_BATTERY = "WAITING_FOR_BATTERY"
MONITOR_MONITORING = "MONITORING"
MONITOR_CANCELING_MISSION = "CANCELING_MISSION"
MONITOR_DOCKING = "DOCKING"
MONITOR_CHARGING = "CHARGING"
MONITOR_UNDOCKING = "UNDOCKING"
MONITOR_ERROR = "ERROR"

# Requests returned by BatteryMonitorLogic.update()
ACTION_CANCEL_MISSION = "CANCEL_MISSION"
ACTION_SEND_DOCK = "SEND_DOCK"
ACTION_SEND_UNDOCK = "SEND_UNDOCK"
# The node forgets the pending dock/undock request (its handle and callbacks) and cancels it
ACTION_ABANDON_DOCK = "ABANDON_DOCK"

# Results reported with BatteryMonitorLogic.on_dock_result()
RESULT_DOCKED = "DOCKED"
RESULT_ALREADY_DOCKED = "ALREADY_DOCKED"
RESULT_UNDOCKED = "UNDOCKED"
RESULT_REJECTED = "REJECTED"
RESULT_FAILED = "FAILED"


@dataclass
class BatteryParams:
    idle_drain_rate: float = 0.0005     # fraction / s
    linear_drain_rate: float = 0.004    # fraction / s per m/s
    angular_drain_rate: float = 0.001   # fraction / s per rad/s
    charge_rate: float = 0.01           # fraction / s while docked
    empty_voltage: float = 10.5
    full_voltage: float = 12.6


class BatteryModel:
    """Simulated battery. The percentage is a fraction in [0, 1]."""

    def __init__(self, params: BatteryParams = None, initial_percentage: float = 1.0):
        self._params = params if params is not None else BatteryParams()
        self._percentage = self._clamp01(initial_percentage)

    @staticmethod
    def _clamp01(value: float) -> float:
        if not math.isfinite(value):
            return 0.0
        return max(0.0, min(1.0, value))

    @property
    def percentage(self) -> float:
        return self._percentage

    def set_percentage(self, value: float) -> None:
        """Jump to the given charge (for demos and tests)."""
        self._percentage = self._clamp01(value)

    def step(self, dt: float, linear: float, angular: float, docked: bool) -> None:
        """Advance the model by dt seconds (clamped to [0, 1])."""
        if not math.isfinite(dt):
            dt = 0.0
        dt = max(0.0, min(1.0, dt))
        par = self._params
        if docked:
            self._percentage = self._clamp01(self._percentage + par.charge_rate * dt)
            return
        rate = (par.idle_drain_rate + par.linear_drain_rate * abs(linear)
                + par.angular_drain_rate * abs(angular))
        self._percentage = self._clamp01(self._percentage - rate * dt)

    def power_supply_status(self, docked: bool) -> int:
        if docked:
            return STATUS_FULL if self._percentage >= 1.0 else STATUS_CHARGING
        return STATUS_DISCHARGING

    def voltage(self) -> float:
        par = self._params
        return par.empty_voltage + (par.full_voltage - par.empty_voltage) * self._percentage


class FallbackClock:
    """
    Node time with a wall-clock fallback.

    now() returns the node time plus the wall time that passed while the node time did not advance
    for stall_timeout wall seconds. 0 disables the fallback.
    """

    def __init__(self, stall_timeout: float):
        self._stall_timeout = stall_timeout
        self._last_node = None
        self._last_wall = 0.0
        self._anchor = 0.0
        self._offset = 0.0

    def now(self, node_now: float, wall_now: float) -> float:
        if self._last_node is None or node_now != self._last_node:
            self._last_node = node_now
            self._last_wall = wall_now
            self._anchor = wall_now
        elif self._stall_timeout > 0.0 and wall_now - self._last_wall >= self._stall_timeout:
            self._offset += wall_now - self._anchor
            self._anchor = wall_now
        return node_now + self._offset


@dataclass
class MonitorParams:
    low_threshold: float = 0.20
    full_threshold: float = 0.95
    rearm_hysteresis: float = 0.05
    battery_timeout: float = 5.0
    mission_cancel_timeout: float = 180.0
    dock_retry_delay: float = 5.0
    dock_request_timeout: float = 120.0
    max_dock_attempts: int = 3
    undock_when_full: bool = True


class BatteryMonitorLogic:
    """
    Low-battery dock trigger.

    Inputs: on_battery, on_mission_status, on_dock_status, on_dock_result. update(now) advances the
    state machine and returns the requests (ACTION_*) the node has to perform. All times are
    seconds on one consistent clock.
    """

    def __init__(self, params: MonitorParams = None):
        self._params = params if params is not None else MonitorParams()
        self._state = MONITOR_WAITING_FOR_BATTERY
        self._percentage = math.nan
        self._charging = False
        self._battery_time = None
        self._mission_active = False
        self._docked = False
        self._armed = True
        self._attempts = 0
        self._goal_active = False
        self._next_send = 0.0
        self._request_start = 0.0
        self._cancel_start = 0.0
        self._last_cancel = -math.inf
        self._goal_start = 0.0
        self._goal_accepted = False
        self._server_lost_since = None
        self._message = ""

    # ----- read-only state -----
    @property
    def state(self) -> str:
        return self._state

    @property
    def attempts(self) -> int:
        return self._attempts

    @property
    def armed(self) -> bool:
        return self._armed

    @property
    def goal_active(self) -> bool:
        return self._goal_active

    @property
    def message(self) -> str:
        return self._message

    # ----- inputs -----
    def on_battery(self, now: float, percentage: float, charging: bool) -> None:
        self._percentage = percentage
        self._charging = bool(charging)
        self._battery_time = now

    def on_mission_status(self, active: bool) -> None:
        self._mission_active = bool(active)

    def on_dock_status(self, docked: bool) -> None:
        self._docked = bool(docked)

    def on_dock_accepted(self) -> None:
        """Report that dock_manager accepted the pending DockRobot goal."""
        self._goal_accepted = True

    def on_dock_result(self, now: float, result: str) -> None:
        """Report the end of the DockRobot goal (or its rejection)."""
        par = self._params
        if self._state == MONITOR_DOCKING:
            self._goal_active = False
            if result in (RESULT_DOCKED, RESULT_ALREADY_DOCKED):
                self._attempts = 0
                self._docked = True
                self._state = MONITOR_CHARGING
            elif result == RESULT_REJECTED:
                # A rejection (dependencies not ready) does not count as an attempt
                self._next_send = now + par.dock_retry_delay
            else:
                self._attempts += 1
                if self._attempts >= par.max_dock_attempts:
                    self._fail("docking failed {} times".format(self._attempts))
                else:
                    self._next_send = now + par.dock_retry_delay
                    self._request_start = now
        elif self._state == MONITOR_UNDOCKING:
            self._goal_active = False
            if result == RESULT_UNDOCKED:
                self._docked = False
                self._armed = False
                self._state = MONITOR_MONITORING
            elif result == RESULT_REJECTED:
                self._next_send = now + par.dock_retry_delay
            else:
                self._fail("undocking failed")

    def reset(self) -> None:
        """Leave ERROR (or any state) and monitor again."""
        self._state = MONITOR_MONITORING
        self._armed = True
        self._attempts = 0
        self._goal_active = False
        self._message = ""

    # ----- driving -----
    def update(self, now: float, server_ready: bool = True) -> List[str]:
        par = self._params
        actions: List[str] = []
        fresh = (self._battery_time is not None and math.isfinite(self._percentage)
                 and now - self._battery_time <= par.battery_timeout)

        if self._state in (MONITOR_WAITING_FOR_BATTERY, MONITOR_MONITORING):
            if not fresh:
                self._state = MONITOR_WAITING_FOR_BATTERY
                return actions
            self._state = MONITOR_MONITORING
            if not self._armed and self._percentage >= par.low_threshold + par.rearm_hysteresis:
                self._armed = True
            if self._armed and self._percentage < par.low_threshold and not self._charging:
                self._attempts = 0
                if self._mission_active:
                    self._state = MONITOR_CANCELING_MISSION
                    self._cancel_start = now
                    self._request_cancel(now, actions)
                else:
                    self._begin_docking(now, actions)
        elif self._state == MONITOR_CANCELING_MISSION:
            if not self._mission_active or now - self._cancel_start >= par.mission_cancel_timeout:
                self._begin_docking(now, actions)
            else:
                # The cancel can get lost (service not ready, lost response): repeat it
                self._resend_cancel(now, actions)
        elif self._state == MONITOR_DOCKING:
            # A mission must not run while docking: cancel it again when one shows up
            self._resend_cancel(now, actions)
            if self._goal_active:
                if self._request_hung(now, server_ready):
                    actions.append(ACTION_ABANDON_DOCK)
                    self.on_dock_result(now, RESULT_FAILED)
            else:
                if now - self._request_start >= par.dock_request_timeout:
                    self._fail("the dock request was not accepted in time")
                elif now >= self._next_send:
                    self._start_request(now)
                    actions.append(ACTION_SEND_DOCK)
        elif self._state == MONITOR_CHARGING:
            self._resend_cancel(now, actions)
            if not self._docked:
                self._state = MONITOR_MONITORING
            elif par.undock_when_full and fresh and self._percentage >= par.full_threshold:
                self._state = MONITOR_UNDOCKING
                self._request_start = now
                self._start_request(now)
                actions.append(ACTION_SEND_UNDOCK)
        elif self._state == MONITOR_UNDOCKING:
            if self._goal_active:
                if self._request_hung(now, server_ready):
                    actions.append(ACTION_ABANDON_DOCK)
                    self.on_dock_result(now, RESULT_FAILED)
            else:
                if now - self._request_start >= par.dock_request_timeout:
                    self._fail("the undock request was not accepted in time")
                elif now >= self._next_send:
                    self._start_request(now)
                    actions.append(ACTION_SEND_UNDOCK)
        return actions

    # ----- internals -----
    def _begin_docking(self, now: float, actions: List[str]) -> None:
        self._state = MONITOR_DOCKING
        self._request_start = now
        self._next_send = now
        self._start_request(now)
        actions.append(ACTION_SEND_DOCK)

    def _start_request(self, now: float) -> None:
        self._goal_active = True
        self._goal_start = now
        self._goal_accepted = False
        self._server_lost_since = None

    def _request_hung(self, now: float, server_ready: bool) -> bool:
        """Return True when a request got no answer or dock_manager is gone for too long."""
        timeout = self._params.dock_request_timeout
        if not self._goal_accepted and now - self._goal_start >= timeout:
            return True
        if server_ready:
            self._server_lost_since = None
        elif self._server_lost_since is None:
            self._server_lost_since = now
        elif now - self._server_lost_since >= timeout:
            return True
        return False

    def _request_cancel(self, now: float, actions: List[str]) -> None:
        self._last_cancel = now
        actions.append(ACTION_CANCEL_MISSION)

    def _resend_cancel(self, now: float, actions: List[str]) -> None:
        if self._mission_active and now - self._last_cancel >= self._params.dock_retry_delay:
            self._request_cancel(now, actions)

    def _fail(self, message: str) -> None:
        self._state = MONITOR_ERROR
        self._goal_active = False
        self._message = message
