"""
Pure mission logic (no ROS dependency): station loading and the MissionTracker state machine.

The C++ twin lives in include/bumperbot_navigation/mission_logic.hpp and src/mission_logic.cpp.
Both implementations expose the same API and are checked against test/data/mission_cases.yaml.
"""
import collections.abc
import math
import re
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import yaml

# Mission states (same strings as the MissionStatus.msg constants)
STATE_IDLE = "IDLE"
STATE_NAVIGATING = "NAVIGATING"
STATE_DWELLING = "DWELLING"
STATE_RETURNING_TO_DOCK = "RETURNING_TO_DOCK"
STATE_COMPLETED = "COMPLETED"
STATE_ABORTED = "ABORTED"
STATE_CANCELED = "CANCELED"
STATE_DOCK_FAILED = "DOCK_FAILED"

# Outcomes (same values as the ExecuteMission.action constants)
OUTCOME_COMPLETED = 0
OUTCOME_COMPLETED_WITH_SKIPS = 1
OUTCOME_ABORTED = 2
OUTCOME_CANCELED = 3
OUTCOME_DOCK_FAILED = 4

# Step kinds
STEP_NAVIGATE = "NAVIGATE"
STEP_DWELL = "DWELL"
STEP_FINISHED = "FINISHED"


@dataclass
class Station:
    name: str
    x: float
    y: float
    yaw: float
    dwell: float


@dataclass
class StationSet:
    frame_id: str = "map"
    dock_station: str = "dock"
    stations: List[Station] = field(default_factory=list)

    def find(self, name: str) -> Optional[Station]:
        for station in self.stations:
            if station.name == name:
                return station
        return None


class _UniqueKeyLoader(yaml.SafeLoader):
    """
    SafeLoader that rejects duplicate mapping keys and also types exponent forms as floats.

    PyYAML (YAML 1.1) types 1e3 as a string; the extra resolver below types it as a float, as
    yaml-cpp does. Numeric rule (same in the C++ twin): a numeric field must be a plain
    (unquoted) YAML number, int or float; quoted strings are rejected.
    """

    def construct_mapping(self, node, deep=False):
        seen = set()
        for key_node, _value_node in node.value:
            key = self.construct_object(key_node, deep=True)
            if isinstance(key, collections.abc.Hashable):
                if key in seen:
                    raise ValueError("duplicate key '{}'".format(key))
                seen.add(key)
        return super().construct_mapping(node, deep)


_UniqueKeyLoader.add_implicit_resolver(
    "tag:yaml.org,2002:float",
    re.compile(r"^[-+]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)[eE][-+]?[0-9]+$"),
    list("-+0123456789."))


def _string(data: dict, key: str, default: str) -> str:
    """Return an optional string field; when present it must be a non-null string."""
    if key not in data:
        return default
    if not isinstance(data[key], str):
        raise ValueError("'{}' is not a string".format(key))
    return data[key]


def _number(value, what: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("{} is not a number".format(what))
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("{} is not finite".format(what))
    return value


def load_stations(path: str) -> StationSet:
    """Load and validate a stations file. Raises ValueError on any problem."""
    try:
        with open(path, "r") as stream:
            data = yaml.load(stream, Loader=_UniqueKeyLoader)
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError("cannot read stations file '{}': {}".format(path, exc))
    if not isinstance(data, dict):
        raise ValueError("stations file '{}' is not a mapping".format(path))

    result = StationSet()
    result.frame_id = _string(data, "frame_id", "map")
    result.dock_station = _string(data, "dock_station", "dock")

    stations = data.get("stations")
    if not isinstance(stations, dict) or not stations:
        raise ValueError("'stations' is missing or empty")

    for name, entry in stations.items():
        name = str(name)
        if not isinstance(entry, dict):
            raise ValueError("station '{}' is not a mapping".format(name))
        values = {}
        for key in ("x", "y", "yaw", "dwell"):
            if key not in entry:
                raise ValueError("station '{}' misses field '{}'".format(name, key))
            values[key] = _number(entry[key], "station '{}' field '{}'".format(name, key))
        if values["dwell"] < 0.0:
            raise ValueError("station '{}' has a negative dwell".format(name))
        result.stations.append(
            Station(name, values["x"], values["y"], values["yaw"], values["dwell"]))

    if result.find(result.dock_station) is None:
        raise ValueError("dock station '{}' is not defined".format(result.dock_station))
    return result


def validate_mission(names: List[str], station_set: StationSet) -> str:
    """Return an empty string when the mission is valid, otherwise the reason."""
    if len(names) == 0:
        return "empty station list"
    for name in names:
        if station_set.find(name) is None:
            return "unknown station '{}'".format(name)
    return ""


def yaw_to_quaternion(yaw: float) -> Tuple[float, float]:
    """Return (z, w) of a planar rotation."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def planar_distance(ax: float, ay: float, bx: float, by: float) -> float:
    return math.hypot(ax - bx, ay - by)


@dataclass
class TrackerParams:
    max_retries: int = 2
    max_consecutive_failures: int = 2
    dock_max_retries: int = 3
    return_to_dock: bool = True
    goal_tolerance: float = 0.2


@dataclass
class Step:
    kind: str = STEP_FINISHED
    station: str = ""
    attempt: int = 0
    docking: bool = False
    dwell: float = 0.0


class MissionTracker:
    """
    Mission state machine. No clock, no threads: the node drives it.

    next_step() is idempotent: it returns the current step until a report_* call advances it.
    """

    _PHASE_STATIONS = 0
    _PHASE_DWELL = 1
    _PHASE_DOCK = 2
    _PHASE_DONE = 3

    def __init__(self, station_set: StationSet, names: List[str], params: TrackerParams = None):
        self._set = station_set
        self._names = list(names)
        self._params = params if params is not None else TrackerParams()
        self._phase = self._PHASE_STATIONS
        self._index = 0
        self._attempt = 1
        self._consecutive = 0
        self._at_dock = False
        self._cancel = False
        self._abort = False
        self._started = False
        self._current_station = ""
        self._visited: List[str] = []
        self._skipped: List[str] = []
        self._outcome = OUTCOME_COMPLETED
        self._docked = False
        self._message = ""
        self._state = STATE_IDLE

    # ----- read-only state -----
    @property
    def state(self) -> str:
        return self._state

    @property
    def current_station(self) -> str:
        return self._current_station

    @property
    def current_index(self) -> int:
        if self._phase in (self._PHASE_STATIONS, self._PHASE_DWELL) and self._started:
            return self._index
        return -1

    @property
    def attempt(self) -> int:
        return self._attempt

    @property
    def progress(self) -> float:
        if not self._names:
            return 1.0
        done = len(self._visited) + len(self._skipped)
        return min(1.0, float(done) / float(len(self._names)))

    @property
    def visited(self) -> List[str]:
        return list(self._visited)

    @property
    def skipped(self) -> List[str]:
        return list(self._skipped)

    @property
    def outcome(self) -> int:
        return self._outcome

    @property
    def docked(self) -> bool:
        return self._docked

    @property
    def message(self) -> str:
        return self._message

    @property
    def finished(self) -> bool:
        return self._phase == self._PHASE_DONE

    # ----- driving -----
    def next_step(self) -> Step:
        self._started = True
        if self._phase == self._PHASE_STATIONS:
            name = self._names[self._index]
            self._current_station = name
            self._state = STATE_NAVIGATING
            # Driving to a station other than the dock: the robot leaves the dock
            if name != self._set.dock_station:
                self._at_dock = False
            return Step(STEP_NAVIGATE, name, self._attempt, False, 0.0)
        if self._phase == self._PHASE_DWELL:
            name = self._names[self._index]
            self._current_station = name
            self._state = STATE_DWELLING
            station = self._set.find(name)
            return Step(STEP_DWELL, name, self._attempt, False, station.dwell)
        if self._phase == self._PHASE_DOCK:
            self._current_station = self._set.dock_station
            self._state = STATE_RETURNING_TO_DOCK
            return Step(STEP_NAVIGATE, self._set.dock_station, self._attempt, True, 0.0)
        return Step(STEP_FINISHED, "", 0, False, 0.0)

    def report_navigation(self, succeeded: bool, distance_error: float) -> None:
        if self._phase not in (self._PHASE_STATIONS, self._PHASE_DOCK):
            return
        docking = self._phase == self._PHASE_DOCK
        name = self._set.dock_station if docking else self._names[self._index]
        reached = (succeeded and not math.isnan(distance_error)
                   and distance_error <= self._params.goal_tolerance)

        if reached:
            self._at_dock = (name == self._set.dock_station)
            if docking:
                self._finish()
                return
            self._visited.append(name)
            self._consecutive = 0
            station = self._set.find(name)
            if station.dwell > 0.0:
                self._phase = self._PHASE_DWELL
            else:
                self._advance()
            return

        # Failed attempt: the robot may have moved, so it is no longer known to be at the dock.
        self._at_dock = False
        limit = (self._params.dock_max_retries if docking else self._params.max_retries) + 1
        if self._attempt < limit:
            self._attempt += 1
            return
        if docking:
            self._finish(dock_failed=True)
            return
        self._skipped.append(name)
        self._consecutive += 1
        max_fail = self._params.max_consecutive_failures
        if max_fail > 0 and self._consecutive >= max_fail:
            self._abort = True
            self._message = "aborted after {} consecutive skipped stations".format(
                self._consecutive)
            self._begin_docking()
            return
        self._advance()

    def report_dwell_done(self) -> None:
        if self._phase == self._PHASE_DWELL:
            self._advance()

    def request_cancel(self) -> None:
        if self._phase == self._PHASE_DONE or self._cancel:
            return
        self._cancel = True
        if self._message == "":
            self._message = "canceled by the client"
        if self._phase == self._PHASE_DOCK:
            return
        self._begin_docking()

    def abort(self, reason: str) -> None:
        if self._phase == self._PHASE_DONE or self._abort:
            return
        self._abort = True
        self._message = reason
        if self._phase == self._PHASE_DOCK:
            return
        self._begin_docking()

    # ----- internals -----
    def _advance(self) -> None:
        self._index += 1
        self._attempt = 1
        if self._index < len(self._names):
            self._phase = self._PHASE_STATIONS
            return
        self._begin_docking()

    def _begin_docking(self) -> None:
        self._attempt = 1
        forced = self._cancel or self._abort
        if (not forced and not self._params.return_to_dock) or self._at_dock:
            self._finish()
            return
        self._phase = self._PHASE_DOCK

    def _finish(self, dock_failed: bool = False) -> None:
        self._phase = self._PHASE_DONE
        if dock_failed:
            self._docked = False
            self._outcome = OUTCOME_DOCK_FAILED
            self._state = STATE_DOCK_FAILED
            self._message = "the dock could not be reached"
            return
        self._docked = self._at_dock
        if self._cancel:
            self._outcome = OUTCOME_CANCELED
            self._state = STATE_CANCELED
        elif self._abort:
            self._outcome = OUTCOME_ABORTED
            self._state = STATE_ABORTED
        elif self._skipped:
            self._outcome = OUTCOME_COMPLETED_WITH_SKIPS
            self._state = STATE_COMPLETED
            self._message = "completed with skipped stations"
        else:
            self._outcome = OUTCOME_COMPLETED
            self._state = STATE_COMPLETED
            self._message = "mission completed"
