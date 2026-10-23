"""
Pure-Python core of the frontier exploration (no rclpy, only numpy and scipy).

The ROS node ``frontier_explorer.py`` is a thin wrapper around this module, so the
algorithms can be tested headless with pytest, with synthetic grids or with the MuJoCo
simulation core.

Conventions
-----------
* A grid is a ``numpy.int8`` array of shape (H, W), row-major, same layout as
  ``nav_msgs/OccupancyGrid.data``: -1 unknown, 0..49 free, >= 50 occupied.
* A cell is a tuple ``(cx, cy)`` (column, row), so ``grid[cy, cx]``.
* World coordinates are in the map frame, in meters.
* The time base is whatever the caller passes in (ROS time in the node, sim time in tests).

Python-only note: this is a new node, so the "keep C++ and Python in sync" rule does not
apply. The logic is shared with the headless tests, which a C++ port could not run. A C++
port would be a separate feature that takes this module as its reference.
"""

import heapq
import math
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np
from scipy import ndimage

Cell = Tuple[int, int]

UNKNOWN = -1
OCCUPIED_THRESHOLD = 50

# Explorer states
WAITING_FOR_MAP = "WAITING_FOR_MAP"
INITIAL_SPIN = "INITIAL_SPIN"
EXPLORING = "EXPLORING"
COMPLETE = "COMPLETE"


def wrap_angle(a):
    """Wrap an angle (or an array of angles) to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# ---------------------------------------------------------------------------
# Grid utilities
# ---------------------------------------------------------------------------

@dataclass
class GridInfo:
    resolution: float
    origin_x: float
    origin_y: float
    width: int
    height: int

    def world_to_cell(self, x: float, y: float) -> Cell:
        return (int(math.floor((x - self.origin_x) / self.resolution)),
                int(math.floor((y - self.origin_y) / self.resolution)))

    def cell_to_world(self, cx: int, cy: int) -> Tuple[float, float]:
        """World coordinates of the center of a cell."""
        return (self.origin_x + (cx + 0.5) * self.resolution,
                self.origin_y + (cy + 0.5) * self.resolution)

    def in_bounds(self, cell: Cell) -> bool:
        return 0 <= cell[0] < self.width and 0 <= cell[1] < self.height


def grid_from_msg_data(data, width: int, height: int) -> np.ndarray:
    """Convert OccupancyGrid.data (flat, row-major) into an int8 array of shape (H, W)."""
    return np.asarray(data, dtype=np.int8).reshape((height, width))


def build_costmap(grid: np.ndarray, inflation_cells: float,
                  unknown_is_obstacle: bool = True) -> np.ndarray:
    """
    Return a boolean mask of traversable cells.

    A cell is not traversable when it is closer than (or exactly at) ``inflation_cells`` to
    an occupied cell, or when it is unknown and ``unknown_is_obstacle`` is set.
    """
    occupied = grid >= OCCUPIED_THRESHOLD
    if occupied.any():
        dist = ndimage.distance_transform_edt(~occupied)
    else:
        dist = np.full(grid.shape, np.inf)
    traversable = (~occupied) & (dist > inflation_cells)
    if unknown_is_obstacle:
        traversable &= grid != UNKNOWN
    return traversable


def detect_frontiers(grid: np.ndarray) -> np.ndarray:
    """Boolean mask of free cells with at least one unknown 4-neighbour."""
    unknown = grid == UNKNOWN
    free = (grid >= 0) & (grid < OCCUPIED_THRESHOLD)
    near_unknown = np.zeros_like(unknown)
    near_unknown[1:, :] |= unknown[:-1, :]
    near_unknown[:-1, :] |= unknown[1:, :]
    near_unknown[:, 1:] |= unknown[:, :-1]
    near_unknown[:, :-1] |= unknown[:, 1:]
    return free & near_unknown


@dataclass
class Frontier:
    cells: np.ndarray        # (N, 2) int array of (cx, cy)
    size: int
    centroid_cell: Cell


def cluster_frontiers(mask: np.ndarray, min_size: int) -> List[Frontier]:
    """8-connected clusters of frontier cells; clusters smaller than min_size are dropped."""
    labels, count = ndimage.label(mask, structure=np.ones((3, 3), dtype=int))
    frontiers: List[Frontier] = []
    if count == 0:
        return frontiers
    objects = ndimage.find_objects(labels)
    for idx, sl in enumerate(objects, start=1):
        ys, xs = np.nonzero(labels[sl] == idx)
        size = len(xs)
        if size < min_size:
            continue
        xs = xs + sl[1].start
        ys = ys + sl[0].start
        cells = np.stack([xs, ys], axis=1)
        centroid = (int(round(float(xs.mean()))), int(round(float(ys.mean()))))
        frontiers.append(Frontier(cells=cells, size=size, centroid_cell=centroid))
    return frontiers


# ---------------------------------------------------------------------------
# Reachability and planning
# ---------------------------------------------------------------------------

def snap_to_traversable(traversable: np.ndarray, cell: Cell,
                        max_radius: int = 7) -> Optional[Cell]:
    """
    Nearest traversable cell to ``cell`` (BFS over the 8-neighbourhood, at most
    ``max_radius`` steps). Returns the cell itself when it is already traversable and
    None when nothing is found.
    """
    h, w = traversable.shape
    cx, cy = cell
    if 0 <= cx < w and 0 <= cy < h and traversable[cy, cx]:
        return (cx, cy)
    seen = {(cx, cy)}
    queue = deque([((cx, cy), 0)])
    while queue:
        (x, y), d = queue.popleft()
        if d >= max_radius:
            continue
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                nx, ny = x + dx, y + dy
                if (nx, ny) in seen:
                    continue
                seen.add((nx, ny))
                if 0 <= nx < w and 0 <= ny < h and traversable[ny, nx]:
                    return (nx, ny)
                queue.append(((nx, ny), d + 1))
    return None


def reachable_mask(traversable: np.ndarray, start_cell: Cell,
                   snap_radius: int = 7) -> np.ndarray:
    """
    Connected component of the traversable mask that contains ``start_cell``.

    If the start cell is not traversable (inside the inflation zone) it is first snapped to
    the nearest traversable cell. Returns an all-False mask when no cell is found.
    Connectivity is 4-neighbour, which equals the 8-connected A* without corner cutting.
    """
    snapped = snap_to_traversable(traversable, start_cell, snap_radius)
    if snapped is None:
        return np.zeros_like(traversable, dtype=bool)
    labels, _ = ndimage.label(traversable)
    return labels == labels[snapped[1], snapped[0]]


_SQRT2 = math.sqrt(2.0)


def astar(traversable: np.ndarray, start: Cell, goal: Cell) -> Optional[List[Cell]]:
    """
    8-connected A* with an octile heuristic and no corner cutting (a diagonal move needs
    both orthogonal neighbours to be traversable). Returns the list of cells from start to
    goal (inclusive) or None.
    """
    h, w = traversable.shape
    sx, sy = start
    gx, gy = goal
    for x, y in (start, goal):
        if not (0 <= x < w and 0 <= y < h) or not traversable[y, x]:
            return None
    if start == goal:
        return [start]

    free = traversable.ravel().tolist()

    def heuristic(x, y):
        dx = abs(x - gx)
        dy = abs(y - gy)
        return (dx + dy) + (_SQRT2 - 2.0) * min(dx, dy)

    start_i = sy * w + sx
    goal_i = gy * w + gx
    g_score: Dict[int, float] = {start_i: 0.0}
    parent: Dict[int, int] = {}
    closed = set()
    heap = [(heuristic(sx, sy), 0.0, start_i)]
    moves = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
             (-1, -1, _SQRT2), (1, -1, _SQRT2), (-1, 1, _SQRT2), (1, 1, _SQRT2)]
    while heap:
        _, g, cur = heapq.heappop(heap)
        if cur in closed:
            continue
        if cur == goal_i:
            path = [cur]
            while cur in parent:
                cur = parent[cur]
                path.append(cur)
            path.reverse()
            return [(i % w, i // w) for i in path]
        closed.add(cur)
        x = cur % w
        y = cur // w
        for dx, dy, cost in moves:
            nx = x + dx
            ny = y + dy
            if nx < 0 or ny < 0 or nx >= w or ny >= h:
                continue
            ni = ny * w + nx
            if not free[ni] or ni in closed:
                continue
            if dx != 0 and dy != 0 and not (free[y * w + nx] and free[ny * w + x]):
                continue
            ng = g + cost
            if ng < g_score.get(ni, math.inf):
                g_score[ni] = ng
                parent[ni] = cur
                heapq.heappush(heap, (ng + heuristic(nx, ny), ng, ni))
    return None


def select_goal(frontiers: List[Frontier], traversable: np.ndarray, reachable: np.ndarray,
                robot_cell: Cell, blacklist: List[Cell], resolution: float = 0.05,
                size_weight: float = 0.5, top_k: int = 5, search_radius: float = 1.0,
                blacklist_radius: float = 0.5, prefer_goal: Optional[Cell] = None,
                prefer_bonus: float = 1.0, start_cell: Optional[Cell] = None
                ) -> Optional[Tuple[Cell, List[Cell]]]:
    """
    Pick the best reachable frontier and plan a path to it.

    For every cluster the goal is the reachable traversable cell closest to the centroid
    (within ``search_radius`` meters). Clusters are ranked by
    ``distance - size_weight * size`` (meters), the top ``top_k`` are planned with A* until
    one succeeds. Goals within ``blacklist_radius`` of a blacklisted cell are skipped.
    ``start_cell`` is the (snapped) traversable start of the path, default ``robot_cell``.
    """
    if start_cell is None:
        start_cell = snap_to_traversable(traversable, robot_cell) or robot_cell
    h, w = reachable.shape
    r_cells = max(1, int(round(search_radius / resolution)))
    bl_cells = blacklist_radius / resolution
    candidates = []
    for fr in frontiers:
        cx, cy = fr.centroid_cell
        x0, x1 = max(0, cx - r_cells), min(w, cx + r_cells + 1)
        y0, y1 = max(0, cy - r_cells), min(h, cy + r_cells + 1)
        ys, xs = np.nonzero(reachable[y0:y1, x0:x1])
        if len(xs) == 0:
            continue
        xs = xs + x0
        ys = ys + y0
        d2 = (xs - cx) ** 2 + (ys - cy) ** 2
        order = np.argsort(d2)
        goal = None
        for k in order:
            if d2[k] > r_cells * r_cells:
                break
            g = (int(xs[k]), int(ys[k]))
            if any(math.hypot(g[0] - b[0], g[1] - b[1]) <= bl_cells for b in blacklist):
                continue
            goal = g
            break
        if goal is None:
            continue
        dist_m = math.hypot(goal[0] - robot_cell[0], goal[1] - robot_cell[1]) * resolution
        score = dist_m - size_weight * fr.size * resolution
        if prefer_goal is not None and math.hypot(
                goal[0] - prefer_goal[0], goal[1] - prefer_goal[1]) * resolution <= search_radius:
            score -= prefer_bonus
        candidates.append((score, goal))
    candidates.sort(key=lambda c: c[0])
    for _, goal in candidates[:top_k]:
        path = astar(traversable, start_cell, goal)
        if path is not None:
            return goal, path
    return None


def simplify_path(path: List[Cell], grid_info: GridInfo,
                  spacing: float = 0.1) -> List[Tuple[float, float]]:
    """Convert a cell path to world points, keeping roughly ``spacing`` m between them."""
    if not path:
        return []
    pts = [grid_info.cell_to_world(*c) for c in path]
    out = [pts[0]]
    for p in pts[1:-1]:
        if math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) >= spacing:
            out.append(p)
    if len(pts) > 1:
        out.append(pts[-1])
    return out


# ---------------------------------------------------------------------------
# Path following
# ---------------------------------------------------------------------------

class PurePursuit:
    """Pure-pursuit follower with rotate-in-place for large heading errors."""

    def __init__(self, max_linear_speed=0.15, max_angular_speed=0.8, lookahead_distance=0.35,
                 rotate_in_place_angle=0.8, goal_tolerance=0.25):
        self.max_v = max_linear_speed
        self.max_w = max_angular_speed
        self.lookahead = lookahead_distance
        self.rotate_angle = rotate_in_place_angle
        self.goal_tolerance = goal_tolerance

    def compute(self, pose, path) -> Tuple[float, float, bool]:
        """pose = (x, y, yaw), path = list of (x, y). Returns (v, w, done)."""
        if not path:
            return 0.0, 0.0, True
        x, y, yaw = pose
        gx, gy = path[-1]
        if math.hypot(gx - x, gy - y) <= self.goal_tolerance:
            return 0.0, 0.0, True

        # Nearest path point, then the first point at lookahead distance after it.
        nearest = min(range(len(path)), key=lambda i: math.hypot(path[i][0] - x, path[i][1] - y))
        target = path[-1]
        for i in range(nearest, len(path)):
            if math.hypot(path[i][0] - x, path[i][1] - y) >= self.lookahead:
                target = path[i]
                break
        dx = target[0] - x
        dy = target[1] - y
        alpha = wrap_angle(math.atan2(dy, dx) - yaw)

        if abs(alpha) > self.rotate_angle:
            w = float(np.clip(2.0 * alpha, -self.max_w, self.max_w))
            if abs(w) < 0.2:
                w = math.copysign(0.2, alpha)
            return 0.0, w, False

        dist = max(math.hypot(dx, dy), 1e-3)
        v = self.max_v * max(0.3, math.cos(alpha))
        kappa = 2.0 * math.sin(alpha) / max(dist, 0.1)
        w = v * kappa
        if abs(w) > self.max_w:
            v *= self.max_w / abs(w)
            w = math.copysign(self.max_w, w)
        v = min(v, self.max_v)
        return float(v), float(w), False


# ---------------------------------------------------------------------------
# Reactive safety
# ---------------------------------------------------------------------------

def scan_safety(ranges, angle_min, angle_increment, laser_yaw_offset,
                stop_distance=0.25, slow_distance=0.45, front_half_angle=0.6,
                rear_half_angle=0.6):
    """
    Evaluate a laser scan in the robot base frame.

    Beam angles in the base frame are ``wrap(angle_min + i * angle_increment +
    laser_yaw_offset)``; the front of the robot is base angle 0.

    Returns ``(scale, blocked, min_all, escape_dir)``:
      * scale: forward speed factor in [0, 1] from the front-sector minimum (0 below
        ``stop_distance``, 1 above ``slow_distance``, linear in between);
      * blocked: True when the front-sector minimum is below ``stop_distance``;
      * min_all: minimum valid range over all beams (inf if none);
      * escape_dir: ``(lin, ang)`` recovery direction, lin in {0, -1} (-1 = reverse, only
        when the rear is clear) and ang in {-1, 0, +1} (+1 = turn left, towards the freer
        side). (0, 0) when not blocked.
    """
    r = np.asarray(ranges, dtype=float)
    if r.size == 0:
        return 1.0, False, math.inf, (0, 0)
    angles = wrap_angle(angle_min + np.arange(r.size) * angle_increment + laser_yaw_offset)
    valid = np.isfinite(r) & (r > 0.02)
    # +inf means "no return": treat as free space, but keep it out of the minimum.
    valid_ranges = np.where(valid, r, np.inf)
    front = np.abs(angles) <= front_half_angle
    front_min = float(valid_ranges[front].min()) if front.any() else math.inf
    min_all = float(valid_ranges.min())

    if front_min >= slow_distance:
        scale = 1.0
    elif front_min < stop_distance:
        scale = 0.0
    else:
        scale = (front_min - stop_distance) / (slow_distance - stop_distance)
    blocked = front_min < stop_distance

    escape = (0, 0)
    if blocked:
        left = angles > 0.0
        right = angles < 0.0
        left_min = float(valid_ranges[left].min()) if left.any() else math.inf
        right_min = float(valid_ranges[right].min()) if right.any() else math.inf
        ang = 1 if left_min >= right_min else -1
        rear = np.abs(angles) >= math.pi - rear_half_angle
        rear_min = float(valid_ranges[rear].min()) if rear.any() else math.inf
        lin = -1 if rear_min > slow_distance else 0
        escape = (lin, ang)
    return scale, bool(blocked), min_all, escape


class CompletionMonitor:
    """True only after N consecutive empty updates AND a minimum time without frontiers."""

    def __init__(self, hysteresis=3, timeout=10.0):
        self.hysteresis = hysteresis
        self.timeout = timeout
        self.reset()

    def reset(self):
        self.count = 0
        self.first_empty_t = None

    def update(self, has_frontier: bool, t: float) -> bool:
        if has_frontier:
            self.reset()
            return False
        if self.first_empty_t is None:
            self.first_empty_t = t
        self.count += 1
        return self.count >= self.hysteresis and (t - self.first_empty_t) >= self.timeout


# ---------------------------------------------------------------------------
# State machine
# ---------------------------------------------------------------------------

DEFAULT_PARAMS = {
    "robot_radius": 0.12,
    "inflation_radius": 0.30,
    "unknown_is_obstacle": True,
    "min_frontier_size": 8,
    "goal_blacklist_radius": 0.5,
    "progress_timeout": 20.0,
    "max_linear_speed": 0.15,
    "max_angular_speed": 0.8,
    "lookahead_distance": 0.35,
    "rotate_in_place_angle": 0.8,
    "goal_tolerance": 0.25,
    "stop_distance": 0.25,
    "slow_distance": 0.45,
    "front_sector_half_angle": 0.6,
    "initial_spin": True,
    "completion_hysteresis": 3,
    "completion_timeout": 10.0,
    "max_exploration_time": 0.0,
    "replan_period": 2.0,
    "frontier_size_weight": 0.5,
}


@dataclass
class ScanData:
    ranges: np.ndarray
    angle_min: float
    angle_increment: float


@dataclass
class Command:
    v: float
    w: float
    state: str
    path: List[Tuple[float, float]] = field(default_factory=list)
    frontiers: List[Frontier] = field(default_factory=list)
    goal: Optional[Tuple[float, float]] = None
    timed_out: bool = False


class ExplorerCore:
    """
    Frontier exploration state machine:
    WAITING_FOR_MAP -> INITIAL_SPIN -> EXPLORING -> COMPLETE.

    ``laser_yaw_offset`` (yaw of the laser frame in the base frame) must be set by the
    caller from TF or the simulation model.
    """

    SPIN_RATE = 0.5          # rad/s
    SPIN_TIMEOUT = 40.0      # s
    RECOVERY_W = 0.5         # rad/s
    RECOVERY_REVERSE = -0.05  # m/s
    BLOCKED_REPLAN_TIME = 3.0  # s blocked before the goal is blacklisted

    def __init__(self, params=None, laser_yaw_offset=0.0):
        self.p = dict(DEFAULT_PARAMS)
        if params:
            self.p.update(params)
        self.laser_yaw_offset = laser_yaw_offset
        self.pursuit = PurePursuit(
            self.p["max_linear_speed"], self.p["max_angular_speed"],
            self.p["lookahead_distance"], self.p["rotate_in_place_angle"],
            self.p["goal_tolerance"])
        self.monitor = CompletionMonitor(self.p["completion_hysteresis"],
                                         self.p["completion_timeout"])
        self.state = WAITING_FOR_MAP
        self.grid: Optional[np.ndarray] = None
        self.info: Optional[GridInfo] = None
        self.replan_due = False
        self.last_plan_t = -math.inf
        self.start_t: Optional[float] = None
        self.timed_out = False
        # Current goal
        self.path: List[Tuple[float, float]] = []
        self.goal_cell: Optional[Cell] = None
        self.blacklist: List[Cell] = []
        self.frontiers: List[Frontier] = []
        self.progress_pos = None
        self.progress_t = 0.0
        # Initial spin
        self.spin_yaw = None
        self.spin_acc = 0.0
        self.spin_start_t = 0.0
        # Blocked recovery
        self.blocked_since: Optional[float] = None

    # -- inputs -----------------------------------------------------------
    def update_map(self, grid: np.ndarray, info: GridInfo, t: float):
        self.grid = grid
        self.info = info
        self.replan_due = True

    # -- helpers ----------------------------------------------------------
    def _zero(self, state=None):
        return Command(0.0, 0.0, state or self.state, [], self.frontiers, None, self.timed_out)

    def _goal_world(self):
        if self.goal_cell is None or self.info is None:
            return None
        return self.info.cell_to_world(*self.goal_cell)

    def _command(self, v, w):
        return Command(v, w, self.state, list(self.path), self.frontiers,
                       self._goal_world(), self.timed_out)

    def _blacklist_goal(self):
        if self.goal_cell is not None:
            self.blacklist.append(self.goal_cell)
        self.goal_cell = None
        self.path = []
        self.replan_due = True
        self.last_plan_t = -math.inf

    def _replan(self, pose, t) -> bool:
        """Replan on the latest map. Returns True when a goal was found."""
        self.last_plan_t = t
        self.replan_due = False
        info = self.info
        res = info.resolution
        grid = self.grid
        inflation_cells = self.p["inflation_radius"] / res
        trav = build_costmap(grid, inflation_cells, self.p["unknown_is_obstacle"])
        self.frontiers = cluster_frontiers(detect_frontiers(grid), self.p["min_frontier_size"])
        robot_cell = info.world_to_cell(pose[0], pose[1])
        start = snap_to_traversable(trav, robot_cell, int(inflation_cells) + 1)
        if start is None:
            # No traversable start: this says nothing about the frontiers, so do not count
            # it for the completion monitor.
            self.goal_cell = None
            self.path = []
            return False
        reach = reachable_mask(trav, start)
        result = select_goal(
            self.frontiers, trav, reach, robot_cell, self.blacklist, resolution=res,
            size_weight=self.p["frontier_size_weight"],
            blacklist_radius=self.p["goal_blacklist_radius"],
            prefer_goal=self.goal_cell, start_cell=start)
        if result is None:
            self.goal_cell = None
            self.path = []
            if self.monitor.update(False, t):
                self.state = COMPLETE
            return False
        self.monitor.update(True, t)
        goal, cells = result
        if goal != self.goal_cell:
            self.progress_pos = (pose[0], pose[1])
            self.progress_t = t
        self.goal_cell = goal
        self.path = simplify_path(cells, info, 0.1)
        return True

    # -- main step --------------------------------------------------------
    def step(self, pose, scan: Optional[ScanData], t: float) -> Command:
        """pose = (x, y, yaw) in the map frame. Returns the velocity command."""
        if self.start_t is None:
            self.start_t = t
        if self.state == COMPLETE:
            return self._zero()
        if self.grid is None:
            self.state = WAITING_FOR_MAP
            return self._zero()
        if scan is None:
            return self._zero()

        max_t = self.p["max_exploration_time"]
        if max_t > 0.0 and t - self.start_t >= max_t:
            self.state = COMPLETE
            self.timed_out = True
            return self._zero()

        scale, blocked, _, escape = scan_safety(
            scan.ranges, scan.angle_min, scan.angle_increment, self.laser_yaw_offset,
            self.p["stop_distance"], self.p["slow_distance"], self.p["front_sector_half_angle"])

        if self.state == WAITING_FOR_MAP:
            if self.p["initial_spin"]:
                self.state = INITIAL_SPIN
                self.spin_yaw = pose[2]
                self.spin_acc = 0.0
                self.spin_start_t = t
            else:
                self.state = EXPLORING

        if self.state == INITIAL_SPIN:
            self.spin_acc += abs(wrap_angle(pose[2] - self.spin_yaw))
            self.spin_yaw = pose[2]
            if self.spin_acc >= 2.0 * math.pi - 0.2 or t - self.spin_start_t > self.SPIN_TIMEOUT:
                self.state = EXPLORING
                self.replan_due = True
            else:
                return self._command(0.0, self.SPIN_RATE)

        # EXPLORING
        need_replan = False
        if self.goal_cell is None or not self.path:
            need_replan = (t - self.last_plan_t) >= self.p["replan_period"] or self.replan_due
        elif self.replan_due:
            need_replan = True
        if need_replan:
            self._replan(pose, t)
            if self.state == COMPLETE:
                return self._zero()

        if self.goal_cell is None or not self.path:
            # Nothing to follow; use the safety layer to get out of tight spots.
            if blocked:
                return self._command(self.RECOVERY_REVERSE if escape[0] < 0 else 0.0,
                                     self.RECOVERY_W * escape[1])
            return self._command(0.0, 0.0)

        v, w, done = self.pursuit.compute(pose, self.path)
        if done:
            self._blacklist_goal()
            return self._command(0.0, 0.0)

        # Progress timeout
        if self.progress_pos is None or math.hypot(pose[0] - self.progress_pos[0],
                                                   pose[1] - self.progress_pos[1]) >= 0.1:
            self.progress_pos = (pose[0], pose[1])
            self.progress_t = t
        elif t - self.progress_t > self.p["progress_timeout"]:
            self._blacklist_goal()
            self.progress_t = t
            return self._command(0.0, 0.0)

        # Reactive safety on top of the follower output
        if blocked:
            if self.blocked_since is None:
                self.blocked_since = t
            elif t - self.blocked_since > self.BLOCKED_REPLAN_TIME:
                self.blocked_since = None
                self._blacklist_goal()
                return self._command(0.0, 0.0)
            v_out = self.RECOVERY_REVERSE if escape[0] < 0 else 0.0
            return self._command(v_out, self.RECOVERY_W * escape[1])
        self.blocked_since = None
        v = v * scale
        return self._command(v, w)
