"""
Occupancy map comparison helper (pure Python, numpy + scipy, no rclpy, no cv2).

Used by the SLAM map quality test (AC9) to decide whether a map saved by slam_toolbox agrees
with a reference map (same world) up to a rigid 2D transform, and whether it contains local
distortions: a fan (walls rotated differently in different places) or a seam (a part of the map
shifted or rotated relative to the rest).

Pipeline of ``compare_maps``:
  1. ``rigid_align``: global rotation + translation of the explored occupied cells onto the
     reference walls (truncated chamfer distance, coarse-to-fine search).
  2. Global metrics: inlier fraction, median distance, residual dominant wall direction.
  3. ``tile_residuals``: the aligned map is cut into overlapping tiles and every tile gets its own
     small rigid search. A tile that needs a large extra rotation or shift, or that matches badly
     before the search, reveals a local distortion that the global alignment cannot absorb.

Conventions: world coordinates in meters, angles in degrees at the public API, map images are
returned with row 0 = minimum y (the PGM file stores the top row first, so it is flipped).
"""

import math
import os
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np
import yaml
from scipy import ndimage

# Pixel classes of a trinary map_server image
OCCUPIED_PIXEL_MAX = 50
FREE_PIXEL = 254
UNKNOWN_PIXEL = 205

OUTSIDE_DISTANCE = 5.0     # distance returned for points outside the reference grid [m]
TRUNCATION = 0.5           # truncation of the chamfer cost [m]
INLIER_DISTANCE = 0.15     # occupied cell counts as matching a wall within this distance [m]

# Thresholds of the AC9 test. See doc/08 for how they were chosen.
DEFAULT_THRESHOLDS = {
    "min_occupied": 2000,
    "min_extent_long_m": 15.0,
    "min_extent_short_m": 9.0,
    "min_inlier_fraction": 0.80,
    "max_dominant_residual_deg": 2.0,
    "max_tile_rotation_deg": 2.0,
    "max_tile_shift_m": 0.20,
    "min_tile_inlier_fraction": 0.50,
    "min_tiles": 1,
}


# ---------------------------------------------------------------------------
# Loading
# ---------------------------------------------------------------------------

@dataclass
class MapImage:
    grid: np.ndarray          # uint8 pixels, shape (H, W), row 0 = min y
    resolution: float
    origin_x: float
    origin_y: float


def read_pgm(path: str) -> np.ndarray:
    """Read a binary (P5, 8-bit) PGM. Returns the pixels with the top row first, as stored."""
    with open(path, "rb") as f:
        data = f.read()
    pos = 0
    tokens: List[bytes] = []
    while len(tokens) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b"#":
            while data[pos:pos + 1] not in (b"\n", b""):
                pos += 1
            continue
        start = pos
        while not data[pos:pos + 1].isspace():
            pos += 1
        tokens.append(data[start:pos])
    pos += 1  # single whitespace after maxval
    if tokens[0] != b"P5":
        raise ValueError("%s: only binary P5 PGM files are supported" % path)
    width, height, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    if maxval > 255:
        raise ValueError("%s: only 8-bit PGM files are supported" % path)
    return np.frombuffer(data, dtype=np.uint8, count=width * height, offset=pos).reshape(height, width)


def load_map(yaml_path: str) -> MapImage:
    """Load a map_server YAML + PGM pair. The image path is relative to the YAML file."""
    with open(yaml_path, "r") as f:
        meta = yaml.safe_load(f)
    image_path = meta["image"]
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), image_path)
    pixels = read_pgm(image_path)
    if int(meta.get("negate", 0)):
        pixels = 255 - pixels
    return MapImage(grid=np.ascontiguousarray(pixels[::-1]),
                    resolution=float(meta["resolution"]),
                    origin_x=float(meta["origin"][0]),
                    origin_y=float(meta["origin"][1]))


def occupied_points(m: MapImage) -> np.ndarray:
    """World coordinates (N, 2) of the centres of the occupied cells."""
    rows, cols = np.nonzero(m.grid < OCCUPIED_PIXEL_MAX)
    return np.column_stack((m.origin_x + (cols + 0.5) * m.resolution,
                            m.origin_y + (rows + 0.5) * m.resolution))


# ---------------------------------------------------------------------------
# Distance field and rigid search
# ---------------------------------------------------------------------------

def reference_distance_field(ref: MapImage) -> Callable[[np.ndarray], np.ndarray]:
    """Return ``f(points (N, 2)) -> distances (N,)`` to the nearest occupied reference cell."""
    occupied = ref.grid < OCCUPIED_PIXEL_MAX
    dist = ndimage.distance_transform_edt(~occupied) * ref.resolution
    height, width = dist.shape

    def lookup(points: np.ndarray) -> np.ndarray:
        col = np.floor((points[..., 0] - ref.origin_x) / ref.resolution).astype(np.int64)
        row = np.floor((points[..., 1] - ref.origin_y) / ref.resolution).astype(np.int64)
        inside = (col >= 0) & (col < width) & (row >= 0) & (row < height)
        out = dist[np.clip(row, 0, height - 1), np.clip(col, 0, width - 1)]
        return np.where(inside, out, OUTSIDE_DISTANCE)

    return lookup


def _transform(points: np.ndarray, center: np.ndarray, theta: float, tx: float, ty: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    rel = points - center
    return np.column_stack((c * rel[:, 0] - s * rel[:, 1], s * rel[:, 0] + c * rel[:, 1])) + center + (tx, ty)


def _search(points, dist_fn, center, thetas, txs, tys, regulariser=0.0):
    """Exhaustive search of the truncated chamfer cost. Returns (theta, tx, ty, cost)."""
    shifts = np.array([(x, y) for x in txs for y in tys])
    best = (0.0, 0.0, 0.0, float("inf"))
    best_score = float("inf")
    rel = points - center
    for theta in thetas:
        c, s = math.cos(theta), math.sin(theta)
        rot = np.column_stack((c * rel[:, 0] - s * rel[:, 1], s * rel[:, 0] + c * rel[:, 1])) + center
        # (S, N, 2) is chunked to bound the memory
        for i in range(0, len(shifts), 64):
            chunk = shifts[i:i + 64]
            cand = rot[None, :, :] + chunk[:, None, :]
            cost = np.minimum(dist_fn(cand), TRUNCATION).mean(axis=1)
            if regulariser:
                score = cost + regulariser * (abs(math.degrees(theta)) + 10.0 * np.hypot(chunk[:, 0], chunk[:, 1]))
            else:
                score = cost
            j = int(np.argmin(score))
            if score[j] < best_score:
                best_score = float(score[j])
                best = (float(theta), float(chunk[j, 0]), float(chunk[j, 1]), float(cost[j]))
    return best


def _subsample(points: np.ndarray, n: int) -> np.ndarray:
    if len(points) <= n:
        return points
    idx = np.random.default_rng(0).choice(len(points), n, replace=False)
    return points[idx]


def rigid_align(points: np.ndarray, dist_fn, max_angle_deg: float = 20.0, max_shift: float = 1.5
                ) -> Tuple[float, float, float, float]:
    """
    Find the rotation (about the centroid of ``points``) and translation that put ``points`` on
    the walls described by ``dist_fn``. Returns ``(theta_rad, tx, ty, cost)``: ``points`` are mapped
    by ``p' = R(theta) (p - centroid) + centroid + (tx, ty)``.
    """
    sub = _subsample(points, 2000)
    center = points.mean(axis=0)
    rad = np.radians
    thetas = np.arange(-max_angle_deg, max_angle_deg + 1e-9, 1.0)
    thetas = rad(thetas)
    shifts = np.arange(-max_shift, max_shift + 1e-9, 0.2)
    theta, tx, ty, _ = _search(sub, dist_fn, center, thetas, shifts, shifts)
    theta, tx, ty, _ = _search(sub, dist_fn, center, theta + rad(np.arange(-1.0, 1.0 + 1e-9, 0.1)),
                               tx + np.arange(-0.2, 0.2 + 1e-9, 0.02), ty + np.arange(-0.2, 0.2 + 1e-9, 0.02))
    # Final cost on all the points
    cost = float(np.minimum(dist_fn(_transform(points, center, theta, tx, ty)), TRUNCATION).mean())
    return theta, tx, ty, cost


def apply_alignment(points: np.ndarray, theta: float, tx: float, ty: float) -> np.ndarray:
    """Apply the result of ``rigid_align`` to ``points`` (the same centroid convention)."""
    return _transform(points, points.mean(axis=0), theta, tx, ty)


# ---------------------------------------------------------------------------
# Wall direction and tiles
# ---------------------------------------------------------------------------

def dominant_direction_deg(points: np.ndarray) -> float:
    """
    Dominant wall direction in [-45, 45) degrees: the angle whose axis and normal give the sharpest
    histogram (0.05 m bins) of the point projections. A rotated square room gives its rotation.
    """
    best_angle, best_sharp = 0.0, -1.0
    for angle in np.arange(-45.0, 45.0, 0.25):
        a = math.radians(angle)
        sharp = 0.0
        for axis in ((math.cos(a), math.sin(a)), (-math.sin(a), math.cos(a))):
            proj = points @ np.array(axis)
            bins = np.floor((proj - proj.min()) / 0.05).astype(np.int64)
            counts = np.bincount(bins)
            sharp += float(np.sum(counts.astype(np.float64) ** 2))
        if sharp > best_sharp:
            best_sharp, best_angle = sharp, float(angle)
    return best_angle


@dataclass
class TileResidual:
    center: Tuple[float, float]
    n_points: int
    rotation_deg: float       # extra rotation the tile needs to match the reference
    shift_m: float            # extra shift the tile needs
    inlier_fraction: float    # before the local search (after the global alignment)


def tile_residuals(aligned_points: np.ndarray, dist_fn, tile: float = 4.0, stride: float = 2.0,
                   min_points: int = 150) -> List[TileResidual]:
    """Local rigid search per tile (tiles with fewer than ``min_points`` cells are skipped)."""
    result: List[TileResidual] = []
    lo = aligned_points.min(axis=0)
    hi = aligned_points.max(axis=0)
    xs = np.arange(lo[0], max(hi[0] - tile, lo[0]) + stride, stride)
    ys = np.arange(lo[1], max(hi[1] - tile, lo[1]) + stride, stride)
    for x0 in xs:
        for y0 in ys:
            mask = ((aligned_points[:, 0] >= x0) & (aligned_points[:, 0] < x0 + tile)
                    & (aligned_points[:, 1] >= y0) & (aligned_points[:, 1] < y0 + tile))
            pts = aligned_points[mask]
            if len(pts) < min_points:
                continue
            inliers = float(np.mean(dist_fn(pts) <= INLIER_DISTANCE))
            sub = _subsample(pts, 600)
            center = sub.mean(axis=0)
            rad = np.radians
            # Coarse: +-15 deg at 1 deg, +-0.4 m at 0.1 m. Fine: 0.25 deg / 0.05 m around the best.
            theta, tx, ty, _ = _search(sub, dist_fn, center, rad(np.arange(-15.0, 15.0 + 1e-9, 1.0)),
                                       np.arange(-0.4, 0.4 + 1e-9, 0.1), np.arange(-0.4, 0.4 + 1e-9, 0.1),
                                       regulariser=1e-4)
            theta, tx, ty, _ = _search(sub, dist_fn, center, theta + rad(np.arange(-1.0, 1.0 + 1e-9, 0.25)),
                                       tx + np.arange(-0.1, 0.1 + 1e-9, 0.05),
                                       ty + np.arange(-0.1, 0.1 + 1e-9, 0.05), regulariser=1e-4)
            result.append(TileResidual(center=(float(x0 + tile / 2), float(y0 + tile / 2)),
                                       n_points=int(len(pts)),
                                       rotation_deg=math.degrees(theta),
                                       shift_m=float(math.hypot(tx, ty)),
                                       inlier_fraction=inliers))
    return result


# ---------------------------------------------------------------------------
# Comparison
# ---------------------------------------------------------------------------

@dataclass
class MapQuality:
    n_occupied: int
    extent_m: Tuple[float, float]      # (long side, short side) of the occupied bounding box
    global_rotation_deg: float         # rotation of the explored map relative to the reference
    global_shift_m: float              # translation needed to align it
    inlier_fraction: float             # occupied cells within INLIER_DISTANCE of a reference wall
    median_distance: float
    dominant_residual_deg: float       # wall direction of the aligned map minus the reference's
    max_tile_rotation_deg: float
    max_tile_shift_m: float
    min_tile_inlier_fraction: float
    n_tiles: int
    tiles: List[TileResidual] = field(default_factory=list, repr=False)

    def passes(self, thresholds: Optional[Dict[str, float]] = None) -> Tuple[bool, List[str]]:
        """Check the metrics against ``thresholds`` (default ``DEFAULT_THRESHOLDS``)."""
        t = dict(DEFAULT_THRESHOLDS)
        t.update(thresholds or {})
        reasons = []
        if self.n_occupied < t["min_occupied"]:
            reasons.append("n_occupied %d < %d" % (self.n_occupied, t["min_occupied"]))
        if self.extent_m[0] < t["min_extent_long_m"] or self.extent_m[1] < t["min_extent_short_m"]:
            reasons.append("extent %.1f x %.1f m < %.1f x %.1f m" % (
                self.extent_m[0], self.extent_m[1], t["min_extent_long_m"], t["min_extent_short_m"]))
        if self.inlier_fraction < t["min_inlier_fraction"]:
            reasons.append("inlier_fraction %.2f < %.2f" % (self.inlier_fraction, t["min_inlier_fraction"]))
        if abs(self.dominant_residual_deg) > t["max_dominant_residual_deg"]:
            reasons.append("dominant_residual_deg %.2f > %.2f" % (
                abs(self.dominant_residual_deg), t["max_dominant_residual_deg"]))
        if self.n_tiles < t["min_tiles"]:
            reasons.append("n_tiles %d < %d" % (self.n_tiles, t["min_tiles"]))
        if abs(self.max_tile_rotation_deg) > t["max_tile_rotation_deg"]:
            reasons.append("max_tile_rotation_deg %.2f > %.2f" % (
                abs(self.max_tile_rotation_deg), t["max_tile_rotation_deg"]))
        if self.max_tile_shift_m > t["max_tile_shift_m"]:
            reasons.append("max_tile_shift_m %.2f > %.2f" % (self.max_tile_shift_m, t["max_tile_shift_m"]))
        if self.min_tile_inlier_fraction < t["min_tile_inlier_fraction"]:
            reasons.append("min_tile_inlier_fraction %.2f < %.2f" % (
                self.min_tile_inlier_fraction, t["min_tile_inlier_fraction"]))
        return (not reasons), reasons


def _as_map(m) -> MapImage:
    return m if isinstance(m, MapImage) else load_map(m)


def compare_maps(explored, reference) -> MapQuality:
    """
    Compare two maps (``MapImage`` or map_server YAML paths) up to a rigid 2D transform.
    ``global_rotation_deg`` is the rotation of ``explored`` relative to ``reference``.
    """
    explored_map, ref_map = _as_map(explored), _as_map(reference)
    points = occupied_points(explored_map)
    if len(points) < 10:
        return MapQuality(len(points), (0.0, 0.0), 0.0, 0.0, 0.0, OUTSIDE_DISTANCE, 90.0,
                          90.0, OUTSIDE_DISTANCE, 0.0, 0)
    dist_fn = reference_distance_field(ref_map)
    theta, tx, ty, _ = rigid_align(points, dist_fn)
    aligned = apply_alignment(points, theta, tx, ty)
    dist = dist_fn(aligned)

    size = points.max(axis=0) - points.min(axis=0)
    ref_dir = dominant_direction_deg(occupied_points(ref_map))
    residual = dominant_direction_deg(aligned) - ref_dir
    residual = (residual + 45.0) % 90.0 - 45.0

    tiles = tile_residuals(aligned, dist_fn)
    return MapQuality(
        n_occupied=int(len(points)),
        extent_m=(float(max(size)), float(min(size))),
        global_rotation_deg=-math.degrees(theta),
        global_shift_m=float(math.hypot(tx, ty)),
        inlier_fraction=float(np.mean(dist <= INLIER_DISTANCE)),
        median_distance=float(np.median(dist)),
        dominant_residual_deg=float(residual),
        max_tile_rotation_deg=max((abs(t.rotation_deg) for t in tiles), default=0.0),
        max_tile_shift_m=max((t.shift_m for t in tiles), default=0.0),
        min_tile_inlier_fraction=min((t.inlier_fraction for t in tiles), default=0.0),
        n_tiles=len(tiles),
        tiles=tiles,
    )
