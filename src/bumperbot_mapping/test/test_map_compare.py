"""AC9 helper tests: map_compare.compare_maps on perturbed, seamed and warped maps."""
import math
import os
import time

import numpy as np
import pytest

from bumperbot_mapping import map_compare as mc

HERE = os.path.dirname(os.path.abspath(__file__))
REF_YAML = os.path.join(HERE, "..", "maps", "small_house", "map.yaml")
WARPED_YAML = os.path.join(HERE, "data", "warped_small_house.yaml")


@pytest.fixture(scope="module")
def ref():
    return mc.load_map(REF_YAML)


def make_map(ref, points, name_res=None):
    """Rasterise world points into a MapImage on the reference grid (occupied = 0, unknown = 205)."""
    grid = np.full(ref.grid.shape, mc.UNKNOWN_PIXEL, dtype=np.uint8)
    col = np.floor((points[:, 0] - ref.origin_x) / ref.resolution).astype(int)
    row = np.floor((points[:, 1] - ref.origin_y) / ref.resolution).astype(int)
    ok = (col >= 0) & (col < grid.shape[1]) & (row >= 0) & (row < grid.shape[0])
    grid[row[ok], col[ok]] = 0
    return mc.MapImage(grid, ref.resolution, ref.origin_x, ref.origin_y)


def rotate(points, deg, center, shift=(0.0, 0.0)):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    rel = points - center
    return np.column_stack((c * rel[:, 0] - s * rel[:, 1], s * rel[:, 0] + c * rel[:, 1])) + center + shift


def test_loader_reads_reference(ref):
    pts = mc.occupied_points(ref)
    assert len(pts) > 2000
    assert ref.resolution == pytest.approx(0.05)


def test_perturbed_reference_passes(ref):
    rng = np.random.default_rng(42)
    pts = mc.occupied_points(ref)
    center = pts.mean(axis=0)
    applied = 7.0
    moved = rotate(pts, applied, center, shift=(0.6, -0.8))
    moved = moved[rng.random(len(moved)) > 0.3]
    moved = moved + rng.normal(0.0, 0.03, moved.shape)
    explored = make_map(ref, moved)
    t0 = time.time()
    q = mc.compare_maps(explored, ref)
    dt = time.time() - t0
    print("\nperturbed:", q, "time %.1f s" % dt)
    ok, reasons = q.passes()
    assert ok, reasons
    assert q.global_rotation_deg == pytest.approx(applied, abs=0.3)
    assert dt <= 30.0


def test_synthetic_seam_fails(ref):
    pts = mc.occupied_points(ref)
    right = pts[:, 0] > -2.3
    seam = pts.copy()
    seam[right] = rotate(pts[right], 8.0, np.array([-2.3, 0.0]))
    q = mc.compare_maps(make_map(ref, seam), ref)
    print("\nseam:", q)
    ok, reasons = q.passes()
    assert not ok
    assert q.max_tile_rotation_deg > 4.0


def test_warped_fixture_fails(ref):
    t0 = time.time()
    q = mc.compare_maps(WARPED_YAML, ref)
    dt = time.time() - t0
    ok, reasons = q.passes()
    print("\nwarped:", q, "\nreasons:", reasons, "time %.1f s" % dt)
    assert not ok
    joined = " | ".join(reasons)
    assert "inlier_fraction" in joined
    assert "max_tile_rotation_deg" in joined
    assert "max_tile_shift_m" in joined or "min_tile_inlier_fraction" in joined
    assert dt <= 30.0


def test_reference_vs_itself_passes(ref):
    q = mc.compare_maps(ref, ref)
    ok, reasons = q.passes()
    assert ok, reasons
    assert q.inlier_fraction > 0.99
