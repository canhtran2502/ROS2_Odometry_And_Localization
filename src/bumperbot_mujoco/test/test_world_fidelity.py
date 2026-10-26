"""AC11: the MuJoCo small_house collision geometry matches the house (lidar-style ground truth)."""
import os
import time

import numpy as np
import pytest
from scipy import ndimage

mujoco = pytest.importorskip("mujoco")

from bumperbot_mujoco.sim_core import BumperbotSim, LIDAR_GEOM_GROUPS  # noqa: E402
from bumperbot_mapping import map_compare  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
MJCF = os.path.join(HERE, "..", "mjcf", "small_house.xml")
REF_YAML = os.path.join(HERE, "..", "..", "bumperbot_mapping", "maps", "small_house", "map.yaml")
WALL_BODY = "HouseWallB_01_001"
GRID_STEP = 0.4
N_RAYS = 1440
RANGE_MAX = 30.0
MIN_CLEARANCE = 0.3     # origins must be this far from reference walls


def _wall_geom_footprints(model, data):
    """Return a list of (geom_name, xy_area, extent_x, extent_y) for the collision geoms of the wall."""
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, WALL_BODY)
    assert body >= 0, "%s not in the model" % WALL_BODY
    from scipy.spatial import ConvexHull
    out = []
    for g in range(model.ngeom):
        if model.geom_bodyid[g] != body or model.geom_group[g] != 4:
            continue
        if model.geom_contype[g] == 0 and model.geom_conaffinity[g] == 0:
            continue
        rot = data.geom_xmat[g].reshape(3, 3)
        pos = data.geom_xpos[g]
        if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX:
            sx, sy, sz = model.geom_size[g]
            corners = np.array([[a * sx, b * sy, c * sz] for a in (-1, 1) for b in (-1, 1)
                                for c in (-1, 1)])
            pts = corners @ rot.T + pos
        elif model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
            m = model.geom_dataid[g]
            adr, n = model.mesh_vertadr[m], model.mesh_vertnum[m]
            pts = model.mesh_vert[adr:adr + n] @ rot.T + pos
        else:
            r = model.geom_rbound[g]
            pts = np.array([[pos[0] + dx, pos[1] + dy, pos[2]] for dx in (-r, r) for dy in (-r, r)])
        xy = pts[:, :2]
        ext = xy.max(axis=0) - xy.min(axis=0)
        try:
            area = float(ConvexHull(xy).volume)
        except Exception:  # noqa: BLE001  degenerate (zero width) footprint
            area = 0.0
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) or "geom%d" % g
        out.append((name, area, float(ext[0]), float(ext[1])))
    return out


def test_wall_collision_geoms_are_thin_segments():
    sim = BumperbotSim(MJCF)
    mujoco.mj_forward(sim.model, sim.data)
    geoms = _wall_geom_footprints(sim.model, sim.data)
    assert len(geoms) > 0, "no collision geoms found on the house wall body"
    slabs = [(n, a, ex, ey) for n, a, ex, ey in geoms if a > 4.0 and ex > 0.6 and ey > 0.6]
    print("wall collision geoms: %d, largest xy area %.2f m2" % (len(geoms), max(g[1] for g in geoms)))
    assert not slabs, "slab-like wall collision geoms (area > 4 m2, wider than 0.6 m both ways): %s" % slabs


def _ground_truth_map(ref):
    sim = BumperbotSim(MJCF)   # robot at the origin, like the real runs
    mujoco.mj_forward(sim.model, sim.data)
    model, data = sim.model, sim.data
    site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "laser_site")
    lidar_z = float(data.site_xpos[site][2])
    robot_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_footprint")

    free = ref.grid > 240
    clearance = ndimage.distance_transform_edt(ref.grid >= 50) * ref.resolution
    h, w = ref.grid.shape
    res = ref.resolution

    angles = np.linspace(0.0, 2.0 * np.pi, N_RAYS, endpoint=False)
    dirs = np.stack([np.cos(angles), np.sin(angles), np.zeros(N_RAYS)], axis=1).reshape(-1)
    gt = np.full((h, w), 205, dtype=np.uint8)
    geom_ids = np.zeros(N_RAYS, dtype=np.int32)
    dist = np.zeros(N_RAYS)
    normals = np.zeros(N_RAYS * 3)

    xs = np.arange(ref.origin_x + GRID_STEP / 2, ref.origin_x + w * res, GRID_STEP)
    ys = np.arange(ref.origin_y + GRID_STEP / 2, ref.origin_y + h * res, GRID_STEP)
    n_origins = 0
    for oy in ys:
        for ox in xs:
            ix, iy = int((ox - ref.origin_x) / res), int((oy - ref.origin_y) / res)
            if not free[iy, ix] or clearance[iy, ix] < MIN_CLEARANCE:
                continue
            # only origins in the part of the free space connected to the start would be ideal;
            # the reference free mask is restricted to the connected component below.
            if not REACH[iy, ix]:
                continue
            origin = np.array([ox, oy, lidar_z])
            mujoco.mj_multiRay(model, data, origin, dirs, LIDAR_GEOM_GROUPS, 1, robot_body,
                               geom_ids, dist, normals, N_RAYS, RANGE_MAX)
            n_origins += 1
            hit = dist > 0.0
            px = ox + dist[hit] * np.cos(angles[hit])
            py = oy + dist[hit] * np.sin(angles[hit])
            cx = np.floor((px - ref.origin_x) / res).astype(int)
            cy = np.floor((py - ref.origin_y) / res).astype(int)
            ok = (cx >= 0) & (cx < w) & (cy >= 0) & (cy < h)
            gt[cy[ok], cx[ok]] = 0
    return gt, lidar_z, n_origins


REACH = None


def test_ground_truth_lidar_map_matches_reference():
    global REACH
    t0 = time.time()
    ref = map_compare.load_map(REF_YAML)
    free = ref.grid > 240
    labels, _ = ndimage.label(free)
    h, w = ref.grid.shape
    start = (int((0.0 - ref.origin_y) / ref.resolution), int((0.0 - ref.origin_x) / ref.resolution))
    assert labels[start] > 0, "world origin is not free in the reference map"
    REACH = labels == labels[start]

    gt, lidar_z, n_origins = _ground_truth_map(ref)
    gt_map = map_compare.MapImage(grid=gt, resolution=ref.resolution,
                                  origin_x=ref.origin_x, origin_y=ref.origin_y)
    q = map_compare.compare_maps(gt_map, ref)
    print("lidar z %.3f m, %d origins, %d rays each, %.1f s" % (
        lidar_z, n_origins, N_RAYS, time.time() - t0))
    print(q)
    assert n_origins > 100
    assert q.n_occupied >= 2000 and q.n_tiles >= 1, "ground-truth map is degenerate: %s" % (q,)
    assert q.inlier_fraction >= 0.85, q
    assert abs(q.max_tile_rotation_deg) <= 2.0, q
    assert q.max_tile_shift_m <= 0.20, q
