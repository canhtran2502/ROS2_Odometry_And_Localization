"""AC4: the charger pose and the insertion segment lie on free space of the small_house map."""
import math
import os

import numpy as np
import pytest
import yaml
from scipy import ndimage

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.abspath(os.path.join(HERE, ".."))
SRC = os.path.abspath(os.path.join(PKG, ".."))
MAP_DIR = os.path.join(SRC, "bumperbot_mapping", "maps", "small_house")
MARKER_CONFIG = os.path.join(SRC, "bumperbot_mujoco", "config", "dock_marker_sim.yaml")

MIN_CLEARANCE = 0.30
STEP = 0.05


def read_pgm(path):
    with open(path, "rb") as f:
        data = f.read()
    tokens, pos = [], 0
    while len(tokens) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b"#":
            while data[pos:pos + 1] != b"\n":
                pos += 1
            continue
        start = pos
        while not data[pos:pos + 1].isspace():
            pos += 1
        tokens.append(data[start:pos])
    pos += 1
    assert tokens[0] == b"P5"
    w, h, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    assert maxval < 256
    return np.frombuffer(data, dtype=np.uint8, count=w * h, offset=pos).reshape(h, w)


@pytest.fixture(scope="module")
def world():
    with open(os.path.join(MAP_DIR, "map.yaml")) as f:
        meta = yaml.safe_load(f)
    img = read_pgm(os.path.join(MAP_DIR, meta["image"]))
    res = float(meta["resolution"])
    ox, oy = float(meta["origin"][0]), float(meta["origin"][1])
    occ_prob = (255.0 - img) / 255.0 if not meta.get("negate", 0) else img / 255.0
    occupied = occ_prob > float(meta["occupied_thresh"])
    free = occ_prob < float(meta["free_thresh"])
    assert occupied.any() and free.any()
    clearance = ndimage.distance_transform_edt(~occupied) * res
    h = img.shape[0]

    def cell(x, y):
        return h - 1 - int(np.floor((y - oy) / res)), int(np.floor((x - ox) / res))

    return free, clearance, cell


@pytest.fixture(scope="module")
def geometry():
    """Return (staging, charger) read from the shipped configuration."""
    with open(os.path.join(PKG, "config", "stations.yaml")) as f:
        stations = yaml.safe_load(f)
    with open(os.path.join(PKG, "config", "dock_manager.yaml")) as f:
        cfg = yaml.safe_load(f)["dock_manager"]["ros__parameters"]
    st = stations["stations"][stations["dock_station"]]
    distance = float(cfg["staging_distance"])
    sign = -1.0 if cfg["dock_backwards"] else 1.0
    staging = (float(st["x"]), float(st["y"]), float(st["yaw"]))
    charger = (staging[0] + sign * distance * math.cos(staging[2]),
               staging[1] + sign * distance * math.sin(staging[2]), staging[2])
    return staging, charger


def test_charger_pose_of_shipped_config(geometry):
    staging, charger = geometry
    assert staging == (0.0, 0.0, 0.0)
    assert charger == pytest.approx((-0.5, 0.0, 0.0), abs=1e-9)


def test_dock_logic_agrees_with_independent_charger_pose(geometry):
    from bumperbot_navigation.dock_logic import Pose2D, compute_dock_pose
    staging, charger = geometry
    got = compute_dock_pose(Pose2D(*staging), 0.5, True)
    assert (got.x, got.y, got.yaw) == pytest.approx(charger, abs=1e-9)


def test_charger_and_insertion_segment_are_free_with_clearance(world, geometry):
    free, clearance, cell = world
    staging, charger = geometry
    length = math.hypot(charger[0] - staging[0], charger[1] - staging[1])
    count = int(round(length / STEP))
    assert count == 10
    problems = []
    for i in range(count + 1):
        t = i / count
        x = staging[0] + t * (charger[0] - staging[0])
        y = staging[1] + t * (charger[1] - staging[1])
        row, col = cell(x, y)
        if not (0 <= row < free.shape[0] and 0 <= col < free.shape[1]):
            problems.append("({:.2f}, {:.2f}): outside the map".format(x, y))
            continue
        if not free[row, col]:
            problems.append("({:.2f}, {:.2f}): not a free cell".format(x, y))
        if clearance[row, col] < MIN_CLEARANCE:
            problems.append("({:.2f}, {:.2f}): clearance {:.2f} m < {}".format(
                x, y, clearance[row, col], MIN_CLEARANCE))
    assert not problems, "; ".join(problems)


def test_marker_config_matches_charger_pose(geometry):
    _, charger = geometry
    with open(MARKER_CONFIG) as f:
        cfg = yaml.safe_load(f)["dock_marker_sim"]["ros__parameters"]
    assert (cfg["dock_x"], cfg["dock_y"], cfg["dock_yaw"]) == pytest.approx(charger, abs=1e-6)
