"""AC3: every shipped station lies on free space of the small_house map with enough clearance
and in the free connected component of the dock."""
import os

import numpy as np
import pytest
import yaml
from scipy import ndimage

HERE = os.path.dirname(os.path.abspath(__file__))
STATIONS = os.path.join(HERE, "..", "config", "stations.yaml")
MAP_DIR = os.path.join(HERE, "..", "..", "bumperbot_mapping", "maps", "small_house")

MIN_CLEARANCE = 0.30
CONNECT_CLEARANCE = 0.15


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
    # distance (m) from every cell to the nearest occupied cell
    clearance = ndimage.distance_transform_edt(~occupied) * res
    h = img.shape[0]

    def cell(x, y):
        col = int(np.floor((x - ox) / res))
        row = h - 1 - int(np.floor((y - oy) / res))
        return row, col

    return occupied, free, clearance, cell


@pytest.fixture(scope="module")
def stations():
    with open(STATIONS) as f:
        return yaml.safe_load(f)["stations"]


def test_station_count(stations):
    assert "dock" in stations
    assert len(stations) >= 4


def test_stations_free_clear_and_connected(world, stations):
    occupied, free, clearance, cell = world
    candidate = free & (clearance > CONNECT_CLEARANCE)
    labels, _ = ndimage.label(candidate)
    drow, dcol = cell(stations["dock"]["x"], stations["dock"]["y"])
    dock_label = labels[drow, dcol]
    assert dock_label != 0, "dock is not in the free space of the map"
    problems = []
    for name, s in stations.items():
        row, col = cell(s["x"], s["y"])
        if not (0 <= row < free.shape[0] and 0 <= col < free.shape[1]):
            problems.append(f"{name}: outside the map")
            continue
        if not free[row, col]:
            problems.append(f"{name}: not a free cell")
        if clearance[row, col] < MIN_CLEARANCE:
            problems.append(f"{name}: clearance {clearance[row, col]:.2f} m < {MIN_CLEARANCE}")
        if labels[row, col] != dock_label:
            problems.append(f"{name}: not in the free component of the dock")
    assert not problems, "; ".join(problems)
