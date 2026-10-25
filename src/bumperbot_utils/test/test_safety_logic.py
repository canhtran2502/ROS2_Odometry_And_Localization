"""AC2 (Python): the safety_logic case table in test/data/safety_cases.yaml."""
import math
import os

import pytest
import yaml

from bumperbot_utils import safety_logic as sl

DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data", "safety_cases.yaml")
TOL = 1e-9

with open(DATA) as _f:
    TABLE = yaml.safe_load(_f)
POLYGONS = {k: [tuple(p) for p in v] for k, v in TABLE["polygons"].items()}
CASES = TABLE["cases"]


def make_config(overrides):
    cfg = sl.Config()
    for key, value in (overrides or {}).items():
        assert hasattr(cfg, key), "unknown Config field in the table: " + key
        setattr(cfg, key, value)
    return cfg


def close(actual, expected):
    if isinstance(expected, bool) or isinstance(actual, bool):
        return actual == expected
    if isinstance(expected, (int, float)):
        if math.isinf(expected):
            return actual == expected
        if math.isnan(expected):
            return math.isnan(actual)
        return abs(actual - expected) <= TOL
    return actual == expected


def check(label, actual, expected):
    assert close(actual, expected), "{}: got {!r}, expected {!r}".format(label, actual, expected)


def run_geometry(case):
    fn = case["fn"]
    poly = POLYGONS[case["poly"]]
    if fn == "polygon_is_simple":
        check(case["name"], sl.polygon_is_simple(poly), case["expect"])
    elif fn == "point_in_polygon":
        check(case["name"], sl.point_in_polygon(case["point"][0], case["point"][1], poly),
              case["expect"])
    elif fn == "distance_to_polygon":
        check(case["name"], sl.distance_to_polygon(case["point"][0], case["point"][1], poly),
              case["expect"])
    else:
        got = sl.footprint_extents(poly)
        assert len(got) == 3
        for a, e in zip(got, case["expect"]):
            check(case["name"], a, e)


def run_scan(case):
    tf = case["tf"]
    points, ratio = sl.scan_to_points(
        case["ranges"], case["angle_min"], case["angle_increment"], case["range_min"],
        case["range_max"], tf[0], tf[1], tf[2])
    assert len(points) == len(case["expect_points"]), points
    for got, exp in zip(points, case["expect_points"]):
        check("x", got[0], exp[0])
        check("y", got[1], exp[1])
    check("invalid ratio", ratio, case["expect_ratio"])


def run_evaluate(case):
    ev = sl.evaluate_points([tuple(p) for p in case["points"]], make_config(case.get("cfg")))
    for key, value in case["expect"].items():
        check(key, getattr(ev, key), value)


def run_speed_limit(case):
    cfg = make_config(case.get("cfg"))
    check(case["name"], sl.speed_limit(case["d"], case["margin"], cfg), case["expect"])


def run_speed_grid(case):
    cfg = make_config(case.get("cfg"))
    n = case["n"]
    margin = case["margin"]
    grid = [case["d_min"] + (case["d_max"] - case["d_min"]) * i / (n - 1) for i in range(n)]
    values = [sl.speed_limit(d, margin, cfg) for d in grid]
    for earlier, later in zip(values, values[1:]):
        assert later >= earlier - 1e-12, "speed_limit decreases with the distance"
    assert all(0.0 <= v <= cfg.max_linear_speed + 1e-12 for v in values)
    assert sl.speed_limit(margin, margin, cfg) == 0.0
    assert sl.speed_limit(margin - 0.01, margin, cfg) == 0.0
    assert sl.speed_limit(float("inf"), margin, cfg) == cfg.max_linear_speed
    assert any(v > 0.0 for v in values)


def run_zone(case):
    got = sl.zone_for(case["d"], case["prev"], case["margin"], make_config(case.get("cfg")),
                      case.get("allow_contact", True))
    assert got == case["expect"]


def run_direction(case):
    got = sl.travel_direction(case["v_cmd"], case["v_odom"], case["stopped"])
    assert got == case["expect"]


def run_validate(case):
    error = sl.validate_config(make_config(case["cfg"]))
    if "expect_error" in case:
        assert error == case["expect_error"]
    else:
        assert error != "", "an invalid configuration was accepted"
        assert case["expect_error_contains"] in error, error


def run_dock_gate(case):
    gate = sl.DockGate(make_config(case.get("cfg")))
    for step in case["steps"]:
        if step.get("reset"):
            gate.reset()
        dist = step["dist"]
        active, reason = gate.update(step["t"], step["enabled"], step["state"], step["age"],
                                     None if math.isnan(dist) else dist)
        assert [active, reason] == step["expect"], "t={}".format(step["t"])


def build_inputs(start):
    inp = sl.Inputs()
    if start == "healthy":
        inp.scan_ok = True
        inp.scan_age = 0.05
        inp.tf_ok = True
        inp.tf_description = "base_footprint <- laser_link"
        inp.odom_age = 0.05
        inp.v_odom = 0.0
        inp.cmd_age = 0.05
        inp.v_cmd = 0.0
    return inp


def apply_set(inp, overrides):
    for key, value in (overrides or {}).items():
        if key == "evaluation":
            for ek, ev in value.items():
                assert hasattr(inp.evaluation, ek), ek
                setattr(inp.evaluation, ek, ev)
        else:
            assert hasattr(inp, key), "unknown Inputs field in the table: " + key
            setattr(inp, key, value)


def run_core(case):
    core = sl.SafetyCore(make_config(case.get("cfg")))
    inp = build_inputs(case["start"])
    for step in case["steps"]:
        apply_set(inp, step.get("set"))
        out = core.update(step["t"], inp)
        label = "{} t={}".format(case["name"], step["t"])
        for key, value in (step.get("expect") or {}).items():
            check("{} {}".format(label, key), getattr(out, key), value)
        action = step.get("action")
        if action == "reset":
            ok, msg = core.request_reset()
            assert [ok, msg] == step["expect_reset"], label
        elif action == "trigger_estop":
            core.trigger_estop("e-stop requested by service")
        elif action == "shutdown":
            core.shutdown()
        if "limit" in step:
            lim = step["limit"]
            v, w = core.limit_command(lim["v"], lim["w"], out)
            check(label + " limit v", v, lim["expect"][0])
            check(label + " limit w", w, lim["expect"][1])


RUNNERS = {
    "polygon_is_simple": run_geometry, "point_in_polygon": run_geometry,
    "distance_to_polygon": run_geometry, "footprint_extents": run_geometry,
    "scan_to_points": run_scan, "evaluate_points": run_evaluate, "speed_limit": run_speed_limit,
    "speed_limit_grid": run_speed_grid, "zone_for": run_zone,
    "travel_direction": run_direction, "validate_config": run_validate,
    "dock_gate": run_dock_gate, "core": run_core,
}


@pytest.mark.parametrize("case", CASES, ids=[c["name"] for c in CASES])
def test_case(case):
    RUNNERS[case["fn"]](case)


def test_table_is_complete():
    names = [c["name"] for c in CASES]
    assert len(names) == len(set(names)), "duplicate case names"
    assert {c["fn"] for c in CASES} == set(RUNNERS), "every function needs a case"
    assert len(CASES) >= 150


def test_default_config_is_valid_and_matches_the_plan_defaults():
    cfg = sl.Config()
    assert sl.validate_config(cfg) == ""
    assert cfg.footprint == [0.10, 0.09, 0.10, -0.09, -0.10, -0.09, -0.10, 0.09]
    assert (cfg.stop_margin, cfg.slowdown_distance, cfg.contact_margin) == (0.05, 0.35, 0.015)
    assert (cfg.dock_max_speed, cfg.dock_stop_margin) == (0.06, 0.01)
