// AC2 (C++): runs the shared case table test/data/safety_cases.yaml against safety_logic.cpp.
// test_safety_logic.py runs the same table against the Python twin.

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <random>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "bumperbot_utils/safety_logic.hpp"

namespace sf = bumperbot_utils::safety;

namespace
{
constexpr double kTol = 1e-9;

using StringMember = std::string sf::Config::*;
using BoolMember = bool sf::Config::*;
using DoubleMember = double sf::Config::*;

const std::map<std::string, StringMember> & string_fields()
{
  static const std::map<std::string, StringMember> m = {
    {"enforcement", &sf::Config::enforcement}, {"robot_frame", &sf::Config::robot_frame},
    {"scan_topic", &sf::Config::scan_topic}, {"odom_topic", &sf::Config::odom_topic},
    {"cmd_vel_in_topic", &sf::Config::cmd_vel_in_topic},
    {"cmd_vel_out_topic", &sf::Config::cmd_vel_out_topic},
    {"lock_topic", &sf::Config::lock_topic}, {"status_topic", &sf::Config::status_topic},
    {"estop_topic", &sf::Config::estop_topic},
    {"dock_status_topic", &sf::Config::dock_status_topic}};
  return m;
}

const std::map<std::string, BoolMember> & bool_fields()
{
  static const std::map<std::string, BoolMember> m = {
    {"use_odom", &sf::Config::use_odom}, {"latch_faults", &sf::Config::latch_faults},
    {"latch_on_contact", &sf::Config::latch_on_contact},
    {"dock_approach_enabled", &sf::Config::dock_approach_enabled},
    {"require_reset_after_restart", &sf::Config::require_reset_after_restart}};
  return m;
}

const std::map<std::string, DoubleMember> & double_fields()
{
  static const std::map<std::string, DoubleMember> m = {
    {"lateral_margin", &sf::Config::lateral_margin}, {"stop_margin", &sf::Config::stop_margin},
    {"stop_release_hysteresis", &sf::Config::stop_release_hysteresis},
    {"slowdown_distance", &sf::Config::slowdown_distance},
    {"contact_margin", &sf::Config::contact_margin},
    {"max_linear_speed", &sf::Config::max_linear_speed},
    {"max_angular_speed", &sf::Config::max_angular_speed},
    {"response_time", &sf::Config::response_time}, {"max_decel", &sf::Config::max_decel},
    {"stopped_speed", &sf::Config::stopped_speed},
    {"brake_hold_time", &sf::Config::brake_hold_time},
    {"scan_timeout", &sf::Config::scan_timeout}, {"odom_timeout", &sf::Config::odom_timeout},
    {"cmd_timeout", &sf::Config::cmd_timeout}, {"recovery_time", &sf::Config::recovery_time},
    {"max_invalid_beam_ratio", &sf::Config::max_invalid_beam_ratio},
    {"rate", &sf::Config::rate}, {"status_rate", &sf::Config::status_rate},
    {"dock_max_speed", &sf::Config::dock_max_speed},
    {"dock_stop_margin", &sf::Config::dock_stop_margin},
    {"dock_approach_radius", &sf::Config::dock_approach_radius},
    {"dock_approach_max_time", &sf::Config::dock_approach_max_time},
    {"dock_status_timeout", &sf::Config::dock_status_timeout}};
  return m;
}

sf::Config make_config(const YAML::Node & overrides)
{
  sf::Config cfg;
  if (!overrides || !overrides.IsMap()) {
    return cfg;
  }
  for (const auto & kv : overrides) {
    const std::string key = kv.first.as<std::string>();
    if (string_fields().count(key)) {
      cfg.*(string_fields().at(key)) = kv.second.as<std::string>();
    } else if (bool_fields().count(key)) {
      cfg.*(bool_fields().at(key)) = kv.second.as<bool>();
    } else if (double_fields().count(key)) {
      cfg.*(double_fields().at(key)) = kv.second.as<double>();
    } else if (key == "footprint") {
      cfg.footprint = kv.second.as<std::vector<double>>();
    } else if (key == "contact_debounce_scans") {
      cfg.contact_debounce_scans = kv.second.as<int64_t>();
    } else if (key == "debug_inject_tick_exceptions") {
      cfg.debug_inject_tick_exceptions = kv.second.as<int64_t>();
    } else {
      ADD_FAILURE() << "unknown Config field in the table: " << key;
    }
  }
  return cfg;
}

bool close(double actual, double expected)
{
  if (std::isinf(expected)) {
    return actual == expected;
  }
  if (std::isnan(expected)) {
    return std::isnan(actual);
  }
  return std::fabs(actual - expected) <= kTol;
}

#define CHECK_NUM(label, actual, expected) \
  EXPECT_TRUE(close((actual), (expected))) << (label) << ": got " << (actual) << ", expected " \
                                           << (expected)

sf::Polygon polygon_of(const YAML::Node & table, const std::string & name)
{
  sf::Polygon poly;
  for (const auto & p : table["polygons"][name]) {
    poly.emplace_back(p[0].as<double>(), p[1].as<double>());
  }
  return poly;
}

void run_geometry(const YAML::Node & table, const YAML::Node & c)
{
  const std::string fn = c["fn"].as<std::string>();
  const sf::Polygon poly = polygon_of(table, c["poly"].as<std::string>());
  if (fn == "polygon_is_simple") {
    EXPECT_EQ(sf::polygon_is_simple(poly), c["expect"].as<bool>());
  } else if (fn == "point_in_polygon") {
    EXPECT_EQ(
      sf::point_in_polygon(c["point"][0].as<double>(), c["point"][1].as<double>(), poly),
      c["expect"].as<bool>());
  } else if (fn == "distance_to_polygon") {
    const double got =
      sf::distance_to_polygon(c["point"][0].as<double>(), c["point"][1].as<double>(), poly);
    CHECK_NUM("distance", got, c["expect"].as<double>());
  } else {
    const sf::Extents e = sf::footprint_extents(poly);
    CHECK_NUM("x_front", e.x_front, c["expect"][0].as<double>());
    CHECK_NUM("x_rear", e.x_rear, c["expect"][1].as<double>());
    CHECK_NUM("half_width", e.half_width, c["expect"][2].as<double>());
  }
}

void run_scan(const YAML::Node & c)
{
  const sf::ScanPoints sp = sf::scan_to_points(
    c["ranges"].as<std::vector<double>>(), c["angle_min"].as<double>(),
    c["angle_increment"].as<double>(), c["range_min"].as<double>(),
    c["range_max"].as<double>(), c["tf"][0].as<double>(), c["tf"][1].as<double>(),
    c["tf"][2].as<double>());
  ASSERT_EQ(sp.points.size(), c["expect_points"].size());
  for (size_t i = 0; i < sp.points.size(); ++i) {
    CHECK_NUM("x", sp.points[i].first, c["expect_points"][i][0].as<double>());
    CHECK_NUM("y", sp.points[i].second, c["expect_points"][i][1].as<double>());
  }
  CHECK_NUM("invalid ratio", sp.invalid_ratio, c["expect_ratio"].as<double>());
}

void run_evaluate(const YAML::Node & c)
{
  std::vector<sf::Point> points;
  for (const auto & p : c["points"]) {
    points.emplace_back(p[0].as<double>(), p[1].as<double>());
  }
  const sf::Evaluation ev = sf::evaluate_points(points, make_config(c["cfg"]));
  for (const auto & kv : c["expect"]) {
    const std::string key = kv.first.as<std::string>();
    if (key == "front_clearance") {
      CHECK_NUM(key, ev.front_clearance, kv.second.as<double>());
    } else if (key == "rear_clearance") {
      CHECK_NUM(key, ev.rear_clearance, kv.second.as<double>());
    } else if (key == "nearest_distance") {
      CHECK_NUM(key, ev.nearest_distance, kv.second.as<double>());
    } else if (key == "nearest_bearing") {
      CHECK_NUM(key, ev.nearest_bearing, kv.second.as<double>());
    } else if (key == "contact_front") {
      EXPECT_EQ(ev.contact_front, kv.second.as<bool>());
    } else if (key == "contact_rear") {
      EXPECT_EQ(ev.contact_rear, kv.second.as<bool>());
    } else {
      ADD_FAILURE() << "unknown Evaluation field: " << key;
    }
  }
}

void run_speed_grid(const YAML::Node & c)
{
  const sf::Config cfg = make_config(c["cfg"]);
  const int n = c["n"].as<int>();
  const double margin = c["margin"].as<double>();
  const double d_min = c["d_min"].as<double>();
  const double d_max = c["d_max"].as<double>();
  double previous = -1.0;
  bool any_positive = false;
  for (int i = 0; i < n; ++i) {
    const double d = d_min + (d_max - d_min) * i / (n - 1);
    const double v = sf::speed_limit(d, margin, cfg);
    EXPECT_GE(v, previous - 1e-12) << "speed_limit decreases with the distance";
    EXPECT_GE(v, 0.0);
    EXPECT_LE(v, cfg.max_linear_speed + 1e-12);
    any_positive = any_positive || v > 0.0;
    previous = v;
  }
  EXPECT_TRUE(any_positive);
  EXPECT_EQ(sf::speed_limit(margin, margin, cfg), 0.0);
  EXPECT_EQ(sf::speed_limit(margin - 0.01, margin, cfg), 0.0);
  EXPECT_EQ(
    sf::speed_limit(std::numeric_limits<double>::infinity(), margin, cfg), cfg.max_linear_speed);
}

void run_validate(const YAML::Node & c)
{
  const std::string error = sf::validate_config(make_config(c["cfg"]));
  if (c["expect_error"]) {
    EXPECT_EQ(error, c["expect_error"].as<std::string>());
  } else {
    EXPECT_NE(error, "") << "an invalid configuration was accepted";
    EXPECT_NE(error.find(c["expect_error_contains"].as<std::string>()), std::string::npos)
      << error;
  }
}

void run_dock_gate(const YAML::Node & c)
{
  sf::DockGate gate(make_config(c["cfg"]));
  for (const auto & step : c["steps"]) {
    if (step["reset"] && step["reset"].as<bool>()) {
      gate.reset();
    }
    const auto result = gate.update(
      step["t"].as<double>(), step["enabled"].as<bool>(), step["state"].as<std::string>(),
      step["age"].as<double>(), step["dist"].as<double>());
    EXPECT_EQ(result.first, step["expect"][0].as<bool>()) << "t=" << step["t"].as<double>();
    EXPECT_EQ(result.second, step["expect"][1].as<std::string>())
      << "t=" << step["t"].as<double>();
  }
}

sf::Inputs build_inputs(const std::string & start)
{
  sf::Inputs inp;
  if (start == "healthy") {
    inp.scan_ok = true;
    inp.scan_age = 0.05;
    inp.tf_ok = true;
    inp.tf_description = "base_footprint <- laser_link";
    inp.odom_age = 0.05;
    inp.v_odom = 0.0;
    inp.cmd_age = 0.05;
    inp.v_cmd = 0.0;
  }
  return inp;
}

void apply_set(sf::Inputs & inp, const YAML::Node & set)
{
  if (!set) {
    return;
  }
  for (const auto & kv : set) {
    const std::string key = kv.first.as<std::string>();
    const YAML::Node & v = kv.second;
    if (key == "scan_ok") {
      inp.scan_ok = v.as<bool>();
    } else if (key == "scan_age") {
      inp.scan_age = v.as<double>();
    } else if (key == "scan_invalid_reason") {
      inp.scan_invalid_reason = v.as<std::string>();
    } else if (key == "tf_ok") {
      inp.tf_ok = v.as<bool>();
    } else if (key == "tf_description") {
      inp.tf_description = v.as<std::string>();
    } else if (key == "odom_age") {
      inp.odom_age = v.as<double>();
    } else if (key == "v_odom") {
      inp.v_odom = v.as<double>();
    } else if (key == "cmd_age") {
      inp.cmd_age = v.as<double>();
    } else if (key == "v_cmd") {
      inp.v_cmd = v.as<double>();
    } else if (key == "scan_seq") {
      inp.scan_seq = v.as<int64_t>();
    } else if (key == "estop_input") {
      inp.estop_input = v.as<bool>();
    } else if (key == "dock_active") {
      inp.dock_active = v.as<bool>();
    } else if (key == "dock_reason") {
      inp.dock_reason = v.as<std::string>();
    } else if (key == "tick_exception") {
      inp.tick_exception = v.as<std::string>();
    } else if (key == "evaluation") {
      for (const auto & ekv : v) {
        const std::string ek = ekv.first.as<std::string>();
        if (ek == "front_clearance") {
          inp.evaluation.front_clearance = ekv.second.as<double>();
        } else if (ek == "rear_clearance") {
          inp.evaluation.rear_clearance = ekv.second.as<double>();
        } else if (ek == "nearest_distance") {
          inp.evaluation.nearest_distance = ekv.second.as<double>();
        } else if (ek == "nearest_bearing") {
          inp.evaluation.nearest_bearing = ekv.second.as<double>();
        } else if (ek == "contact_front") {
          inp.evaluation.contact_front = ekv.second.as<bool>();
        } else if (ek == "contact_rear") {
          inp.evaluation.contact_rear = ekv.second.as<bool>();
        } else {
          ADD_FAILURE() << "unknown evaluation field: " << ek;
        }
      }
    } else {
      ADD_FAILURE() << "unknown Inputs field in the table: " << key;
    }
  }
}

void check_outputs(const std::string & label, const sf::Outputs & out, const YAML::Node & expect)
{
  if (!expect) {
    return;
  }
  for (const auto & kv : expect) {
    const std::string key = kv.first.as<std::string>();
    const YAML::Node & v = kv.second;
    const std::string what = label + " " + key;
    if (key == "state") {
      EXPECT_EQ(out.state, v.as<std::string>()) << what;
    } else if (key == "locked") {
      EXPECT_EQ(out.locked, v.as<bool>()) << what;
    } else if (key == "braking") {
      EXPECT_EQ(out.braking, v.as<bool>()) << what;
    } else if (key == "estop_latched") {
      EXPECT_EQ(out.estop_latched, v.as<bool>()) << what;
    } else if (key == "front_zone") {
      EXPECT_EQ(out.front_zone, v.as<std::string>()) << what;
    } else if (key == "rear_zone") {
      EXPECT_EQ(out.rear_zone, v.as<std::string>()) << what;
    } else if (key == "active_zone") {
      EXPECT_EQ(out.active_zone, v.as<std::string>()) << what;
    } else if (key == "max_forward") {
      CHECK_NUM(what, out.max_forward, v.as<double>());
    } else if (key == "max_reverse") {
      CHECK_NUM(what, out.max_reverse, v.as<double>());
    } else if (key == "reason") {
      EXPECT_EQ(out.reason, v.as<std::string>()) << what;
    } else if (key == "dock_profile_active") {
      EXPECT_EQ(out.dock_profile_active, v.as<bool>()) << what;
    } else {
      ADD_FAILURE() << "unknown Outputs field: " << key;
    }
  }
}

void run_core(const YAML::Node & c)
{
  sf::SafetyCore core(make_config(c["cfg"]));
  sf::Inputs inp = build_inputs(c["start"].as<std::string>());
  for (const auto & step : c["steps"]) {
    apply_set(inp, step["set"]);
    const double t = step["t"].as<double>();
    const sf::Outputs out = core.update(t, inp);
    const std::string label = c["name"].as<std::string>() + " t=" + std::to_string(t);
    check_outputs(label, out, step["expect"]);
    if (step["action"]) {
      const std::string action = step["action"].as<std::string>();
      if (action == "reset") {
        const auto result = core.request_reset();
        EXPECT_EQ(result.first, step["expect_reset"][0].as<bool>()) << label;
        EXPECT_EQ(result.second, step["expect_reset"][1].as<std::string>()) << label;
      } else if (action == "trigger_estop") {
        core.trigger_estop("e-stop requested by service");
      } else if (action == "shutdown") {
        core.shutdown();
      } else {
        ADD_FAILURE() << "unknown action " << action;
      }
    }
    if (step["limit"]) {
      const YAML::Node & lim = step["limit"];
      const auto vw = core.limit_command(lim["v"].as<double>(), lim["w"].as<double>(), out);
      CHECK_NUM(label + " limit v", vw.first, lim["expect"][0].as<double>());
      CHECK_NUM(label + " limit w", vw.second, lim["expect"][1].as<double>());
    }
  }
}

void run_case(const YAML::Node & table, const YAML::Node & c)
{
  const std::string fn = c["fn"].as<std::string>();
  if (fn == "polygon_is_simple" || fn == "point_in_polygon" || fn == "distance_to_polygon" ||
    fn == "footprint_extents")
  {
    run_geometry(table, c);
  } else if (fn == "scan_to_points") {
    run_scan(c);
  } else if (fn == "evaluate_points") {
    run_evaluate(c);
  } else if (fn == "speed_limit") {
    const double got = sf::speed_limit(
      c["d"].as<double>(), c["margin"].as<double>(),
      make_config(c["cfg"]));
    CHECK_NUM(c["name"].as<std::string>(), got, c["expect"].as<double>());
  } else if (fn == "speed_limit_grid") {
    run_speed_grid(c);
  } else if (fn == "zone_for") {
    const bool allow = c["allow_contact"] ? c["allow_contact"].as<bool>() : true;
    EXPECT_EQ(
      sf::zone_for(
        c["d"].as<double>(), c["prev"].as<std::string>(), c["margin"].as<double>(),
        make_config(c["cfg"]), allow),
      c["expect"].as<std::string>());
  } else if (fn == "travel_direction") {
    EXPECT_EQ(
      sf::travel_direction(
        c["v_cmd"].as<double>(), c["v_odom"].as<double>(), c["stopped"].as<double>()),
      c["expect"].as<int>());
  } else if (fn == "validate_config") {
    run_validate(c);
  } else if (fn == "dock_gate") {
    run_dock_gate(c);
  } else if (fn == "core") {
    run_core(c);
  } else {
    ADD_FAILURE() << "unknown function in the table: " << fn;
  }
}

YAML::Node load_table()
{
  return YAML::LoadFile(std::string(TEST_DATA_DIR) + "/safety_cases.yaml");
}
}  // namespace

TEST(SafetyLogicCases, EveryCaseOfTheSharedTable)
{
  const YAML::Node table = load_table();
  std::set<std::string> names;
  int count = 0;
  for (const auto & c : table["cases"]) {
    const std::string name = c["name"].as<std::string>();
    EXPECT_TRUE(names.insert(name).second) << "duplicate case " << name;
    SCOPED_TRACE(name);
    run_case(table, c);
    ++count;
  }
  EXPECT_GE(count, 150);
}

TEST(SafetyLogicCases, DefaultConfigIsValid)
{
  const sf::Config cfg;
  EXPECT_EQ(sf::validate_config(cfg), "");
  const std::vector<double> fp = {0.10, 0.125, 0.10, -0.125, -0.10, -0.125, -0.10, 0.125};
  EXPECT_EQ(cfg.footprint, fp);
  // Iteration 3: new parameters and their defaults
  EXPECT_EQ(cfg.contact_debounce_scans, 3);
  EXPECT_FALSE(cfg.require_reset_after_restart);
}

TEST(SafetyLogicCases, ContactDebounceScansBelowOneIsInvalid)
{
  sf::Config cfg;
  cfg.contact_debounce_scans = 0;
  EXPECT_EQ(sf::validate_config(cfg), "contact_debounce_scans must be >= 1");
  cfg.contact_debounce_scans = -2;
  EXPECT_NE(sf::validate_config(cfg), "");
  cfg.contact_debounce_scans = 1;
  EXPECT_EQ(sf::validate_config(cfg), "");
}

TEST(SafetyLogicCases, BlindZoneWarningWhenRangeMinOverlapsTheStopCorridor)
{
  const sf::Config cfg;
  // Shipped geometry: laser at x=0.0, y=0 with range_min 0.12 reaches into the rear dock corridor
  // (x from -0.11 to -0.10) and the front corridor (0.10 .. 0.15)
  const std::string both = sf::blind_zone_warning(cfg, 0.0, 0.0, 0.12);
  EXPECT_NE(both.find("front"), std::string::npos) << both;
  EXPECT_NE(both.find("rear"), std::string::npos) << both;
  EXPECT_NE(both.find("blind"), std::string::npos) << both;
  // A tiny range_min does not reach any stop corridor: no warning
  EXPECT_EQ(sf::blind_zone_warning(cfg, 0.0, 0.0, 0.05), "");
}

TEST(SafetyLogicCases, ShippedConfigYamlEqualsCompiledDefaults)
{
  const YAML::Node root = YAML::LoadFile(std::string(CONFIG_DIR) + "/safety_supervisor.yaml");
  const YAML::Node params = root["safety_supervisor"]["ros__parameters"];
  const sf::Config defaults;
  size_t seen = 0;
  for (const auto & kv : params) {
    const std::string key = kv.first.as<std::string>();
    ++seen;
    if (string_fields().count(key)) {
      EXPECT_EQ(kv.second.as<std::string>(), defaults.*(string_fields().at(key))) << key;
    } else if (bool_fields().count(key)) {
      EXPECT_EQ(kv.second.as<bool>(), defaults.*(bool_fields().at(key))) << key;
    } else if (double_fields().count(key)) {
      EXPECT_DOUBLE_EQ(kv.second.as<double>(), defaults.*(double_fields().at(key))) << key;
    } else if (key == "footprint") {
      EXPECT_EQ(kv.second.as<std::vector<double>>(), defaults.footprint);
    } else if (key == "contact_debounce_scans") {
      EXPECT_EQ(kv.second.as<int64_t>(), defaults.contact_debounce_scans);
    } else if (key == "debug_inject_tick_exceptions") {
      EXPECT_EQ(kv.second.as<int64_t>(), defaults.debug_inject_tick_exceptions);
    } else {
      ADD_FAILURE() << "parameter not known to the C++ Config: " << key;
    }
  }
  EXPECT_EQ(
    seen, string_fields().size() + bool_fields().size() + double_fields().size() + 3)
    << "the YAML must list every parameter";
}

// ---------------------------------------------------------------------------------------------
// Iteration 3 (M6): Monte Carlo of the contact debounce with range noise (sigma 0.01 m)
// ---------------------------------------------------------------------------------------------
namespace
{
struct MonteCarlo
{
  int latched = 0;
  int worst_first = 0;
  int contact_scans = 0;
  int total_scans = 0;
  int resets = 0;
};

MonteCarlo mc_run(
  double clearance, double v, int64_t debounce, int scans, bool manual_reset = false,
  unsigned seed = 12345, int runs = 200)
{
  std::mt19937 gen(seed);
  std::normal_distribution<double> noise(0.0, 0.01);
  std::vector<double> angles;
  for (int a = -60; a <= 60; ++a) {
    angles.push_back(a * M_PI / 180.0);
  }
  MonteCarlo result;
  for (int run = 0; run < runs; ++run) {
    sf::Config cfg;
    cfg.contact_debounce_scans = debounce;
    sf::SafetyCore core(cfg);
    int first = 0;
    sf::Inputs inp;
    for (int scan = 1; scan <= scans; ++scan) {
      std::vector<double> ranges;
      for (const double angle : angles) {
        ranges.push_back((0.10 + clearance) / std::cos(angle) + noise(gen));
      }
      const auto sp = sf::scan_to_points(
        ranges, angles.front(), M_PI / 180.0, 0.05, 12.0, 0.0, 0.0, 0.0);
      const sf::Evaluation ev = sf::evaluate_points(sp.points, cfg);
      result.contact_scans += ev.contact_front ? 1 : 0;
      ++result.total_scans;
      inp = sf::Inputs();
      inp.scan_ok = true;
      inp.scan_age = 0.05;
      inp.tf_ok = true;
      inp.odom_age = 0.05;
      inp.v_odom = v;
      inp.cmd_age = 0.05;
      inp.v_cmd = v;
      inp.evaluation = ev;
      inp.scan_seq = scan;
      const sf::Outputs out = core.update(0.2 * scan, inp);
      if (out.estop_latched && first == 0) {
        first = scan;
      }
    }
    if (first != 0) {
      ++result.latched;
      result.worst_first = std::max(result.worst_first, first);
    }
    if (manual_reset) {
      core.trigger_estop("e-stop requested by service");
      inp.scan_seq = scans + 1;
      core.update(0.2 * (scans + 1), inp);
      result.resets += core.request_reset().first ? 1 : 0;
    }
  }
  return result;
}
}  // namespace

TEST(SafetyLogicMonteCarlo, RestDistanceNeverLatchesAndResetIsReliable)
{
  const MonteCarlo r = mc_run(0.035, 0.0, 3, 10, true);
  EXPECT_GT(static_cast<double>(r.contact_scans) / r.total_scans, 0.2);
  EXPECT_EQ(r.latched, 0);
  EXPECT_EQ(r.resets, 200);
}

TEST(SafetyLogicMonteCarlo, DebounceKeepsFalseLatchesNearZeroWhileCreeping)
{
  const MonteCarlo with = mc_run(0.04, 0.03, 3, 3);
  const MonteCarlo without = mc_run(0.04, 0.03, 1, 3);
  EXPECT_LE(with.latched, 6) << "debounce 3: " << with.latched << " of 200";
  EXPECT_GE(without.latched, 80) << "debounce 1: " << without.latched << " of 200";
  EXPECT_GE(without.latched, 10 * std::max(with.latched, 1));
  EXPECT_EQ(mc_run(0.05, 0.03, 3, 3).latched, 0);
}

TEST(SafetyLogicMonteCarlo, TrueContactStillLatchesWithinThreeScans)
{
  for (const double clearance : {0.005, 0.0}) {
    const MonteCarlo r = mc_run(clearance, 0.1, 3, 5);
    EXPECT_EQ(r.latched, 200) << clearance;
    EXPECT_EQ(r.worst_first, 3) << clearance;
  }
}
