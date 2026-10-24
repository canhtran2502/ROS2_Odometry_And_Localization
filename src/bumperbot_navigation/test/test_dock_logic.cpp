// AC2 (C++ side): dock_logic geometry, controller, detection filter, phase timer and tracker.
// Executes the same case table (test/data/dock_cases.yaml) as test_dock_logic.py.
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "bumperbot_navigation/dock_logic.hpp"
#include "bumperbot_navigation/mission_logic.hpp"

using namespace bumperbot_navigation;  // NOLINT

namespace
{
const std::string kData = TEST_DATA_DIR;
const double kTol = 1e-9;

YAML::Node cases()
{
  static YAML::Node doc = YAML::LoadFile(kData + "/dock_cases.yaml");
  return doc;
}

Pose2D to_pose(const YAML::Node & n)
{
  Pose2D p;
  p.x = n[0].as<double>();
  p.y = n[1].as<double>();
  p.yaw = n[2].as<double>();
  return p;
}

void expect_pose(const Pose2D & got, const YAML::Node & exp, bool abs_yaw = false)
{
  EXPECT_NEAR(got.x, exp[0].as<double>(), kTol);
  EXPECT_NEAR(got.y, exp[1].as<double>(), kTol);
  if (abs_yaw) {
    EXPECT_NEAR(std::fabs(got.yaw), std::fabs(exp[2].as<double>()), kTol);
  } else {
    EXPECT_NEAR(got.yaw, exp[2].as<double>(), kTol);
  }
}

ControlParams to_control_params(const YAML::Node & n)
{
  ControlParams p;
  for (const auto & kv : n) {
    const std::string k = kv.first.as<std::string>();
    if (k == "staging_distance") {
      p.staging_distance = kv.second.as<double>();
    } else if (k == "dock_backwards") {
      p.dock_backwards = kv.second.as<bool>();
    } else if (k == "max_linear_speed") {
      p.max_linear_speed = kv.second.as<double>();
    } else if (k == "max_angular_speed") {
      p.max_angular_speed = kv.second.as<double>();
    } else if (k == "approach_speed") {
      p.approach_speed = kv.second.as<double>();
    } else if (k == "slow_down_distance") {
      p.slow_down_distance = kv.second.as<double>();
    } else if (k == "k_lateral") {
      p.k_lateral = kv.second.as<double>();
    } else if (k == "k_heading") {
      p.k_heading = kv.second.as<double>();
    } else if (k == "align_position_tolerance") {
      p.align_position_tolerance = kv.second.as<double>();
    } else if (k == "align_yaw_tolerance") {
      p.align_yaw_tolerance = kv.second.as<double>();
    } else if (k == "max_insert_lateral_error") {
      p.max_insert_lateral_error = kv.second.as<double>();
    } else if (k == "max_insert_yaw_error") {
      p.max_insert_yaw_error = kv.second.as<double>();
    } else {
      ADD_FAILURE() << "unknown control param " << k;
    }
  }
  return p;
}

PhaseResult to_phase(const std::string & name)
{
  if (name == "OK") {return PhaseResult::OK;}
  if (name == "TIMEOUT") {return PhaseResult::TIMEOUT;}
  if (name == "STALL") {return PhaseResult::STALL;}
  if (name == "SAFETY_TIMEOUT") {return PhaseResult::SAFETY_TIMEOUT;}
  if (name == "MISALIGNED") {return PhaseResult::MISALIGNED;}
  ADD_FAILURE() << "unknown phase " << name;
  return PhaseResult::TIMEOUT;
}

std::string fmt(const DockStep & s)
{
  if (s.kind == DockStepKind::FINISHED) {return "FINISHED";}
  return std::string(to_string(s.kind)) + ":" + std::to_string(s.attempt);
}

void apply_event(DockTracker & tr, const YAML::Node & ev)
{
  if (ev.IsMap()) {
    tr.report_verify(ev["verify"][0].as<double>(), ev["verify"][1].as<double>());
    return;
  }
  const std::string s = ev.as<std::string>();
  if (s == "cancel") {
    tr.request_cancel();
  } else if (s == "abort") {
    tr.abort("internal error");
  } else if (s.rfind("nav:", 0) == 0) {
    tr.report_nav(s == "nav:ok");
  } else if (s.rfind("det:", 0) == 0) {
    tr.report_detection(s == "det:ok");
  } else if (s.rfind("phase:", 0) == 0) {
    tr.report_phase(to_phase(s.substr(6)));
  } else {
    ADD_FAILURE() << "unknown event " << s;
  }
}

void check_fields(const DockTracker & tr, const YAML::Node & exp)
{
  for (const auto & kv : exp) {
    const std::string k = kv.first.as<std::string>();
    if (k == "attempt") {
      EXPECT_EQ(tr.attempt(), kv.second.as<int>());
    } else if (k == "docked") {
      EXPECT_EQ(tr.docked(), kv.second.as<bool>());
    } else if (k == "message") {
      EXPECT_EQ(tr.message(), kv.second.as<std::string>());
    } else if (k == "detection_used") {
      EXPECT_EQ(tr.detection_used(), kv.second.as<bool>());
    } else if (k == "state") {
      EXPECT_EQ(tr.state(), kv.second.as<std::string>());
    } else if (k == "outcome") {
      EXPECT_EQ(tr.outcome(), kv.second.as<int>());
    } else if (k != "steps") {
      ADD_FAILURE() << "unknown expectation " << k;
    }
  }
}
}  // namespace

// ------------------------------------------------------------------ geometry
TEST(DockGeometry, ComputeDockPose)
{
  for (const auto & c : cases()["geometry"]["compute_dock_pose"]) {
    Pose2D got = compute_dock_pose(
      to_pose(c["staging"]), c["distance"].as<double>(), c["backwards"].as<bool>());
    expect_pose(got, c["expect"]);
  }
  expect_pose(compute_dock_pose(Pose2D{0.0, 0.0, 0.0}, 0.5, true), YAML::Load("[-0.5, 0.0, 0.0]"));
}

TEST(DockGeometry, NormalizeAngle)
{
  for (const auto & c : cases()["geometry"]["normalize_angle"]) {
    const double got = normalize_angle(c["angle"].as<double>());
    if (c["expect_abs"]) {
      EXPECT_NEAR(std::fabs(got), c["expect_abs"].as<double>(), kTol);
    } else {
      EXPECT_NEAR(got, c["expect"].as<double>(), kTol);
    }
    EXPECT_LE(std::fabs(got), M_PI + kTol);
  }
}

TEST(DockGeometry, ComposeInverseRelative)
{
  for (const auto & c : cases()["geometry"]["compose"]) {
    expect_pose(compose(to_pose(c["a"]), to_pose(c["b"])), c["expect"]);
  }
  for (const auto & c : cases()["geometry"]["inverse"]) {
    expect_pose(inverse(to_pose(c["a"])), c["expect"]);
  }
  for (const auto & c : cases()["geometry"]["relative"]) {
    const bool abs_yaw = c["expect_abs_yaw"] && c["expect_abs_yaw"].as<bool>();
    expect_pose(relative(to_pose(c["frame"]), to_pose(c["pose"])), c["expect"], abs_yaw);
  }
}

TEST(DockGeometry, RoundTrip)
{
  for (const auto & v : cases()["geometry"]["roundtrip"]) {
    const Pose2D a = to_pose(v);
    for (const Pose2D & got : {compose(a, inverse(a)), compose(inverse(a), a)}) {
      EXPECT_LT(std::hypot(got.x, got.y), kTol);
      EXPECT_LT(std::fabs(normalize_angle(got.yaw)), kTol);
    }
  }
}

TEST(DockGeometry, ProjectPose)
{
  for (const auto & c : cases()["geometry"]["project_pose"]) {
    Pose3 frame, pose;
    for (size_t i = 0; i < 7; ++i) {
      frame[i] = c["frame"][i].as<double>();
      pose[i] = c["pose"][i].as<double>();
    }
    expect_pose(project_pose(frame, pose), c["expect"]);
  }
}

// ------------------------------------------------------------------ controller
TEST(DockController, Cases)
{
  for (const auto & c : cases()["controller"]) {
    SCOPED_TRACE(c["name"].as<std::string>());
    const ControlParams par = to_control_params(c["params"]);
    DockController ctl(par);
    const bool insert = c["op"].as<std::string>() == "insert";
    ASSERT_EQ(c["poses"].size(), c["expect"].size());
    for (size_t i = 0; i < c["poses"].size(); ++i) {
      const Pose2D p = to_pose(c["poses"][i]);
      const Command got = insert ? ctl.insert_command(p) : ctl.undock_command(p);
      const YAML::Node exp = c["expect"][i];
      EXPECT_NEAR(got.v, exp["v"].as<double>(), kTol) << i;
      EXPECT_NEAR(got.w, exp["w"].as<double>(), kTol) << i;
      EXPECT_EQ(got.done, exp["done"].as<bool>()) << i;
      EXPECT_EQ(got.failed, exp["failed"].as<bool>()) << i;
      EXPECT_LE(std::fabs(got.v), par.max_linear_speed + 1e-12);
      EXPECT_LE(std::fabs(got.w), par.max_angular_speed + 1e-12);
      if (insert) {
        EXPECT_TRUE(par.dock_backwards ? got.v <= 0.0 : got.v >= 0.0) << i;
      } else {
        EXPECT_TRUE(par.dock_backwards ? got.v >= 0.0 : got.v <= 0.0) << i;
      }
    }
  }
}

TEST(DockController, AlignerSequences)
{
  for (const auto & c : cases()["aligner"]) {
    SCOPED_TRACE(c["name"].as<std::string>());
    const ControlParams par = to_control_params(c["params"]);
    DockController ctl(par);
    for (size_t i = 0; i < c["poses"].size(); ++i) {
      const Command got = ctl.align_command(to_pose(c["poses"][i]));
      const YAML::Node exp = c["expect"][i];
      EXPECT_NEAR(got.v, exp["v"].as<double>(), kTol) << i;
      EXPECT_NEAR(got.w, exp["w"].as<double>(), kTol) << i;
      EXPECT_EQ(got.done, exp["done"].as<bool>()) << i;
      EXPECT_LE(std::fabs(got.v), par.max_linear_speed + 1e-12);
      EXPECT_LE(std::fabs(got.w), par.max_angular_speed + 1e-12);
      EXPECT_GE(got.v, 0.0);
    }
  }
}

TEST(DockController, PoseError)
{
  auto e = DockController::pose_error(Pose2D{0.03, 0.04, -0.2});
  EXPECT_NEAR(e.first, 0.05, kTol);
  EXPECT_NEAR(e.second, 0.2, kTol);
  e = DockController::pose_error(Pose2D{0.0, 0.0, 3.0 * M_PI});
  EXPECT_NEAR(e.second, M_PI, kTol);
}

TEST(DockController, ClosedLoopReachesCharger)
{
  const ControlParams par;
  const double starts[][3] =
  {{0.3, 0.2, 0.5}, {0.8, -0.3, -1.0}, {0.1, 0.1, 2.5}, {0.5, 0.05, 0.1}};
  for (const auto & s : starts) {
    DockController ctl(par);
    double x = s[0], y = s[1], yaw = s[2];
    const double dt = 0.05;
    bool aligned = false;
    bool docked = false;
    for (int i = 0; i < 8000 && !docked; ++i) {
      const Pose2D p{x, y, yaw};
      const Command cmd = aligned ? ctl.insert_command(p) : ctl.align_command(p);
      if (cmd.failed) {break;}
      if (cmd.done) {
        if (aligned) {docked = true;} else {aligned = true;}
        continue;
      }
      x += cmd.v * std::cos(yaw) * dt;
      y += cmd.v * std::sin(yaw) * dt;
      yaw = normalize_angle(yaw + cmd.w * dt);
    }
    ASSERT_TRUE(docked) << s[0] << " " << s[1] << " " << s[2];
    const auto err = DockController::pose_error(Pose2D{x, y, yaw});
    EXPECT_LT(err.first, 0.02);
    EXPECT_LT(err.second, 0.05);
  }
}

// ------------------------------------------------------------------ detection filter
TEST(DockFilter, Cases)
{
  for (const auto & c : cases()["filter"]) {
    SCOPED_TRACE(c["name"].as<std::string>());
    DetectionFilter flt(c["max_jump"].as<double>(), c["alpha"].as<double>());
    const auto res = flt.update(to_pose(c["estimate"]), to_pose(c["detection"]));
    EXPECT_EQ(res.first, c["expect_accepted"].as<bool>());
    expect_pose(res.second, c["expect"]);
  }
}

// ------------------------------------------------------------------ phase timer
TEST(DockTimer, Cases)
{
  for (const auto & c : cases()["timer"]) {
    SCOPED_TRACE(c["name"].as<std::string>());
    PhaseTimer timer(c["timeout"].as<double>(), c["clock_stall_timeout"].as<double>());
    timer.start(c["start"][0].as<double>(), c["start"][1].as<double>());
    auto run = [&timer](const YAML::Node & events) {
        for (const auto & e : events) {
          EXPECT_EQ(timer.expired(e[0].as<double>(), e[1].as<double>()), e[2].as<bool>())
            << e[0].as<double>() << " " << e[1].as<double>();
          EXPECT_EQ(timer.stalled(), e[3].as<bool>());
        }
      };
    run(c["events"]);
    if (c["restart"]) {
      timer.start(c["restart"][0].as<double>(), c["restart"][1].as<double>());
      EXPECT_FALSE(timer.stalled());
      run(c["events_after"]);
    }
  }
}

// ------------------------------------------------------------------ tracker
TEST(DockTrackerCases, AllCases)
{
  ASSERT_GT(cases()["tracker"].size(), 0u);
  for (const auto & c : cases()["tracker"]) {
    const std::string name = c["name"].as<std::string>();
    SCOPED_TRACE(name);
    DockTrackerParams params;
    for (const auto & kv : c["params"]) {
      const std::string k = kv.first.as<std::string>();
      if (k == "max_retries") {
        params.max_retries = kv.second.as<int>();
      } else if (k == "nav_max_retries") {
        params.nav_max_retries = kv.second.as<int>();
      } else if (k == "detection_mode") {
        params.detection_mode = kv.second.as<std::string>();
      } else if (k == "dock_position_tolerance") {
        params.dock_position_tolerance = kv.second.as<double>();
      } else if (k == "dock_yaw_tolerance") {
        params.dock_yaw_tolerance = kv.second.as<double>();
      } else {
        FAIL() << "unknown tracker param " << k;
      }
    }
    DockTracker tr(params);
    for (const auto & g : c["goals"]) {
      const std::string cmd = g["command"].as<std::string>();
      tr.start(
        cmd == "DOCK" ? COMMAND_DOCK : COMMAND_UNDOCK, g["docked"].as<bool>(),
        g["near"].as<bool>());
      if (g["start_expect"]) {check_fields(tr, g["start_expect"]);}
      std::vector<YAML::Node> events;
      for (const auto & e : g["events"]) {
        events.push_back(e);
      }
      size_t next = 0;
      std::vector<std::string> steps;
      bool finished = false;
      for (int i = 0; i < 100; ++i) {
        const DockStep step = tr.next_step();
        steps.push_back(fmt(step));
        if (i == 0 && g["first_step_expect"]) {check_fields(tr, g["first_step_expect"]);}
        if (step.kind == DockStepKind::FINISHED) {finished = true; break;}
        ASSERT_LT(next, events.size()) << "ran out of events";
        apply_event(tr, events[next++]);
      }
      ASSERT_TRUE(finished);
      EXPECT_EQ(next, events.size()) << "unused events";
      const YAML::Node exp = g["expect"];
      EXPECT_EQ(steps, exp["steps"].as<std::vector<std::string>>());
      check_fields(tr, exp);
      if (g["after_finish"]) {
        const int outcome = tr.outcome();
        const bool docked = tr.docked();
        const std::string state = tr.state();
        const int attempt = tr.attempt();
        for (const auto & e : g["after_finish"]) {
          apply_event(tr, e);
        }
        EXPECT_EQ(tr.outcome(), outcome);
        EXPECT_EQ(tr.docked(), docked);
        EXPECT_EQ(tr.state(), state);
        EXPECT_EQ(tr.attempt(), attempt);
      }
      if (tr.outcome() == DOCK_OUTCOME_CANCELED || tr.outcome() == DOCK_OUTCOME_ERROR) {
        EXPECT_FALSE(tr.docked());
      }
      EXPECT_TRUE(tr.finished());
    }
  }
}

TEST(DockTrackerCases, NextStepIsIdempotent)
{
  DockTracker tr;
  tr.start(COMMAND_DOCK, false, true);
  const DockStep a = tr.next_step();
  const DockStep b = tr.next_step();
  EXPECT_EQ(a.kind, b.kind);
  EXPECT_EQ(a.attempt, b.attempt);
}

TEST(DockTrackerCases, MessageSetOnFailure)
{
  DockTrackerParams params;
  params.max_retries = 0;
  DockTracker tr(params);
  tr.start(COMMAND_DOCK, false, true);
  tr.next_step();
  tr.report_detection(true);
  tr.next_step();
  tr.report_phase(PhaseResult::TIMEOUT);
  EXPECT_EQ(tr.outcome(), DOCK_OUTCOME_DOCKING_FAILED);
  EXPECT_FALSE(tr.message().empty());
}

TEST(DockTrackerCases, TableCoversRequiredScenarios)
{
  std::vector<std::string> names;
  for (const auto & c : cases()["tracker"]) {
    names.push_back(c["name"].as<std::string>());
  }
  for (const char * required : {
    "success_first_attempt", "near_staging_skips_nav", "nav_fails_all_attempts",
    "required_without_detection", "optional_without_detection_continues",
    "verify_failure_retry_then_success", "docking_failed_timeouts",
    "docking_failed_mixed_reasons", "cancel_during_navigation", "cancel_during_wait_detection",
    "cancel_during_align", "cancel_during_insert", "cancel_during_verify", "abort_during_align",
    "undock_success", "undock_failure_timeout", "dock_while_docked_is_already_docked",
    "reuse_after_failure_starts_clean"})
  {
    EXPECT_NE(std::find(names.begin(), names.end(), required), names.end()) << required;
  }
}

// The staging pose of the shipped config goes through the shared station loader
TEST(DockStations, ShippedDockStationGivesCharger)
{
  const StationSet st = load_stations(std::string(CONFIG_DIR) + "/stations.yaml");
  const Station * dock = st.find(st.dock_station);
  ASSERT_NE(dock, nullptr);
  const Pose2D charger = compute_dock_pose(Pose2D{dock->x, dock->y, dock->yaw}, 0.5, true);
  EXPECT_NEAR(charger.x, -0.5, kTol);
  EXPECT_NEAR(charger.y, 0.0, kTol);
  EXPECT_NEAR(charger.yaw, 0.0, kTol);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
