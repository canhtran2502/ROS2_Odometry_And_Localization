// AC2 and AC4 (C++ side): station loading, mission validation and MissionTracker cases.
// Executes the same data files as test_mission_logic.py.
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "bumperbot_navigation/mission_logic.hpp"

using namespace bumperbot_navigation;  // NOLINT

namespace
{
const std::string kData = TEST_DATA_DIR;
const std::string kConfig = CONFIG_DIR;
}  // namespace

TEST(StationsTest, ShippedFileLoads)
{
  StationSet st = load_stations(kConfig + "/stations.yaml");
  EXPECT_EQ(st.frame_id, "map");
  EXPECT_EQ(st.dock_station, "dock");
  EXPECT_NE(st.find("dock"), nullptr);
  EXPECT_GE(st.stations.size(), 4u);
}

TEST(StationsTest, TestFileValues)
{
  StationSet st = load_stations(kData + "/stations_test.yaml");
  EXPECT_EQ(st.frame_id, "map");
  EXPECT_EQ(st.dock_station, "dock");
  struct Exp {const char * n; double x, y, yaw, dwell;};
  const Exp expected[] = {
    {"dock", 0.0, 0.0, 0.0, 0.0}, {"a", 1.0, 0.5, 0.5, 1.0}, {"b", 2.0, -1.0, 1.57, 0.2},
    {"c", -1.5, 2.0, 3.14, 0.2}, {"u1", 3.0, 3.0, 0.0, 0.2}, {"u2", -3.0, -3.0, -1.0, 0.2}};
  EXPECT_EQ(st.stations.size(), 6u);
  for (const auto & e : expected) {
    const Station * s = st.find(e.n);
    ASSERT_NE(s, nullptr) << e.n;
    EXPECT_DOUBLE_EQ(s->x, e.x);
    EXPECT_DOUBLE_EQ(s->y, e.y);
    EXPECT_DOUBLE_EQ(s->yaw, e.yaw);
    EXPECT_DOUBLE_EQ(s->dwell, e.dwell);
  }
}

TEST(StationsTest, InvalidFixturesRejected)
{
  const std::vector<std::string> files = {
    "invalid_empty.yaml", "invalid_missing_field.yaml", "invalid_nan.yaml",
    "invalid_negative_dwell.yaml", "invalid_no_dock.yaml", "invalid_not_a_mapping.yaml",
    "does_not_exist.yaml"};
  for (const auto & f : files) {
    EXPECT_THROW(load_stations(kData + "/" + f), std::invalid_argument) << f;
  }
}

TEST(ValidateTest, EmptyAndUnknown)
{
  StationSet st = load_stations(kData + "/stations_test.yaml");
  EXPECT_EQ(validate_mission({"a", "b"}, st), "");
  EXPECT_NE(validate_mission({}, st), "");
  EXPECT_NE(validate_mission({"a", "nowhere"}, st).find("nowhere"), std::string::npos);
}

TEST(ValidateTest, Helpers)
{
  auto q = yaw_to_quaternion(1.0);
  EXPECT_NEAR(q.first, std::sin(0.5), 1e-12);
  EXPECT_NEAR(q.second, std::cos(0.5), 1e-12);
  EXPECT_NEAR(planar_distance(0, 0, 3, 4), 5.0, 1e-12);
}

namespace
{
std::string fmt(const Step & s)
{
  if (s.kind == StepKind::FINISHED) {return "FINISHED";}
  if (s.kind == StepKind::DWELL) {return "DWELL:" + s.station;}
  return "NAVIGATE:" + s.station + ":" + std::to_string(s.attempt) + (s.docking ? ":dock" : "");
}
}  // namespace

TEST(MissionCases, AllCases)
{
  YAML::Node doc = YAML::LoadFile(kData + "/mission_cases.yaml");
  StationSet st = load_stations(kData + "/" + doc["stations_file"].as<std::string>());
  ASSERT_GT(doc["cases"].size(), 0u);
  for (const auto & c : doc["cases"]) {
    const std::string name = c["name"].as<std::string>();
    SCOPED_TRACE(name);
    TrackerParams p;
    if (c["params"]) {
      for (const auto & kv : c["params"]) {
        const std::string k = kv.first.as<std::string>();
        if (k == "max_retries") {
          p.max_retries = kv.second.as<int>();
        } else if (k == "max_consecutive_failures") {
          p.max_consecutive_failures = kv.second.as<int>();
        } else if (k == "dock_max_retries") {
          p.dock_max_retries = kv.second.as<int>();
        } else if (k == "return_to_dock") {
          p.return_to_dock = kv.second.as<bool>();
        } else if (k == "goal_tolerance") {
          p.goal_tolerance = kv.second.as<double>();
        } else {
          FAIL() << "unknown param " << k;
        }
      }
    }
    std::vector<std::string> names = c["stations"].as<std::vector<std::string>>();
    MissionTracker tr(st, names, p);
    std::vector<YAML::Node> events;
    for (const auto & e : c["events"]) {
      events.push_back(e);
    }
    size_t ei = 0;
    std::vector<std::string> steps;
    double last_progress = -1.0;
    bool finished = false;
    for (int i = 0; i < 200; ++i) {
      Step step = tr.next_step();
      steps.push_back(fmt(step));
      EXPECT_GE(tr.progress(), last_progress - 1e-9);
      last_progress = tr.progress();
      if (step.kind == StepKind::FINISHED) {finished = true; break;}
      ASSERT_LT(ei, events.size()) << "ran out of events";
      YAML::Node ev = events[ei++];
      if (ev["cancel"]) {
        tr.request_cancel();
      } else if (ev["nav"]) {
        ASSERT_EQ(step.kind, StepKind::NAVIGATE);
        double err = ev["nav"]["err"].as<double>();
        tr.report_navigation(
          ev["nav"]["ok"].as<bool>(),
          err < 0 ? std::numeric_limits<double>::quiet_NaN() : err);
      } else if (ev["dwell_done"]) {
        ASSERT_EQ(step.kind, StepKind::DWELL);
        tr.report_dwell_done();
      }
    }
    ASSERT_TRUE(finished);
    YAML::Node exp = c["expect"];
    EXPECT_EQ(steps, exp["steps"].as<std::vector<std::string>>());
    EXPECT_EQ(tr.outcome(), exp["outcome"].as<int>());
    EXPECT_EQ(tr.docked(), exp["docked"].as<bool>());
    EXPECT_EQ(tr.visited(), exp["visited"].as<std::vector<std::string>>());
    EXPECT_EQ(tr.skipped(), exp["skipped"].as<std::vector<std::string>>());
    if (exp["state"]) {EXPECT_EQ(tr.state(), exp["state"].as<std::string>());}
    if (exp["progress"]) {EXPECT_NEAR(tr.progress(), exp["progress"].as<double>(), 1e-9);}
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
