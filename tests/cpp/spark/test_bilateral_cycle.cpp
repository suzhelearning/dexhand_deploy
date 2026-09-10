#include "tianji_spark/bilateral_cycle.hpp"
#include <gtest/gtest.h>

namespace tianji_spark {
namespace {
const std::string root = TIANJI_PROJECT_SOURCE_DIR;
const std::string model = root + "/src/tianji_teleop/assets/spark/marvin_m6_wuji2.xml";
const std::string urdf = root + "/src/tianji_teleop/assets/spark/marvin_m6_s_ccs_696_v4_local.urdf";
QpIkConfig configuration() {
  auto c = loadConfig(root + "/src/tianji_teleop/config/producers/spark_reference.yaml");
  // Unit tests test deterministic logic, not the reference's wall-clock budget.
  c.spark_upper_qpoases.ik_cycle_budget_seconds = 1.0;
  return c;
}

TEST(SparkBilateralCycle, TicksOnceAndRejectsDuplicateBeforeAdvancingState) {
  NativeSparkCycle cycle(configuration(), model, urdf, true);
  const auto first = cycle.step(1, 1000000000LL, std::nullopt);
  EXPECT_EQ(first.guidance_updates, 1U);
  EXPECT_EQ(first.headroom_updates, 1U);
  EXPECT_TRUE(first.left.q.allFinite());
  EXPECT_THROW(cycle.step(1, 1005000000LL, std::nullopt), std::invalid_argument);
  const auto second = cycle.step(2, 1005000000LL, std::nullopt);
  EXPECT_EQ(second.guidance_updates, 2U);
  EXPECT_EQ(second.headroom_updates, 2U);
  EXPECT_FALSE(second.freshness.live);
}

TEST(SparkBilateralCycle, DisabledDoesNotAdvanceControllerOrHeadroom) {
  NativeSparkCycle cycle(configuration(), model, urdf, false);
  const auto first = cycle.step(1, 1000000000LL, std::nullopt);
  EXPECT_FALSE(first.control_executed);
  EXPECT_EQ(first.headroom_updates, 0U);
  EXPECT_GT(first.control.left.target.position.norm(), 0.1);
  EXPECT_EQ(first.control.hold_reason, HoldReason::kNone);
  cycle.set_enabled(true);
  const auto second = cycle.step(2, 1005000000LL, std::nullopt);
  EXPECT_TRUE(second.control_executed);
  EXPECT_EQ(second.headroom_updates, 1U);
}

TEST(SparkBilateralCycle, ButtonChangesToggleAndEpochChangeDoesNotToggle) {
  NativeSparkCycle cycle(configuration(), model, urdf, true);
  PicoTeleopFrame frame;
  frame.tracking_epoch = 9;
  frame.sequence = 1;
  frame.receive_monotonic_ns = 1000000000LL;
  auto result = cycle.step(1, frame.receive_monotonic_ns, frame);
  EXPECT_EQ(result.button_action, PicoTeleopButtonAction::kNone);
  frame.sequence = 2;
  frame.user_button_pressed = true;
  frame.receive_monotonic_ns += 5000000LL;
  result = cycle.step(2, frame.receive_monotonic_ns, frame);
  EXPECT_EQ(result.button_action, PicoTeleopButtonAction::kPause);
  EXPECT_FALSE(result.control_executed);
  frame.sequence = 3;
  frame.user_button_pressed = false;
  frame.receive_monotonic_ns += 5000000LL;
  result = cycle.step(3, frame.receive_monotonic_ns, frame);
  EXPECT_EQ(result.button_action, PicoTeleopButtonAction::kResume);
  EXPECT_TRUE(result.control_executed);
  frame.tracking_epoch = 10;
  frame.sequence = 1;
  frame.user_button_pressed = true;
  frame.receive_monotonic_ns += 5000000LL;
  result = cycle.step(4, frame.receive_monotonic_ns, frame);
  EXPECT_EQ(result.button_action, PicoTeleopButtonAction::kNone);
}

TEST(SparkBilateralCycle, RejectsOtherAlgorithmAndActualFeedbackMode) {
  auto config = configuration();
  config.controller.model_state_only = false;
  EXPECT_THROW(NativeSparkCycle(config, model, urdf, true), std::invalid_argument);
  config = configuration();
  config.ik_algorithm = IkAlgorithm::kHierarchicalQp;
  EXPECT_THROW(NativeSparkCycle(config, model, urdf, true), std::invalid_argument);
}
}  // namespace
}  // namespace tianji_spark
