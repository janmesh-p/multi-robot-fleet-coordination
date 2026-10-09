#include <vector>

#include <gtest/gtest.h>

#include "fleet_core/faults/fault_injector.hpp"

namespace fleet_core {
namespace {

FaultEvent death(Timestep at, RobotId robot) {
  return FaultEvent{at, FaultType::kRobotDeath, robot, {}, 0};
}

TEST(FaultInjector, ReleasesEventsInTimeOrder) {
  const FaultEvent block{5, FaultType::kBlockCells, -1, {{2, 2}}, 0};
  FaultInjector injector({death(10, 3), block});
  EXPECT_TRUE(injector.due(4).empty());
  const std::vector<FaultEvent> at5 = injector.due(5);
  ASSERT_EQ(at5.size(), 1U);
  EXPECT_EQ(at5[0].type, FaultType::kBlockCells);
  EXPECT_TRUE(injector.due(9).empty());
  const std::vector<FaultEvent> at20 = injector.due(20);
  ASSERT_EQ(at20.size(), 1U);
  EXPECT_EQ(at20[0].robot, 3);
  EXPECT_TRUE(injector.due(100).empty());
}

TEST(FaultInjector, TiesKeepConfigOrder) {
  FaultInjector injector({death(7, 1), death(7, 0), death(7, 2)});
  const std::vector<FaultEvent> due = injector.due(7);
  ASSERT_EQ(due.size(), 3U);
  EXPECT_EQ(due[0].robot, 1);
  EXPECT_EQ(due[1].robot, 0);
  EXPECT_EQ(due[2].robot, 2);
}

TEST(FaultInjector, CatchesUpAfterSkippedSteps) {
  FaultInjector injector({death(3, 0), death(1, 1), death(2, 2)});
  const std::vector<FaultEvent> due = injector.due(10);
  ASSERT_EQ(due.size(), 3U);
  EXPECT_EQ(due[0].at, 1);
  EXPECT_EQ(due[1].at, 2);
  EXPECT_EQ(due[2].at, 3);
}

}  // namespace
}  // namespace fleet_core
