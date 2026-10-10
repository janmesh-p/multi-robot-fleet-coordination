#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "fleet_core/verification/invariant_checker.hpp"

namespace fleet_core {
namespace {

const GridMap kMap = GridMap::fromString(
    ".....\n"
    ".#...\n"
    ".....\n");

// Two robots; by default robot 0 moves (0,0)->(0,1) and robot 1 stays at (2,4).
struct Fixture {
  StepTruth step;
  TruthSnapshot after;
  TaskManager tasks;

  Fixture() {
    step.t = 5;
    step.motion = {MotionRecord{0, {0, 0}, {0, 1}, {0, 1}, MotionOutcome::kMoved},
                   MotionRecord{1, {2, 4}, {2, 4}, {2, 4}, MotionOutcome::kIdle}};
    after.cells = {{0, 1}, {2, 4}};
    after.alive = {true, true};
    after.status = {RobotStatus::kIdle, RobotStatus::kIdle};
    after.holding = {std::nullopt, std::nullopt};
  }

  std::vector<ViolationKind> check(bool conflict_free = true, int64_t issued = 0) {
    InvariantChecker checker(conflict_free);
    std::vector<ViolationKind> kinds;
    for (const Violation& v : checker.check(kMap, step, after, tasks, issued)) {
      EXPECT_EQ(v.t, 5);
      EXPECT_FALSE(v.detail.empty());
      kinds.push_back(v.kind);
    }
    return kinds;
  }
};

TEST(InvariantChecker, CleanStepPasses) {
  Fixture f;
  EXPECT_TRUE(f.check().empty());
}

TEST(InvariantChecker, PhysicalViolations) {
  {
    Fixture f;
    f.after.cells[1] = {0, 1};
    EXPECT_EQ(f.check(), std::vector<ViolationKind>{ViolationKind::kCellShared});
  }
  {
    Fixture f;
    f.step.motion[1] = MotionRecord{1, {0, 1}, {0, 0}, {0, 0}, MotionOutcome::kMoved};
    f.after.cells = {{0, 1}, {0, 0}};
    EXPECT_EQ(f.check(), std::vector<ViolationKind>{ViolationKind::kSwap});
  }
  {
    Fixture f;
    f.step.motion[0].to = {0, 3};
    f.after.cells[0] = {0, 3};
    EXPECT_EQ(f.check(), std::vector<ViolationKind>{ViolationKind::kTeleport});
  }
  {
    Fixture f;
    f.after.cells[1] = {1, 1};  // shelf
    EXPECT_EQ(f.check(), std::vector<ViolationKind>{ViolationKind::kOffFreeCell});
  }
  {
    Fixture f;
    f.after.alive[0] = false;
    EXPECT_EQ(f.check(), std::vector<ViolationKind>{ViolationKind::kDeadRobotMoved});
  }
}

TEST(InvariantChecker, CoordinationViolations) {
  {
    Fixture f;
    f.step.motion[1].outcome = MotionOutcome::kInvalidStep;
    f.step.motion[1].intended = {0, 4};
    EXPECT_EQ(f.check(false), std::vector<ViolationKind>{ViolationKind::kInvalidPlan});
  }
  {
    Fixture f;
    f.step.motion[1].outcome = MotionOutcome::kBlockedOccupied;
    EXPECT_EQ(f.check(true), std::vector<ViolationKind>{ViolationKind::kCoordinationConflict});
    EXPECT_TRUE(f.check(false).empty());  // the uncoordinated baseline is allowed conflicts
  }
}

TEST(InvariantChecker, TaskViolations) {
  {
    Fixture f;
    f.after.holding = {TaskId{3}, TaskId{3}};
    EXPECT_EQ(f.check(), std::vector<ViolationKind>{ViolationKind::kTaskHeldTwice});
  }
  {
    Fixture f;
    InvariantChecker checker(true);
    f.step.deliveries = {Assignment{0, 3}};
    EXPECT_TRUE(checker.check(kMap, f.step, f.after, f.tasks, 0).empty());
    const std::vector<Violation> again = checker.check(kMap, f.step, f.after, f.tasks, 0);
    ASSERT_EQ(again.size(), 1U);
    EXPECT_EQ(again[0].kind, ViolationKind::kTaskDeliveredTwice);
  }
  {
    Fixture f;
    Task t;
    t.order.id = 0;
    f.tasks.add(t);
    f.tasks.assign(0, 0, 1);
    f.tasks.markPickedUp(0, 2);
    f.tasks.markDone(0, 3);  // the coordinator says done, but no robot delivered it
    EXPECT_EQ(f.check(true, 1), std::vector<ViolationKind>{ViolationKind::kPhantomDelivery});
  }
  {
    Fixture f;
    EXPECT_EQ(f.check(true, 2), std::vector<ViolationKind>{ViolationKind::kTaskLost});
  }
}

}  // namespace
}  // namespace fleet_core
