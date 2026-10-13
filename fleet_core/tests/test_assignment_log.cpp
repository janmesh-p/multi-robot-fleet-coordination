#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "fleet_core/metrics/assignment_log.hpp"

namespace fleet_core {
namespace {

// Ground truth for one step: robot i moves from from[i] to to[i].
StepTruth step(Timestep t, const std::vector<std::pair<Cell, Cell>>& moves,
               const std::vector<Assignment>& pickups = {},
               const std::vector<Assignment>& deliveries = {}) {
  StepTruth s;
  s.t = t;
  for (size_t i = 0; i < moves.size(); ++i) {
    MotionRecord m;
    m.robot = static_cast<RobotId>(i);
    m.from = moves[i].first;
    m.to = moves[i].second;
    s.motion.push_back(m);
  }
  s.pickups = pickups;
  s.deliveries = deliveries;
  return s;
}

TickOutput assigned(Timestep t, TaskId order, RobotId robot, Cell pickup, int32_t dist) {
  TickOutput out;
  AssignmentRecord r;
  r.t = t;
  r.order = order;
  r.robot = robot;
  r.pickup = pickup;
  r.features.dist_to_pickup = dist;
  out.assignments.push_back(r);
  return out;
}

TEST(AssignmentLog, TracksArrivalPickupAndDelivery) {
  AssignmentLog log;
  const Cell pickup{0, 3};
  log.record(assigned(5, 0, 0, pickup, 3), step(5, {{{0, 0}, {0, 1}}}));
  log.record(TickOutput{}, step(6, {{{0, 1}, {0, 2}}}));
  log.record(TickOutput{}, step(7, {{{0, 2}, pickup}}));
  log.record(TickOutput{}, step(8, {{pickup, pickup}}));
  log.record(TickOutput{}, step(9, {{pickup, pickup}}, {{0, 0}}));
  ASSERT_EQ(log.rows().size(), 1U);
  EXPECT_EQ(log.rows()[0].arrived_t, 7);
  EXPECT_EQ(log.rows()[0].steps_to_pickup, 3);  // the distance: nothing slowed it
  EXPECT_EQ(log.rows()[0].end, AssignmentEnd::kPickedUp);
  EXPECT_EQ(log.rows()[0].delivered_t, -1);
  log.record(TickOutput{}, step(20, {{{0, 9}, {0, 9}}}, {}, {{0, 0}}));
  EXPECT_EQ(log.rows()[0].delivered_t, 20);
  EXPECT_EQ(log.rows()[0].end, AssignmentEnd::kDelivered);
}

TEST(AssignmentLog, RobotAlreadyOnThePickup) {
  AssignmentLog log;
  const Cell pickup{2, 0};
  log.record(assigned(4, 7, 0, pickup, 0), step(4, {{pickup, pickup}}));
  EXPECT_EQ(log.rows()[0].arrived_t, 3);
  EXPECT_EQ(log.rows()[0].steps_to_pickup, 0);
}

TEST(AssignmentLog, NeverArrivesStaysMinusOne) {
  AssignmentLog log;
  log.record(assigned(0, 1, 0, {0, 5}, 5), step(0, {{{0, 0}, {0, 1}}}));
  for (Timestep t = 1; t < 10; ++t) log.record(TickOutput{}, step(t, {{{0, 1}, {0, 1}}}));
  EXPECT_EQ(log.rows()[0].arrived_t, -1);
  EXPECT_EQ(log.rows()[0].steps_to_pickup, -1);
  EXPECT_EQ(log.rows()[0].end, AssignmentEnd::kAssigned);
}

TEST(AssignmentLog, RequeuedOrderEndsItsRowAndGetsANewOne) {
  AssignmentLog log;
  log.record(assigned(1, 3, 0, {0, 5}, 4), step(1, {{{0, 0}, {0, 1}}, {{0, 9}, {0, 9}}}));
  TickOutput again = assigned(4, 3, 1, {0, 5}, 4);
  again.requeued = {3};
  log.record(again, step(4, {{{0, 1}, {0, 1}}, {{0, 9}, {0, 8}}}));
  const std::vector<AssignmentRow> rows = log.rows();
  ASSERT_EQ(rows.size(), 2U);
  EXPECT_EQ(rows[0].record.robot, 0);
  EXPECT_EQ(rows[0].end, AssignmentEnd::kRequeued);
  EXPECT_EQ(rows[1].record.robot, 1);
  EXPECT_EQ(rows[1].end, AssignmentEnd::kAssigned);
  // The first robot reaching the pickup later does not touch its closed row.
  log.record(TickOutput{}, step(5, {{{0, 1}, {0, 5}}, {{0, 8}, {0, 7}}}));
  EXPECT_EQ(log.rows()[0].arrived_t, -1);
}

TEST(AssignmentLog, RowsSortByStepThenRobot) {
  AssignmentLog log;
  TickOutput two = assigned(2, 10, 1, {5, 0}, 1);
  two.assignments.push_back(assigned(2, 11, 0, {6, 0}, 1).assignments[0]);
  log.record(two, step(2, {{{0, 0}, {0, 0}}, {{0, 1}, {0, 1}}}));
  log.record(assigned(3, 12, 0, {7, 0}, 1), step(3, {{{0, 0}, {0, 0}}, {{0, 1}, {0, 1}}}));
  const std::vector<AssignmentRow> rows = log.rows();
  ASSERT_EQ(rows.size(), 3U);
  EXPECT_EQ(rows[0].record.order, 11);
  EXPECT_EQ(rows[1].record.order, 10);
  EXPECT_EQ(rows[2].record.order, 12);
}

// The header and the example row of docs/learning_track.md (section 3), verbatim.
TEST(AssignmentLog, CsvMatchesTheContract) {
  EXPECT_EQ(assignmentCsvHeader(),
            "t,order,robot,dist_to_pickup,manhattan_to_pickup,dist_pickup_to_dropoff,robot_row,"
            "robot_col,pickup_row,pickup_col,robot_on_endpoint,robots_near_robot,"
            "robots_near_pickup,robots_in_box,box_free_cells,pickup_assigned,pickup_pending,"
            "pending_orders,available_robots,busy_robots,alive_robots,order_age,arrived_t,"
            "steps_to_pickup,delivered_t,end_status");
  AssignmentRow row;
  row.record.t = 612;
  row.record.order = 57;
  row.record.robot = 3;
  row.record.features = PairFeatures{14, 14, 39, 0, 9, 5, 0, 1, 2, 1, 2, 39, 1, 0, 3, 4, 6, 10, 5};
  row.arrived_t = 631;
  row.steps_to_pickup = 20;
  row.delivered_t = 674;
  row.end = AssignmentEnd::kDelivered;
  EXPECT_EQ(assignmentCsvRow(row),
            "612,57,3,14,14,39,0,9,5,0,1,2,1,2,39,1,0,3,4,6,10,5,631,20,674,delivered");
  EXPECT_EQ(toString(AssignmentEnd::kPickedUp), "picked_up");
  EXPECT_EQ(toString(AssignmentEnd::kRequeued), "requeued");
  EXPECT_EQ(toString(AssignmentEnd::kAssigned), "assigned");
}

}  // namespace
}  // namespace fleet_core
