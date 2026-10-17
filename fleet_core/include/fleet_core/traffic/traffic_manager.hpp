#pragma once

#include <cstdint>
#include <map>
#include <vector>

#include "fleet_core/common/types.hpp"
#include "fleet_core/world/world_model.hpp"

namespace fleet_core {

// A plan the coordinator has sent to a robot.
struct CommittedPlan {
  RobotId robot{-1};
  int32_t plan_id{-1};
  Path path{};
};

// A robot's position along a plan: the path index it last reported for that plan, or 0 when it
// has not reported on it yet (a plan starts where the robot was). Clamped to the path.
int32_t pathProgress(const RobotBelief& robot, int32_t plan_id, const std::vector<Cell>& path);

// How far one robot may go along its committed path, decided every tick.
struct Permit {
  int32_t until{-1};      // last path index it may reach; -1 without a plan
  RobotId follow{-1};     // its next move follows this robot out of its cell (RobotCommand)
  bool held{false};       // its path goes on, but it may not take the next step
  RobotId waits_for{-1};  // held because of this robot; -1 when a closed cell holds it
};

struct TrafficDecision {
  std::vector<Permit> permits{};               // index == RobotId
  std::vector<std::vector<RobotId>> cycles{};  // held robots waiting on each other, ascending
};

// Which rules apply. FleetManager turns the feature toggles into these: both are for coordinated
// planners only, and the dependency graph needs rolling replanning.
struct TrafficRules {
  bool dependency_graph{false};  // features.dependency_graph
  bool reservations{false};      // features.intersection_reservations
};

// Turns committed plans into motion permits: how far along its path each robot may go this tick.
//
// Without rules (the uncoordinated baseline) a robot may follow its whole path, stopping only
// before a closed cell. The rules keep plans safe when robots run late:
//
//   Dependency graph (Hönig et al. 2019). For every cell, the order in which the plans visit it.
//   A robot may enter a cell only after the robot planned there before it has left it, or while
//   that robot leaves it in the same step, which it then follows. A late robot holds up only
//   the robots planned after it in some cell, so plans that are conflict-free stay that way
//   however late anyone runs. Robots that each enter the next one's cell in a ring move together
//   or not at all. It needs plans made together, which rolling replanning provides.
//
//   Reservations. Checked against where the robots are, not where the plans put them. A robot
//   may enter a cell only while it holds the cell's reservation, which it gets when the cell is
//   empty or its robot leaves in the same step, one robot per cell per step, older order first.
//   Any plans will do, so it also covers replanning a few robots at a time. But it does not keep
//   the planned order, so two robots can end up facing each other.
//
// A held robot waits for at most one other robot. Held robots that wait for each other in a
// cycle cannot move until a plan changes: decide() reports the cycles, and the coordinator
// replans (features.deadlock_recovery).
class TrafficManager {
 public:
  explicit TrafficManager(TrafficRules rules);

  // Replaces the committed plan of each listed robot. With the dependency graph, rebuilds it from
  // every committed plan.
  void commit(const std::vector<CommittedPlan>& plans);

  // Drops a robot's plan (robot dead or plan finished).
  void release(RobotId robot);

  // The robot's position along its committed plan (pathProgress); 0 without one.
  int32_t progress(const WorldModel& world, RobotId robot) const;

  // Permits for the whole fleet (index == RobotId; robots without a plan get until -1).
  // priority: per robot, the higher value wins a contested cell (the age of its order, -1 without
  // one); ties go to the lower id.
  TrafficDecision decide(const WorldModel& world, const std::vector<int32_t>& priority) const;

  const TrafficRules& rules() const { return rules_; }

 private:
  // The robot that must have left a cell before a move into it, and the path index it reaches
  // by leaving. robot -1: no one.
  struct Dependency {
    RobotId robot{-1};
    int32_t leave{0};
  };

  void rebuildGraph();

  TrafficRules rules_;
  std::map<RobotId, CommittedPlan> plans_{};
  std::map<RobotId, std::vector<Dependency>> graph_{};  // per robot, per path index
};

}  // namespace fleet_core
