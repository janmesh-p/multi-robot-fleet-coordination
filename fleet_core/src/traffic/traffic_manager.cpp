#include "fleet_core/traffic/traffic_manager.hpp"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <tuple>
#include <utility>

namespace fleet_core {
namespace {

// The path index a robot reaches by leaving a cell its path ends in: never.
constexpr int32_t kNever = std::numeric_limits<int32_t>::max();

// One stay of one robot in one cell, in plan time.
struct Visit {
  Timestep from{0};  // the step it arrives: plan start + first
  RobotId robot{-1};
  int32_t first{0};  // path index of arrival
  int32_t leave{0};  // path index it reaches by leaving; kNever if its path ends there
};

// One robot's next step this tick.
struct Next {
  bool exists{false};  // its path goes on
  bool move{false};    // false: a planned wait
  Cell to{};
  RobotId after_adg{-1};  // may move only if this robot leaves the cell in the same step
  RobotId after_res{-1};  // the same, from the reservation check
  bool held{false};
  RobotId waits_for{-1};
};

}  // namespace

int32_t pathProgress(const RobotBelief& robot, int32_t plan_id, const std::vector<Cell>& path) {
  if (path.empty() || robot.plan_id != plan_id) return 0;
  return std::clamp(robot.path_index, 0, static_cast<int32_t>(path.size()) - 1);
}

TrafficManager::TrafficManager(TrafficRules rules) : rules_(rules) {}

void TrafficManager::commit(const std::vector<CommittedPlan>& plans) {
  for (const CommittedPlan& plan : plans) plans_[plan.robot] = plan;
  if (rules_.dependency_graph && !plans.empty()) rebuildGraph();
}

void TrafficManager::release(RobotId robot) {
  plans_.erase(robot);
  if (rules_.dependency_graph) rebuildGraph();
}

// Every committed plan, split into stays, sorted per cell by arrival in plan time. A move into a
// cell depends on the stay just before it there, unless that stay is the robot's own.
void TrafficManager::rebuildGraph() {
  std::map<Cell, std::vector<Visit>> visits;
  for (const auto& [robot, plan] : plans_) {
    const std::vector<Cell>& cells = plan.path.cells;
    const auto n = static_cast<int32_t>(cells.size());
    for (int32_t k = 0; k < n;) {
      int32_t j = k;
      while (j + 1 < n && cells[static_cast<size_t>(j + 1)] == cells[static_cast<size_t>(k)]) ++j;
      visits[cells[static_cast<size_t>(k)]].push_back(
          Visit{plan.path.start + k, robot, k, j + 1 < n ? j + 1 : kNever});
      k = j + 1;
    }
  }
  graph_.clear();
  for (const auto& [robot, plan] : plans_) {
    graph_[robot].assign(plan.path.cells.size(), Dependency{});
  }
  for (auto& [cell, list] : visits) {
    std::sort(list.begin(), list.end(), [](const Visit& a, const Visit& b) {
      return std::tie(a.from, a.robot) < std::tie(b.from, b.robot);
    });
    for (size_t v = 1; v < list.size(); ++v) {
      const Visit& before = list[v - 1];
      const Visit& visit = list[v];
      if (visit.first == 0 || before.robot == visit.robot) continue;
      graph_[visit.robot][static_cast<size_t>(visit.first)] =
          Dependency{before.robot, before.leave};
    }
  }
}

int32_t TrafficManager::progress(const WorldModel& world, RobotId robot) const {
  const auto it = plans_.find(robot);
  if (it == plans_.end()) return 0;
  return pathProgress(world.robot(robot), it->second.plan_id, it->second.path.cells);
}

TrafficDecision TrafficManager::decide(const WorldModel& world,
                                       const std::vector<int32_t>& priority) const {
  const GridMap& map = world.map();
  const auto n = static_cast<size_t>(world.fleetSize());
  const bool rules = rules_.dependency_graph || rules_.reservations;
  TrafficDecision out;
  out.permits.assign(n, Permit{});

  std::map<Cell, RobotId> occupant;  // believed positions; dead robots still occupy their cells
  for (const RobotBelief& b : world.robots()) occupant[b.cell] = b.id;

  const auto planOf = [&](RobotId r) -> const CommittedPlan* {
    const auto it = plans_.find(r);
    return it == plans_.end() || it->second.path.cells.empty() ? nullptr : &it->second;
  };
  const auto rank = [&](RobotId r) {
    return static_cast<size_t>(r) < priority.size() ? priority[static_cast<size_t>(r)] : -1;
  };
  // The dependency of the move to path index `index`, strictly met: the robot before has left.
  const auto departed = [&](RobotId r, int32_t index) {
    const Dependency& dep = graph_.at(r)[static_cast<size_t>(index)];
    return dep.robot < 0 || dep.robot == r || progress(world, dep.robot) >= dep.leave;
  };

  // 1. Each robot's next step, checked alone.
  std::vector<int32_t> at(n, 0);
  std::vector<Next> next(n);
  for (size_t i = 0; i < n; ++i) {
    const auto r = static_cast<RobotId>(i);
    const CommittedPlan* plan = planOf(r);
    if (plan == nullptr) continue;
    const std::vector<Cell>& path = plan->path.cells;
    const int32_t k = progress(world, r);
    at[i] = k;
    Permit& permit = out.permits[i];
    permit.until = k;
    if (k + 1 >= static_cast<int32_t>(path.size())) continue;
    Next& step = next[i];
    step.exists = true;
    step.to = path[static_cast<size_t>(k + 1)];
    step.move = step.to != path[static_cast<size_t>(k)];
    if (rules && path[static_cast<size_t>(k)] != world.robot(r).cell) {
      step.held = true;  // not where its plan says: wait for a new plan
      continue;
    }
    if (!step.move) continue;
    if (map.isBlocked(step.to)) {
      step.held = true;
      continue;
    }
    if (rules_.dependency_graph) {
      const Dependency& dep = graph_.at(r)[static_cast<size_t>(k + 1)];
      if (dep.robot >= 0 && dep.robot != r) {
        const int32_t p = progress(world, dep.robot);
        if (p == dep.leave - 1) {
          step.after_adg = dep.robot;  // it leaves in this step, at best
        } else if (p < dep.leave) {
          step.held = true;
          step.waits_for = dep.robot;
          continue;
        }
      }
    }
    if (rules_.reservations) {
      const auto o = occupant.find(step.to);
      if (o != occupant.end() && o->second != r) step.after_res = o->second;
    }
  }

  if (rules) {
    // 2. Reservations: the robot in the wanted cell must be leaving, and one robot per cell.
    if (rules_.reservations) {
      for (size_t i = 0; i < n; ++i) {
        Next& step = next[i];
        if (!step.exists || !step.move || step.held || step.after_res < 0) continue;
        const Next& other = next[static_cast<size_t>(step.after_res)];
        if (!other.exists || !other.move) {
          step.held = true;
          step.waits_for = step.after_res;
        }
      }
      std::map<Cell, RobotId> winner;
      for (size_t i = 0; i < n; ++i) {
        const Next& step = next[i];
        if (!step.exists || !step.move || step.held) continue;
        const auto r = static_cast<RobotId>(i);
        const auto [it, inserted] = winner.emplace(step.to, r);
        if (!inserted &&
            std::make_pair(rank(r), -r) > std::make_pair(rank(it->second), -it->second))
          it->second = r;
      }
      for (size_t i = 0; i < n; ++i) {
        Next& step = next[i];
        if (!step.exists || !step.move || step.held) continue;
        const RobotId w = winner.at(step.to);
        if (w != static_cast<RobotId>(i)) {
          step.held = true;
          step.waits_for = w;
        }
      }
    }
    // 3. Two robots that would swap cells both wait: no rule lets robots pass through each other.
    for (size_t i = 0; i < n; ++i) {
      Next& step = next[i];
      if (!step.exists || !step.move || step.held) continue;
      const auto o = occupant.find(step.to);
      if (o == occupant.end() || o->second == static_cast<RobotId>(i)) continue;
      Next& other = next[static_cast<size_t>(o->second)];
      if (other.exists && other.move && !other.held &&
          other.to == world.robot(static_cast<RobotId>(i)).cell) {
        step.held = true;
        step.waits_for = o->second;
        other.held = true;
        other.waits_for = static_cast<RobotId>(i);
      }
    }
    // 4. A move that depends on another robot leaving goes only if that robot's move goes. Rings
    //    of such moves stay: they all move together.
    for (bool changed = true; changed;) {
      changed = false;
      for (size_t i = 0; i < n; ++i) {
        Next& step = next[i];
        if (!step.exists || !step.move || step.held) continue;
        for (const RobotId leader : {step.after_adg, step.after_res}) {
          if (leader < 0) continue;
          const Next& lead = next[static_cast<size_t>(leader)];
          if (!lead.exists || !lead.move || lead.held) {
            step.held = true;
            step.waits_for = leader;
            changed = true;
            break;
          }
        }
      }
    }
  }

  // 5. Permits. With reservations a robot gets one step at a time; otherwise it may go on while
  //    every move is already safe.
  for (size_t i = 0; i < n; ++i) {
    const auto r = static_cast<RobotId>(i);
    const Next& step = next[i];
    Permit& permit = out.permits[i];
    if (!step.exists) continue;
    if (step.held) {
      permit.held = true;
      permit.waits_for = step.waits_for;
      continue;
    }
    permit.until = at[i] + 1;
    if (step.move) permit.follow = step.after_res >= 0 ? step.after_res : step.after_adg;
    if (rules_.reservations) continue;
    const std::vector<Cell>& path = planOf(r)->path.cells;
    for (auto b = static_cast<size_t>(permit.until) + 1; b < path.size(); ++b) {
      if (path[b] != path[b - 1]) {
        if (map.isBlocked(path[b])) break;
        if (rules_.dependency_graph && !departed(r, static_cast<int32_t>(b))) break;
      }
      permit.until = static_cast<int32_t>(b);
    }
  }

  // 6. Cycles among held robots. Each waits for at most one robot, so walk each chain until it
  //    ends or comes back to itself. state: 0 unseen, 1 on the current walk, 2 done.
  std::vector<uint8_t> state(n, 0);
  for (size_t s = 0; s < n; ++s) {
    if (state[s] != 0) continue;
    std::vector<RobotId> walk;
    auto v = static_cast<RobotId>(s);
    while (v >= 0 && state[static_cast<size_t>(v)] == 0) {
      state[static_cast<size_t>(v)] = 1;
      walk.push_back(v);
      const Permit& p = out.permits[static_cast<size_t>(v)];
      v = p.held ? p.waits_for : -1;
    }
    if (v >= 0 && state[static_cast<size_t>(v)] == 1) {
      std::vector<RobotId> cycle(std::find(walk.begin(), walk.end(), v), walk.end());
      std::sort(cycle.begin(), cycle.end());
      out.cycles.push_back(std::move(cycle));
    }
    for (const RobotId r : walk) state[static_cast<size_t>(r)] = 2;
  }
  return out;
}

}  // namespace fleet_core
