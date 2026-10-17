#include "fleet_core/fleet_manager.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>

namespace fleet_core {
namespace {

// Robots start at home, so their start cells are never offered as station waiting cells.
std::vector<Cell> initialCells(const WorldModel& world) {
  std::vector<Cell> cells;
  for (const RobotBelief& b : world.robots()) cells.push_back(b.cell);
  return cells;
}

// Dwelling at a station: the robot ignores motion until its service ends.
bool servicing(RobotStatus status) {
  return status == RobotStatus::kPicking || status == RobotStatus::kDropping;
}

// The traffic rules a config turns on. Only coordinated planners make plans the rules can keep,
// so the uncoordinated baseline stays as it was. The dependency graph needs plans made together,
// which only rolling replanning makes.
TrafficRules trafficRules(const Config& config) {
  const bool coordinated = isCoordinated(config.planning.method);
  const FeatureToggles& f = config.features;
  return TrafficRules{coordinated && f.rolling_replanning && f.dependency_graph,
                      coordinated && f.intersection_reservations};
}

}  // namespace

std::optional<std::string> planningRequirementGap(const Config& config) {
  const PlanningMethod method = config.planning.method;
  if (!isCoordinated(method)) return std::nullopt;
  const std::string name = "planning.method " + std::string(toString(method));
  if (!config.features.station_queues) {
    return name + " needs features.station_queues: robots that share a goal cannot all reach it";
  }
  if (!config.features.parking) {
    return name + " needs features.parking: idle robots must wait off the aisles";
  }
  if (!config.features.rolling_replanning &&
      (method == PlanningMethod::kWindowedCoopAStar || method == PlanningMethod::kPibt)) {
    return name + " needs features.rolling_replanning: it plans only planning.window steps ahead";
  }
  return std::nullopt;
}

FleetManager::FleetManager(const Config& config, WorldModel world)
    : FleetManager(config, std::move(world), makeAllocator(config), makePlanner(config.planning)) {}

FleetManager::FleetManager(const Config& config, WorldModel world,
                           std::unique_ptr<TaskAllocator> allocator,
                           std::unique_ptr<PathPlanner> planner)
    : config_(config),
      world_(std::move(world)),
      distances_(world_.map(), config.features.distance_oracle),
      stations_(world_.map(), distances_, initialCells(world_), config.features.station_queues),
      allocator_(std::move(allocator)),
      planner_(std::move(planner)),
      traffic_(trafficRules(config)),
      faults_(config.faults),
      plans_(static_cast<size_t>(world_.fleetSize())) {
  if (!allocator_ || !planner_) {
    throw std::invalid_argument("FleetManager: an allocator and a planner are required");
  }
  if (const auto gap = planningRequirementGap(config_)) {
    throw std::invalid_argument("FleetManager: " + *gap);
  }
  if (config_.features.parking) parking_.emplace(world_.map(), world_.fleetSize());
}

FleetManager::~FleetManager() = default;

Cell FleetManager::goalOf(RobotId robot) const {
  if (robot < 0 || robot >= world_.fleetSize()) {
    throw std::out_of_range("FleetManager: unknown robot " + std::to_string(robot));
  }
  return plans_[static_cast<size_t>(robot)].goal;
}

TickOutput FleetManager::tick(const TickInput& input) {
  TickOutput output;
  world_.setNow(input.now);
  for (const RobotReport& report : input.reports) world_.applyReport(report);
  bool cells_changed = false;
  for (const CellObservation& observation : input.observations) {
    if (!observation.cleared.empty()) {
      world_.applyObservation(CellObservation{observation.stamp, {}, observation.cleared});
      for (std::vector<Cell>& closure : pending_closures_) {
        closure.erase(std::remove_if(closure.begin(), closure.end(),
                                     [&](const Cell& c) {
                                       return std::find(observation.cleared.begin(),
                                                        observation.cleared.end(),
                                                        c) != observation.cleared.end();
                                     }),
                      closure.end());
      }
      cells_changed = true;
    }
    if (!observation.blocked.empty()) pending_closures_.push_back(observation.blocked);
  }
  // A closure takes effect once no robot stands on any of its cells, as on the floor: closing
  // the cells around a robot would leave it no way out.
  for (auto it = pending_closures_.begin(); it != pending_closures_.end();) {
    if (!it->empty() && onClosedCells(*it)) {
      ++it;
      continue;
    }
    if (!it->empty()) {
      world_.applyObservation(CellObservation{input.now, *it, {}});
      cells_changed = true;
    }
    it = pending_closures_.erase(it);
  }
  if (cells_changed) onCellsChanged(input.now);

  const FaultActions faults = faults_.update(input.now, world_, tasks_);
  output.stats.robots_declared_dead = static_cast<int32_t>(faults.declared_dead.size());
  output.stats.tasks_requeued = static_cast<int32_t>(faults.requeued.size());
  output.requeued = faults.requeued;

  for (const Task& task : input.new_orders) tasks_.add(task);
  confirmProgress(input.now);
  allocate(input.now, output);
  plan(input.now, computeGoals(), output.stats);
  const TrafficDecision traffic = traffic_.decide(world_, priorities(input.now));
  if (config_.features.deadlock_recovery && isCoordinated(config_.planning.method) &&
      !traffic.cycles.empty()) {
    recover(input.now, traffic.cycles, output.stats);
  }
  output.commands = commands(input.now, traffic);
  return output;
}

bool FleetManager::onClosedCells(const std::vector<Cell>& cells) const {
  return std::any_of(world_.robots().begin(), world_.robots().end(), [&](const RobotBelief& b) {
    return std::find(cells.begin(), cells.end(), b.cell) != cells.end();
  });
}

// A closed or reopened cell changes distances and can cut planned paths. The distance table is
// rebuilt around closed cells, then robots replan: the baseline replans each robot whose path now
// crosses a closed cell; coordinated planners replan the whole fleet at once, so the new plans fit
// together.
void FleetManager::onCellsChanged(Timestep now) {
  distances_ = DistanceOracle(world_.map(), config_.features.distance_oracle);
  if (!isCoordinated(config_.planning.method)) {
    for (const RobotBelief& b : world_.robots()) {
      RobotPlan& p = plans_[static_cast<size_t>(b.id)];
      if (!p.valid) continue;
      const auto from = static_cast<size_t>(pathProgress(b, p.plan_id, p.path.cells));
      for (size_t k = from + 1; k < p.path.cells.size(); ++k) {
        if (world_.map().isBlocked(p.path.cells[k])) {
          p.valid = false;
          break;
        }
      }
    }
  } else if (config_.features.rolling_replanning) {
    next_replan_ = now;
  } else {
    for (RobotPlan& p : plans_) p.valid = false;
  }
}

// Reports are state, not events: a delivery stays in last_completed until the next one, so a
// confirmation is never missed, only delayed.
void FleetManager::confirmProgress(Timestep now) {
  for (const RobotBelief& b : world_.robots()) {
    if (b.status == RobotStatus::kDead) continue;
    if (b.last_completed && tasks_.contains(*b.last_completed)) {
      const Task& task = tasks_.get(*b.last_completed);
      if (task.status != TaskStatus::kDone && task.robot == b.id) {
        if (task.status == TaskStatus::kAssigned) {  // pickup report never seen
          stations_.release(b.id, task.order.pickup);
          tasks_.markPickedUp(task.order.id, now);
        }
        stations_.release(b.id, task.order.dropoff);
        tasks_.markDone(task.order.id, now);
      }
    }
    const bool carrying = b.status == RobotStatus::kToDropoff || b.status == RobotStatus::kDropping;
    if (carrying && b.task && tasks_.contains(*b.task)) {
      const Task& task = tasks_.get(*b.task);
      if (task.status == TaskStatus::kAssigned && task.robot == b.id) {
        stations_.release(b.id, task.order.pickup);
        tasks_.markPickedUp(task.order.id, now);
      }
    }
  }
}

void FleetManager::allocate(Timestep now, TickOutput& output) {
  if (tasks_.pendingCount() == 0) return;
  AllocationContext context{world_, tasks_, distances_, {}, tasks_.pending()};
  for (const RobotBelief& b : world_.robots()) {
    if (b.status != RobotStatus::kDead && !tasks_.taskOf(b.id)) context.robots.push_back(b.id);
  }
  if (context.robots.empty()) return;
  const std::vector<Assignment> assignments = allocator_->assign(context);
  if (!assignments.empty()) {
    const PairFeatureTable table(context);  // before this tick's assignments change the counts
    for (const Assignment& a : assignments) {
      output.assignments.push_back(AssignmentRecord{
          now, a.task, a.robot, tasks_.get(a.task).order.pickup, table.pair(a.robot, a.task)});
    }
  }
  for (const Assignment& a : assignments) tasks_.assign(a.task, a.robot, now);
  output.stats.assignments = static_cast<int32_t>(assignments.size());
}

std::vector<Cell> FleetManager::computeGoals() {
  std::vector<Cell> goals(plans_.size());
  for (const RobotBelief& b : world_.robots()) {
    Cell& goal = goals[static_cast<size_t>(b.id)];
    const auto task_id = tasks_.taskOf(b.id);
    if (b.status == RobotStatus::kDead) {
      goal = b.cell;
    } else if (!task_id) {
      goal = parking_ ? parking_->home(b.id) : b.cell;
    } else {
      const Task& task = tasks_.get(*task_id);
      const Cell station =
          task.status == TaskStatus::kPickedUp ? task.order.dropoff : task.order.pickup;
      stations_.request(b.id, station);
      // With every waiting cell taken, the baseline waits where it is. Coordinated planners send
      // the robot home instead: a robot waiting on a station or in an aisle can block the very
      // robots it waits for.
      const bool home = parking_ && isCoordinated(config_.planning.method);
      goal = stations_.goalFor(b.id, station, home ? parking_->home(b.id) : b.cell);
    }
  }
  return goals;
}

void FleetManager::plan(Timestep now, const std::vector<Cell>& goals, TickStats& stats) {
  if (!isCoordinated(config_.planning.method)) {
    planIndependent(now, goals, stats);
  } else if (config_.features.rolling_replanning) {
    planRolling(now, goals, stats);
  } else {
    planToGoals(now, goals, stats);
  }
}

// The uncoordinated baseline: robots plan alone, so each needs a new path only when its goal
// changes. A failed call holds the robots in place.
void FleetManager::planIndependent(Timestep now, const std::vector<Cell>& goals, TickStats& stats) {
  std::vector<CommittedPlan> committed;
  std::vector<PlanAgent> agents;
  for (const RobotBelief& b : world_.robots()) {
    if (b.status == RobotStatus::kDead) continue;
    const auto i = static_cast<size_t>(b.id);
    const RobotPlan& current = plans_[i];
    if (current.valid && current.goal == goals[i]) continue;  // still heading there
    if (b.cell == goals[i]) {
      commit(b.id, goals[i], Path{b.id, now, {b.cell}}, true, committed);  // there: hold
      continue;
    }
    agents.push_back(agent(b, goals[i], now));
  }

  if (!agents.empty()) {
    PlanResult result = runPlanner(now, agents, fixedPaths(agents), config_.planning.window, stats);
    if (result.success) {
      for (size_t k = 0; k < agents.size(); ++k) {
        commit(agents[k].robot, agents[k].goal, std::move(result.paths[k]), true, committed);
      }
    } else {
      // Hold in place and try again next tick.
      for (const PlanAgent& a : agents) {
        commit(a.robot, a.goal, Path{a.robot, now, {a.start}}, false, committed);
      }
    }
  }
  traffic_.commit(committed);
}

// Rolling horizon (Li et al. 2021): every replan_period steps, every robot not dwelling at a
// station gets a new plan, conflict-free for the next `window` steps. Robots at a station hold
// it until their service ends. Between replannings the robots follow their plans, which cover
// at least replan_period steps because replan_period <= window.
void FleetManager::planRolling(Timestep now, const std::vector<Cell>& goals, TickStats& stats) {
  if (now < next_replan_) return;
  std::vector<PlanAgent> agents;
  std::vector<Path> fixed;
  for (const RobotBelief& b : world_.robots()) {
    if (b.status == RobotStatus::kDead) continue;
    if (servicing(b.status)) {
      fixed.push_back(Path{b.id, now, {b.cell}});
      continue;
    }
    agents.push_back(agent(b, goals[static_cast<size_t>(b.id)], now));
  }
  if (agents.empty()) {
    next_replan_ = now + config_.planning.replan_period;
    return;
  }
  PlanResult result = runPlanner(now, agents, std::move(fixed), config_.planning.window, stats);
  if (!result.success) {
    next_replan_ = now + 1;
    return;
  }
  std::vector<CommittedPlan> committed;
  for (size_t k = 0; k < agents.size(); ++k) {
    commit(agents[k].robot, agents[k].goal, std::move(result.paths[k]), true, committed);
  }
  traffic_.commit(committed);
  next_replan_ = now + config_.planning.replan_period;
}

// Without rolling replanning: a robot whose goal changed gets a path all the way there, around
// the committed plans of the others, which then hold their goals. Robots dwelling at a station
// are left alone.
void FleetManager::planToGoals(Timestep now, const std::vector<Cell>& goals, TickStats& stats) {
  std::vector<PlanAgent> agents;
  for (const RobotBelief& b : world_.robots()) {
    if (b.status == RobotStatus::kDead || servicing(b.status)) continue;
    const auto i = static_cast<size_t>(b.id);
    const RobotPlan& current = plans_[i];
    if (current.valid && current.goal == goals[i]) continue;
    agents.push_back(agent(b, goals[i], now));
  }
  if (agents.empty()) return;
  PlanResult result = runPlanner(now, agents, fixedPaths(agents), 0, stats);
  if (!result.success) {
    for (const PlanAgent& a : agents) plans_[static_cast<size_t>(a.robot)].valid = false;
    return;
  }
  std::vector<CommittedPlan> committed;
  for (size_t k = 0; k < agents.size(); ++k) {
    commit(agents[k].robot, agents[k].goal, std::move(result.paths[k]), true, committed);
  }
  traffic_.commit(committed);
}

PlanResult FleetManager::runPlanner(Timestep now, const std::vector<PlanAgent>& agents,
                                    std::vector<Path> fixed, int32_t window, TickStats& stats) {
  const PlanningRequest request{world_,
                                distances_,
                                agents,
                                std::move(fixed),
                                now,
                                window,
                                config_.planning.expansion_budget,
                                config_.planning.ecbs_suboptimality};
  const auto started = std::chrono::steady_clock::now();
  PlanResult result = planner_->plan(request);
  const auto finished = std::chrono::steady_clock::now();
  stats.planner_called = true;
  stats.planner_failed = !result.success;
  stats.planning_ms = std::chrono::duration<double, std::milli>(finished - started).count();
  stats.expansions = result.expansions;
  stats.robots_planned = static_cast<int32_t>(agents.size());
  if (!result.success) return result;

  if (result.paths.size() != agents.size()) {
    throw std::logic_error("FleetManager: planner returned " + std::to_string(result.paths.size()) +
                           " paths for " + std::to_string(agents.size()) + " agents");
  }
  for (size_t k = 0; k < agents.size(); ++k) {
    const Path& path = result.paths[k];
    if (path.robot != agents[k].robot || path.cells.empty() ||
        path.cells.front() != agents[k].start) {
      throw std::logic_error("FleetManager: planner path " + std::to_string(k) +
                             " does not start at robot " + std::to_string(agents[k].robot));
    }
  }
  return result;
}

PlanAgent FleetManager::agent(const RobotBelief& robot, const Cell& goal, Timestep now) const {
  int32_t priority = -1;
  if (const auto task_id = tasks_.taskOf(robot.id)) {
    priority = now - tasks_.get(*task_id).created_at;
  }
  return PlanAgent{robot.id, robot.cell, goal, priority};
}

void FleetManager::commit(RobotId robot, const Cell& goal, Path path, bool valid,
                          std::vector<CommittedPlan>& committed) {
  RobotPlan& p = plans_[static_cast<size_t>(robot)];
  p.valid = valid;
  p.goal = goal;
  ++p.plan_id;
  p.path = std::move(path);
  committed.push_back(CommittedPlan{robot, p.plan_id, p.path});
}

std::vector<Path> FleetManager::fixedPaths(const std::vector<PlanAgent>& agents) const {
  std::vector<Path> fixed;
  for (const RobotBelief& b : world_.robots()) {
    if (b.status == RobotStatus::kDead) continue;
    const bool replanned = std::any_of(agents.begin(), agents.end(),
                                       [&](const PlanAgent& a) { return a.robot == b.id; });
    if (replanned) continue;
    const RobotPlan& p = plans_[static_cast<size_t>(b.id)];
    if (p.path.cells.empty()) {
      fixed.push_back(Path{b.id, world_.now(), {b.cell}});
      continue;
    }
    // The rest of its plan from where it really is, starting now: a late robot keeps its place in
    // line instead of being assumed further along. On schedule, this is the plan itself. A plan
    // committed in this tick (a robot told to hold) starts where the robot is.
    const auto from = static_cast<std::ptrdiff_t>(pathProgress(b, p.plan_id, p.path.cells));
    fixed.push_back(Path{b.id, world_.now(),
                         std::vector<Cell>(p.path.cells.begin() + from, p.path.cells.end())});
  }
  return fixed;
}

std::vector<int32_t> FleetManager::priorities(Timestep now) const {
  std::vector<int32_t> out(plans_.size(), -1);
  for (const RobotBelief& b : world_.robots()) {
    out[static_cast<size_t>(b.id)] = agent(b, b.cell, now).priority;
  }
  return out;
}

// Held robots that wait for each other in a cycle cannot move until a plan changes. Rolling
// replanning brings its next replanning forward to the next tick. Replanning to goals replans the
// whole fleet together at the next tick: the robots on the cycle alone may find no way out around
// the others' plans, and then the same failing call would repeat forever.
void FleetManager::recover(Timestep now, const std::vector<std::vector<RobotId>>& cycles,
                           TickStats& stats) {
  if (config_.features.rolling_replanning) {
    next_replan_ = std::min(next_replan_, now + 1);
  } else {
    for (RobotPlan& p : plans_) p.valid = false;
  }
  stats.deadlocks_resolved += static_cast<int32_t>(cycles.size());
}

std::vector<RobotCommand> FleetManager::commands(Timestep now,
                                                 const TrafficDecision& traffic) const {
  std::vector<RobotCommand> out;
  for (const RobotBelief& b : world_.robots()) {
    if (b.status == RobotStatus::kDead) continue;
    const auto i = static_cast<size_t>(b.id);
    const RobotPlan& p = plans_[i];
    RobotCommand command;
    command.robot = b.id;
    command.plan_id = p.plan_id;
    command.issued_at = now;
    command.path = p.path.cells;
    command.permitted_until = std::max(traffic.permits[i].until, 0);
    command.follow = traffic.permits[i].follow;
    if (const auto task_id = tasks_.taskOf(b.id)) command.task = tasks_.get(*task_id).order;
    out.push_back(std::move(command));
  }
  return out;
}

}  // namespace fleet_core
