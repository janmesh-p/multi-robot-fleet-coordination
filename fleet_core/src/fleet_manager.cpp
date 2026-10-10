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

}  // namespace

FleetManager::FleetManager(const Config& config, WorldModel world)
    : FleetManager(config, std::move(world), makeAllocator(config.allocation, config.features),
                   makePlanner(config.planning)) {}

FleetManager::FleetManager(const Config& config, WorldModel world,
                           std::unique_ptr<TaskAllocator> allocator,
                           std::unique_ptr<PathPlanner> planner)
    : config_(config),
      world_(std::move(world)),
      distances_(world_.map(), config.features.distance_oracle),
      stations_(world_.map(), distances_, initialCells(world_), config.features.station_queues),
      allocator_(std::move(allocator)),
      planner_(std::move(planner)),
      traffic_(config.features),
      faults_(config.faults),
      plans_(static_cast<size_t>(world_.fleetSize())) {
  if (!allocator_ || !planner_) {
    throw std::invalid_argument("FleetManager: an allocator and a planner are required");
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
  for (const CellObservation& observation : input.observations) {
    world_.applyObservation(observation);
  }

  const FaultActions faults = faults_.update(input.now, world_, tasks_);
  output.stats.robots_declared_dead = static_cast<int32_t>(faults.declared_dead.size());
  output.stats.tasks_requeued = static_cast<int32_t>(faults.requeued.size());

  for (const Task& task : input.new_orders) tasks_.add(task);
  confirmProgress(input.now);
  allocate(input.now, output.stats);
  plan(input.now, computeGoals(), output.stats);
  output.commands = commands(input.now);
  return output;
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

void FleetManager::allocate(Timestep now, TickStats& stats) {
  if (tasks_.pendingCount() == 0) return;
  AllocationContext context{world_, tasks_, distances_, {}, tasks_.pending()};
  for (const RobotBelief& b : world_.robots()) {
    if (b.status != RobotStatus::kDead && !tasks_.taskOf(b.id)) context.robots.push_back(b.id);
  }
  if (context.robots.empty()) return;
  const std::vector<Assignment> assignments = allocator_->assign(context);
  for (const Assignment& a : assignments) tasks_.assign(a.task, a.robot, now);
  stats.assignments = static_cast<int32_t>(assignments.size());
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
      goal = stations_.goalFor(b.id, station, b.cell);
    }
  }
  return goals;
}

void FleetManager::plan(Timestep now, const std::vector<Cell>& goals, TickStats& stats) {
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
    int32_t priority = -1;
    if (const auto task_id = tasks_.taskOf(b.id)) priority = now - tasks_.get(*task_id).created_at;
    agents.push_back(PlanAgent{b.id, b.cell, goals[i], priority});
  }

  if (!agents.empty()) {
    const PlanningRequest request{world_,
                                  distances_,
                                  agents,
                                  fixedPaths(agents),
                                  now,
                                  config_.planning.window,
                                  config_.planning.expansion_budget,
                                  config_.planning.ecbs_suboptimality};
    const auto started = std::chrono::steady_clock::now();
    PlanResult result = planner_->plan(request);
    const auto finished = std::chrono::steady_clock::now();
    stats.planner_called = true;
    stats.planning_ms = std::chrono::duration<double, std::milli>(finished - started).count();
    stats.expansions = result.expansions;
    stats.robots_planned = static_cast<int32_t>(agents.size());

    if (result.success) {
      if (result.paths.size() != agents.size()) {
        throw std::logic_error("FleetManager: planner returned " +
                               std::to_string(result.paths.size()) + " paths for " +
                               std::to_string(agents.size()) + " agents");
      }
      for (size_t k = 0; k < agents.size(); ++k) {
        Path& path = result.paths[k];
        if (path.robot != agents[k].robot || path.cells.empty() ||
            path.cells.front() != agents[k].start) {
          throw std::logic_error("FleetManager: planner path " + std::to_string(k) +
                                 " does not start at robot " + std::to_string(agents[k].robot));
        }
        commit(agents[k].robot, agents[k].goal, std::move(path), true, committed);
      }
    } else {
      // Hold in place and try again next tick.
      stats.planner_failed = true;
      for (const PlanAgent& a : agents) {
        commit(a.robot, a.goal, Path{a.robot, now, {a.start}}, false, committed);
      }
    }
  }
  traffic_.commit(committed);
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
    const RobotPlan& p = plans_[static_cast<size_t>(b.id)];
    if (!replanned && !p.path.cells.empty()) fixed.push_back(p.path);
  }
  return fixed;
}

std::vector<RobotCommand> FleetManager::commands(Timestep now) const {
  const std::vector<int32_t> permits = traffic_.permits(world_);
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
    command.permitted_until = std::max(permits[i], 0);
    if (const auto task_id = tasks_.taskOf(b.id)) command.task = tasks_.get(*task_id).order;
    out.push_back(std::move(command));
  }
  return out;
}

}  // namespace fleet_core
