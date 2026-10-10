#include "fleet_core/execution/grid_executor.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <utility>

namespace fleet_core {

std::string_view toString(MotionOutcome outcome) {
  switch (outcome) {
    case MotionOutcome::kIdle:
      return "idle";
    case MotionOutcome::kMoved:
      return "moved";
    case MotionOutcome::kPlannedWait:
      return "wait";
    case MotionOutcome::kHold:
      return "hold";
    case MotionOutcome::kService:
      return "service";
    case MotionOutcome::kDelayed:
      return "delayed";
    case MotionOutcome::kBlockedVertex:
      return "blocked_vertex";
    case MotionOutcome::kBlockedSwap:
      return "blocked_swap";
    case MotionOutcome::kBlockedOccupied:
      return "blocked_occupied";
    case MotionOutcome::kBlockedCell:
      return "blocked_cell";
    case MotionOutcome::kInvalidStep:
      return "invalid_step";
    case MotionOutcome::kDead:
      return "dead";
  }
  return "unknown";
}

bool isSafetyStop(MotionOutcome outcome) {
  return outcome == MotionOutcome::kBlockedVertex || outcome == MotionOutcome::kBlockedSwap ||
         outcome == MotionOutcome::kBlockedOccupied;
}

GridExecutor::GridExecutor(GridMap map, const std::vector<Cell>& spawn, Timestep service_steps,
                           double delay_probability, uint64_t delay_seed)
    : map_(std::move(map)),
      service_steps_(service_steps),
      delay_probability_(delay_probability),
      delay_rng_(delay_seed) {
  std::vector<uint8_t> used(static_cast<size_t>(map_.cellCount()), 0);
  for (const Cell& cell : spawn) {
    if (!map_.isFree(cell)) throw std::invalid_argument("GridExecutor: spawn cell is not free");
    uint8_t& u = used[static_cast<size_t>(map_.index(cell))];
    if (u != 0) throw std::invalid_argument("GridExecutor: two robots spawn on one cell");
    u = 1;
    SimRobot robot;
    robot.cell = cell;
    robot.path = {cell};
    robots_.push_back(robot);
  }
}

std::vector<RobotReport> GridExecutor::collectReports(Timestep now) {
  std::vector<RobotReport> reports;
  reports.reserve(robots_.size());
  for (size_t i = 0; i < robots_.size(); ++i) {
    SimRobot& r = robots_[i];
    if (!r.alive) continue;
    RobotReport report;
    report.robot = static_cast<RobotId>(i);
    report.seq = ++r.seq;
    report.stamp = now;
    report.cell = r.cell;
    report.status = r.status;
    if (r.task) report.task = r.task->id;
    report.last_completed = r.last_completed;
    report.plan_id = r.plan_id;
    report.path_index = r.path_index;
    reports.push_back(report);
  }
  return reports;
}

void GridExecutor::dispatch(Timestep /*now*/, const std::vector<RobotCommand>& commands) {
  for (const RobotCommand& command : commands) {
    if (command.robot < 0 || static_cast<size_t>(command.robot) >= robots_.size()) {
      throw std::out_of_range("GridExecutor: command for unknown robot " +
                              std::to_string(command.robot));
    }
    SimRobot& r = robots_[static_cast<size_t>(command.robot)];
    if (!r.alive || command.plan_id < r.plan_id) continue;  // stale command
    if (command.plan_id > r.plan_id) {
      r.plan_id = command.plan_id;
      const auto on_path = std::find(command.path.begin(), command.path.end(), r.cell);
      if (on_path == command.path.end()) {
        // The plan does not pass through where the robot is: hold until a usable plan arrives.
        r.path = {r.cell};
        r.path_index = 0;
        r.permitted_until = 0;
        continue;
      }
      r.path = command.path;
      r.path_index = static_cast<int32_t>(on_path - command.path.begin());
    }
    r.permitted_until = command.permitted_until;
    // Adopt a task only when free. Reassignment while busy is Stage 6 (false deaths).
    if (command.task && !r.task) {
      r.task = command.task;
      r.carrying = false;
      if (r.status == RobotStatus::kIdle) r.status = RobotStatus::kToPickup;
    }
  }
}

void GridExecutor::step(Timestep now) {
  const size_t n = robots_.size();
  last_ = StepTruth{};
  last_.t = now;
  last_.motion.resize(n);

  // One delay draw per robot per step, used or not, so the delay stream never shifts.
  std::vector<uint8_t> delayed(n, 0);
  for (size_t i = 0; i < n; ++i) delayed[i] = delay_rng_.bernoulli(delay_probability_) ? 1 : 0;

  std::vector<uint8_t> mover(n, 0);
  std::vector<uint8_t> servicing(n, 0);
  std::vector<Cell> target(n);

  // 1. Each robot's intent for this step.
  for (size_t i = 0; i < n; ++i) {
    SimRobot& r = robots_[i];
    MotionRecord& m = last_.motion[i];
    m.robot = static_cast<RobotId>(i);
    m.from = m.to = m.intended = r.cell;
    if (!r.alive) {
      m.outcome = MotionOutcome::kDead;
      continue;
    }
    if (r.status == RobotStatus::kPicking || r.status == RobotStatus::kDropping) {
      servicing[i] = 1;
      m.outcome = MotionOutcome::kService;
      continue;
    }
    const int32_t limit = std::min(r.permitted_until, static_cast<int32_t>(r.path.size()) - 1);
    if (r.path_index >= limit) {
      m.outcome = r.task ? MotionOutcome::kHold : MotionOutcome::kIdle;
      continue;
    }
    const Cell next = r.path[static_cast<size_t>(r.path_index + 1)];
    m.intended = next;
    if (delayed[i] != 0) {
      m.outcome = MotionOutcome::kDelayed;
      continue;
    }
    if (next == r.cell) {
      m.outcome = MotionOutcome::kPlannedWait;
      ++r.path_index;
      continue;
    }
    if (std::abs(next.row - r.cell.row) + std::abs(next.col - r.cell.col) != 1 ||
        !map_.isStaticFree(next)) {
      m.outcome = MotionOutcome::kInvalidStep;
      continue;
    }
    if (map_.isBlocked(next)) {
      m.outcome = MotionOutcome::kBlockedCell;
      continue;
    }
    mover[i] = 1;
    target[i] = next;
    m.outcome = MotionOutcome::kMoved;  // until safety says otherwise
  }

  // 2. Robot safety. Dead robots still occupy their cells.
  std::vector<int32_t> occupant(static_cast<size_t>(map_.cellCount()), -1);
  for (size_t i = 0; i < n; ++i) {
    occupant[static_cast<size_t>(map_.index(robots_[i].cell))] = static_cast<int32_t>(i);
  }
  const auto stop = [&](size_t i, MotionOutcome why) {
    mover[i] = 0;
    last_.motion[i].outcome = why;
  };
  // Two robots want the same cell: the lower id goes.
  std::vector<int32_t> claimant(static_cast<size_t>(map_.cellCount()), -1);
  for (size_t i = 0; i < n; ++i) {
    if (mover[i] == 0) continue;
    int32_t& claim = claimant[static_cast<size_t>(map_.index(target[i]))];
    if (claim == -1) {
      claim = static_cast<int32_t>(i);
    } else {
      stop(i, MotionOutcome::kBlockedVertex);
    }
  }
  // Head-on: both stop.
  for (size_t i = 0; i < n; ++i) {
    if (mover[i] == 0) continue;
    const int32_t j = occupant[static_cast<size_t>(map_.index(target[i]))];
    if (j < 0 || static_cast<size_t>(j) == i) continue;
    const auto ju = static_cast<size_t>(j);
    if (mover[ju] != 0 && target[ju] == robots_[i].cell) {
      stop(i, MotionOutcome::kBlockedSwap);
      stop(ju, MotionOutcome::kBlockedSwap);
    }
  }
  // A robot cannot enter a cell whose occupant stays. Every new stop can block the robot behind
  // it, so repeat until nothing changes (at most n rounds).
  for (bool changed = true; changed;) {
    changed = false;
    for (size_t i = 0; i < n; ++i) {
      if (mover[i] == 0) continue;
      const int32_t j = occupant[static_cast<size_t>(map_.index(target[i]))];
      if (j >= 0 && static_cast<size_t>(j) != i && mover[static_cast<size_t>(j)] == 0) {
        stop(i, MotionOutcome::kBlockedOccupied);
        changed = true;
      }
    }
  }

  // 3. Move.
  for (size_t i = 0; i < n; ++i) {
    if (mover[i] == 0) continue;
    robots_[i].cell = target[i];
    ++robots_[i].path_index;
    last_.motion[i].to = target[i];
  }

  // 4. Stations. Robots that were already servicing count down; robots whose path ends on their
  //    station start service.
  for (size_t i = 0; i < n; ++i) {
    SimRobot& r = robots_[i];
    if (!r.alive) continue;
    if (servicing[i] != 0) {
      if (--r.service_left <= 0) finishService(static_cast<RobotId>(i));
      continue;
    }
    if (!r.task) continue;
    const bool path_done = r.path_index == static_cast<int32_t>(r.path.size()) - 1;
    const Cell station = r.carrying ? r.task->dropoff : r.task->pickup;
    const bool heading = r.status == RobotStatus::kToPickup || r.status == RobotStatus::kToDropoff;
    if (path_done && heading && r.cell == station) {
      r.status = r.carrying ? RobotStatus::kDropping : RobotStatus::kPicking;
      r.service_left = service_steps_;
      if (r.service_left <= 0) finishService(static_cast<RobotId>(i));
    }
  }
}

void GridExecutor::finishService(RobotId id) {
  SimRobot& r = robots_[static_cast<size_t>(id)];
  r.service_left = 0;
  if (r.status == RobotStatus::kPicking) {
    r.carrying = true;
    r.status = RobotStatus::kToDropoff;
    last_.pickups.push_back(Assignment{id, r.task->id});
    return;
  }
  last_.deliveries.push_back(Assignment{id, r.task->id});
  r.last_completed = r.task->id;
  r.task.reset();
  r.carrying = false;
  r.status = RobotStatus::kIdle;
}

TruthSnapshot GridExecutor::snapshot() const {
  TruthSnapshot s;
  for (const SimRobot& r : robots_) {
    s.cells.push_back(r.cell);
    s.alive.push_back(r.alive);
    s.status.push_back(r.status);
    s.holding.push_back(r.task ? std::optional<TaskId>(r.task->id) : std::nullopt);
  }
  return s;
}

}  // namespace fleet_core
