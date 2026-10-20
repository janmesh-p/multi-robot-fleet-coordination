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
    case MotionOutcome::kYield:
      return "yield";
    case MotionOutcome::kNoPermit:
      return "no_permit";
    case MotionOutcome::kNoCommand:
      return "no_command";
    case MotionOutcome::kSafeStop:
      return "safe_stop";
  }
  return "unknown";
}

bool isSafetyStop(MotionOutcome outcome) {
  return outcome == MotionOutcome::kBlockedVertex || outcome == MotionOutcome::kBlockedSwap ||
         outcome == MotionOutcome::kBlockedOccupied;
}

GridExecutor::GridExecutor(GridMap map, const std::vector<Cell>& spawn, Timestep service_steps,
                           double delay_probability, uint64_t delay_seed, ExecutorFaults faults)
    : map_(std::move(map)),
      pending_block_(static_cast<size_t>(map_.cellCount()), 0),
      service_steps_(service_steps),
      delay_probability_(delay_probability),
      delay_rng_(delay_seed),
      faults_(faults),
      link_rng_(faults.link_seed),
      silent_until_(spawn.size(), 0) {
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

// One draw per robot, used or not, so the loss stream never shifts. No draws without loss.
std::vector<uint8_t> GridExecutor::drawLosses(Timestep now) {
  std::vector<uint8_t> lost(robots_.size(), 0);
  for (size_t i = 0; i < robots_.size(); ++i) {
    const bool dropped = faults_.packet_loss > 0.0 && link_rng_.bernoulli(faults_.packet_loss);
    lost[i] = dropped || now < silent_until_[i] ? 1 : 0;
  }
  return lost;
}

std::vector<RobotReport> GridExecutor::collectReports(Timestep now) {
  const std::vector<uint8_t> lost = drawLosses(now);
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
    report.status = r.safe_stop ? RobotStatus::kSafeStop : r.status;
    if (r.task) report.task = r.task->id;
    report.carrying = r.carrying;
    report.last_completed = r.last_completed;
    report.plan_id = r.plan_id;
    report.path_index = r.path_index;
    if (lost[i] != 0) {
      ++reports_lost_;
      continue;
    }
    reports.push_back(report);
  }
  return reports;
}

void GridExecutor::dispatch(Timestep now, const std::vector<RobotCommand>& commands) {
  const std::vector<uint8_t> lost = drawLosses(now);
  for (const RobotCommand& command : commands) {
    if (command.robot < 0 || static_cast<size_t>(command.robot) >= robots_.size()) {
      throw std::out_of_range("GridExecutor: command for unknown robot " +
                              std::to_string(command.robot));
    }
    SimRobot& r = robots_[static_cast<size_t>(command.robot)];
    if (!r.alive) continue;
    if (lost[static_cast<size_t>(command.robot)] != 0) {
      ++commands_lost_;
      continue;
    }
    if (command.plan_id < r.plan_id) continue;  // stale command
    r.last_command = now;                       // its permit is good for this step only
    r.safe_stop = false;
    if (command.plan_id > r.plan_id) {
      r.plan_id = command.plan_id;
      const auto on_path = std::find(command.path.begin(), command.path.end(), r.cell);
      if (on_path == command.path.end()) {
        // The plan does not pass through where the robot is: hold until a usable plan arrives.
        r.path = {r.cell};
        r.path_index = 0;
        r.permitted_until = 0;
        r.follow = -1;
        continue;
      }
      r.path = command.path;
      r.path_index = static_cast<int32_t>(on_path - command.path.begin());
    }
    r.permitted_until = command.permitted_until;
    r.follow = command.follow;
    // The command names the order the robot holds. Another or none: the coordinator gave this
    // one's order away while it was out of touch, so it drops it. An order it already delivered
    // is never taken back: the command was sent before its delivery was heard.
    const auto want = command.task ? std::optional<TaskId>(command.task->id) : std::nullopt;
    const auto have = r.task ? std::optional<TaskId>(r.task->id) : std::nullopt;
    if (want == have) continue;
    if (have) {
      r.task.reset();
      r.carrying = false;
      r.service_left = 0;
      r.status = RobotStatus::kIdle;
    }
    if (want && want != r.last_completed) {
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

  // Rescue and lost link (Stage 6).
  for (size_t i = 0; i < n; ++i) {
    SimRobot& r = robots_[i];
    if (!r.alive) {
      if (!r.removed && r.died_at >= 0 && now - r.died_at >= faults_.rescue_after) {
        r.removed = true;
        last_.removed.push_back(static_cast<RobotId>(i));
      }
      continue;
    }
    if (now - r.last_command > faults_.safe_stop_after) r.safe_stop = true;
  }

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
    if (r.safe_stop) {  // holds, and station service waits too
      m.outcome = MotionOutcome::kSafeStop;
      continue;
    }
    if (r.status == RobotStatus::kPicking || r.status == RobotStatus::kDropping) {
      servicing[i] = 1;
      m.outcome = MotionOutcome::kService;
      continue;
    }
    const auto last = static_cast<int32_t>(r.path.size()) - 1;
    if (r.last_command != now && r.path_index < last) {  // no command this step, so no permit
      m.intended = r.path[static_cast<size_t>(r.path_index + 1)];
      m.outcome = MotionOutcome::kNoCommand;
      continue;
    }
    const int32_t limit = std::min(r.permitted_until, last);
    if (r.path_index >= limit) {
      if (r.path_index < last) {  // the path goes on, the permit does not
        m.intended = r.path[static_cast<size_t>(r.path_index + 1)];
        m.outcome = MotionOutcome::kNoPermit;
      } else {
        m.outcome = r.task ? MotionOutcome::kHold : MotionOutcome::kIdle;
      }
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

  // 2. Robot safety. Dead robots still occupy their cells until they are taken away.
  std::vector<int32_t> occupant(static_cast<size_t>(map_.cellCount()), -1);
  for (size_t i = 0; i < n; ++i) {
    if (robots_[i].removed) continue;
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
  // it, so repeat until nothing changes (at most n rounds). A robot told to follow that occupant
  // yields instead: the coordinator allowed the move only if the occupant left.
  for (bool changed = true; changed;) {
    changed = false;
    for (size_t i = 0; i < n; ++i) {
      if (mover[i] == 0) continue;
      const int32_t j = occupant[static_cast<size_t>(map_.index(target[i]))];
      if (j >= 0 && static_cast<size_t>(j) != i && mover[static_cast<size_t>(j)] == 0) {
        stop(i, robots_[i].follow == j ? MotionOutcome::kYield : MotionOutcome::kBlockedOccupied);
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

  // 4. Closures whose cells have emptied.
  for (auto it = pending_.begin(); it != pending_.end();) {
    if (occupied(*it)) {
      ++it;
      continue;
    }
    for (const int32_t c : *it) {
      map_.block(map_.cellAt(c));
      pending_block_[static_cast<size_t>(c)] = 0;
    }
    it = pending_.erase(it);
  }

  // 5. Stations. Robots that were already servicing count down; robots whose path ends on their
  //    station start service.
  for (size_t i = 0; i < n; ++i) {
    SimRobot& r = robots_[i];
    if (!r.alive || r.safe_stop) continue;
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

bool GridExecutor::occupied(const std::vector<int32_t>& cells) const {
  return std::any_of(robots_.begin(), robots_.end(), [&](const SimRobot& r) {
    return !r.removed && std::find(cells.begin(), cells.end(), map_.index(r.cell)) != cells.end();
  });
}

void GridExecutor::block(const std::vector<Cell>& cells) {
  std::vector<int32_t> closure;
  for (const Cell& cell : cells) {
    if (!map_.inBounds(cell)) throw std::out_of_range("GridExecutor: closure outside the map");
    closure.push_back(map_.index(cell));
  }
  if (!occupied(closure)) {
    for (const Cell& cell : cells) map_.block(cell);
    return;
  }
  for (const int32_t c : closure) pending_block_[static_cast<size_t>(c)] = 1;
  pending_.push_back(std::move(closure));
}

void GridExecutor::unblock(const std::vector<Cell>& cells) {
  for (const Cell& cell : cells) {
    map_.unblock(cell);  // throws std::out_of_range outside the map
    const int32_t c = map_.index(cell);
    pending_block_[static_cast<size_t>(c)] = 0;
    for (std::vector<int32_t>& closure : pending_) {
      closure.erase(std::remove(closure.begin(), closure.end(), c), closure.end());
    }
  }
  pending_.erase(
      std::remove_if(pending_.begin(), pending_.end(),
                     [](const std::vector<int32_t>& closure) { return closure.empty(); }),
      pending_.end());
}

SimRobot& GridExecutor::robotAt(RobotId id) {
  if (id < 0 || static_cast<size_t>(id) >= robots_.size()) {
    throw std::out_of_range("GridExecutor: unknown robot " + std::to_string(id));
  }
  return robots_[static_cast<size_t>(id)];
}

void GridExecutor::kill(RobotId id, Timestep now) {
  SimRobot& r = robotAt(id);
  if (!r.alive) return;
  r.alive = false;
  r.died_at = now;
  r.status = RobotStatus::kDead;
  r.task.reset();
  r.carrying = false;
  r.service_left = 0;
  r.safe_stop = false;
}

void GridExecutor::outage(RobotId id, Timestep now, Timestep duration) {
  robotAt(id);
  Timestep& until = silent_until_[static_cast<size_t>(id)];
  until = std::max(until, now + duration);
}

bool GridExecutor::closing(const Cell& c) const {
  return map_.isBlocked(c) || pending_block_[static_cast<size_t>(map_.index(c))] != 0;
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
    s.on_floor.push_back(!r.removed);
    s.status.push_back(r.status);
    s.holding.push_back(r.task ? std::optional<TaskId>(r.task->id) : std::nullopt);
  }
  return s;
}

}  // namespace fleet_core
