#pragma once

#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "fleet_core/common/config.hpp"
#include "fleet_core/common/types.hpp"
#include "fleet_core/execution/grid_executor.hpp"

namespace fleet_core {

// One run's results. Steady-state numbers count only steps at or after the warmup.
struct RunSummary {
  std::string run_name{};
  std::string allocator{};
  std::string planner{};
  int32_t fleet_size{0};
  uint64_t seed{0};
  Timestep horizon{0};
  Timestep warmup{0};
  Timestep steps_run{0};  // less than horizon when a violation aborted the run

  int64_t orders_created{0};    // whole run
  int64_t orders_delivered{0};  // whole run

  double orders_per_hour{0.0};        // steady state
  double mean_completion_steps{0.0};  // created after warmup and delivered before the end
  double p95_completion_steps{0.0};
  int64_t completion_samples{0};
  double utilization{0.0};      // share of robot-steps spent holding a task
  int64_t conflicts{0};         // robot-steps stopped by robot safety
  int64_t congestion_steps{0};  // robot-steps stopped by safety or waiting in a plan

  int64_t planning_calls{0};    // whole run: planning latency is a compute cost, not a
  double planning_p50_ms{0.0};  // steady-state number, so warmup ticks count too
  double planning_p95_ms{0.0};
  int64_t expansions{0};        // whole run
  int64_t planner_failures{0};  // whole run: calls that found no plan; not in summary.csv

  // Stage 5, steady state. Not in summary.csv, whose columns the baseline reference fixes.
  int64_t delayed_steps{0};          // robot-steps lost to execution noise
  int64_t yields{0};                 // robot-steps waiting behind a robot it was told to follow
  int64_t traffic_holds{0};          // robot-steps held by the traffic rules, path going on
  int64_t deadlocks_resolved{0};     // whole run: wait-for cycles the coordinator replanned away
  double max_completion_steps{0.0};  // slowest of the orders in mean_completion_steps
  Timestep oldest_open_steps{0};     // age at the end of the oldest order not delivered; 0: none

  // Stage 6, whole run. Not in summary.csv either.
  int64_t reports_lost{0};          // reports from live robots the link dropped
  int64_t commands_lost{0};         // commands to live robots the link dropped
  int32_t robots_died{0};           // robots a robot_death event killed
  int32_t declared_dead{0};         // heartbeat leases that ran out
  int32_t false_deaths{0};          // declared dead while still alive
  int32_t reconciled{0};            // robots declared dead that reported again
  int32_t rescued{0};               // dead robots taken off the floor
  int64_t requeued{0};              // orders taken back from robots
  int64_t duplicate_deliveries{0};  // deliveries of an order already delivered
  int64_t no_command_steps{0};      // robot-steps waiting because no command arrived
  int64_t safe_stop_steps{0};       // robot-steps in safe stop

  // Stage 7, steady state: where the robot time goes. Every robot-step after the warmup falls in
  // exactly one of these, so they add up to fleet_size x (steps_run - warmup). Not in summary.csv.
  int64_t time_drive_to_pickup{0};  // moved, holding an order it has not picked up yet
  int64_t time_drive_loaded{0};     // moved, carrying an order
  int64_t time_drive_free{0};       // moved without an order, for example home
  int64_t time_service{0};          // picking or dropping at a station
  int64_t time_traffic{0};          // waiting for other robots: a wait in its plan, a hold by the
                                    // traffic rules, or a yield
  int64_t time_station_wait{0};     // holding an order with no step left: waiting for its
                                    // station turn, or for a new plan
  int64_t time_idle{0};             // no order and nothing to do
  int64_t time_safety_stop{0};      // stopped by robot safety
  int64_t time_lost{0};             // late (execution noise), no command, safe stop, dead, or a
                                    // blocked or invalid step
  int32_t stations{0};              // pickup and dropoff cells: time_service over stations x
                                    // measured steps is how busy the stations were
  // The orders in mean_completion_steps, split at their first assignment and first pickup. The
  // three means add up to mean_completion_steps.
  double mean_wait_for_robot_steps{0.0};  // created to assigned
  double mean_to_pickup_steps{0.0};       // assigned to picked up, station waits and service in
  double mean_to_dropoff_steps{0.0};      // picked up to delivered, station waits and service in

  int64_t violations{0};
  bool aborted{false};

  // Progress, from ProgressMonitor; whole run. -1: never happened.
  Timestep first_deadlock_step{-1};  // formation step of the first deadlock
  int64_t deadlocks{0};              // wait-for cycles that lasted ProgressMonitor::kConfirmSteps
  int32_t deadlocked_robots{0};      // on a deadlock at the end
  int32_t blocked_behind{0};         // waiting on a chain into a deadlock at the end
  int32_t stalled_robots{0};         // no progress for ProgressMonitor::kStallSteps at the end
  int64_t stalled_robot_steps{0};
  Timestep frozen_at{-1};  // from here to the end no robot made progress, work left
  Timestep last_delivery_step{-1};

  uint64_t log_hash{0};  // fingerprint of the trajectory; same seed and config, same hash
  double wall_seconds{0.0};
};

// Accumulates a run in memory and fingerprints it. The fingerprint covers every robot's cell,
// status, and outcome at every step, plus every pickup and delivery; it excludes wall time.
class RunStats {
 public:
  RunStats(int32_t fleet_size, Timestep warmup);

  void recordOrders(const std::vector<Task>& orders);
  // Orders the coordinator assigned in this tick; only the first assignment of an order counts.
  void recordAssignments(Timestep t, const std::vector<TaskId>& orders);
  void recordStep(const StepTruth& step, const TruthSnapshot& after, const TickStats& tick);

  RunSummary summarize(const Config& config, Timestep steps_run, double wall_seconds,
                       int64_t violations, bool aborted) const;

  uint64_t hash() const { return hash_; }

 private:
  void mix(int64_t value);

  int32_t fleet_size_;
  Timestep warmup_;
  uint64_t hash_{0xCBF29CE484222325ULL};
  std::map<TaskId, Timestep> created_at_{};
  int64_t orders_created_{0};
  int64_t delivered_total_{0};
  int64_t delivered_steady_{0};
  std::vector<double> completion_steps_{};
  int64_t holding_robot_steps_{0};
  int64_t conflicts_{0};
  int64_t congestion_steps_{0};
  std::vector<double> planning_ms_{};
  int64_t expansions_{0};
  int64_t planner_failures_{0};
  int64_t delayed_steps_{0};
  int64_t yields_{0};
  int64_t traffic_holds_{0};
  int64_t deadlocks_resolved_{0};
  std::map<TaskId, Timestep> open_{};  // created, not yet delivered
  std::set<TaskId> delivered_{};       // an order delivered again counts once (Stage 6)
  int64_t duplicate_deliveries_{0};
  int64_t no_command_steps_{0};
  int64_t safe_stop_steps_{0};
  // Stage 7.
  int64_t drive_to_pickup_{0};
  int64_t drive_loaded_{0};
  int64_t drive_free_{0};
  int64_t service_{0};
  int64_t traffic_{0};
  int64_t station_wait_{0};
  int64_t idle_{0};
  int64_t safety_stop_{0};
  int64_t lost_{0};
  std::map<TaskId, Timestep> assigned_at_{};  // first assignment
  std::map<TaskId, Timestep> picked_at_{};    // first pickup
  double wait_for_robot_sum_{0.0};
  double to_pickup_sum_{0.0};
  double to_dropoff_sum_{0.0};
};

}  // namespace fleet_core
