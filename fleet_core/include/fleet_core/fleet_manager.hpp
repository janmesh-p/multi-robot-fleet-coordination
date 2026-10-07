#pragma once

#include <memory>
#include <string>
#include <vector>

#include "fleet_core/fault_injector.hpp"
#include "fleet_core/grid_map.hpp"
#include "fleet_core/mapf_planner.hpp"
#include "fleet_core/metrics_logger.hpp"
#include "fleet_core/order_generator.hpp"
#include "fleet_core/task_allocator.hpp"
#include "fleet_core/traffic_manager.hpp"

namespace fleet_core {

struct FleetConfig {
  std::string map_path{};
  std::string allocator{"nearest"};
  std::string planner{"independent_astar"};
  int32_t fleet_size{5};
  double orders_per_step{0.1};
  Timestep horizon{1000};
  Timestep heartbeat_timeout{5};
  uint32_t seed{42};
  std::string out_dir{"benchmarks/results/run"};
};

// Central coordinator. Same class drives grid sim, Isaac Sim bridge, and HIL.
// Transport (ROS 2 or direct) lives outside fleet_core.
class FleetManager {
 public:
  explicit FleetManager(const FleetConfig& cfg);

  void tick(Timestep t);                       // allocate, plan, dispatch
  void updateRobot(const RobotState& s);       // pose / heartbeat in
  const std::vector<Path>& activePaths() const { return paths_; }
  const std::vector<RobotState>& robots() const { return robots_; }

 private:
  FleetConfig cfg_;
  GridMap map_;
  std::unique_ptr<TaskAllocator> allocator_;
  std::unique_ptr<MAPFPlanner> planner_;
  std::vector<RobotState> robots_{};
  std::vector<Task> tasks_{};
  std::vector<Path> paths_{};
};

}  // namespace fleet_core
