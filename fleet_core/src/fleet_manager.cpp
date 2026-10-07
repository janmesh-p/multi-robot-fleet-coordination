#include "fleet_core/fleet_manager.hpp"

namespace fleet_core {

FleetManager::FleetManager(const FleetConfig& cfg)
    : cfg_(cfg), map_(GridMap::fromFile(cfg.map_path)) {
  // Allocator and planner are created once their stages land:
  //   allocator_ = makeAllocator(cfg_.allocator);
  //   planner_   = makePlanner(cfg_.planner);
  // TODO(stage1): spawn cfg_.fleet_size robots on free cells.
}

void FleetManager::tick(Timestep /*t*/) {
  // TODO(stage1-6):
  //  1. mark robots dead if heartbeat older than cfg_.heartbeat_timeout
  //  2. return their tasks to pending
  //  3. allocator_->assign(...)
  //  4. planner_->plan(...) for robots with new goals or invalidated paths
  //  5. publish paths (transport layer)
}

void FleetManager::updateRobot(const RobotState& s) {
  for (auto& r : robots_) {
    if (r.id == s.id) { r = s; return; }
  }
  robots_.push_back(s);
}

}  // namespace fleet_core
