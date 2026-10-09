#pragma once

#include <vector>

#include "fleet_core/common/types.hpp"

namespace fleet_core {

// The robots, as the coordinator sees them. Backends:
//   grid simulator   simulated robots, ground truth, link loss, delays   (Stage 1, 5, 6)
//   Isaac bridge     Nav2 robots in Isaac Sim                             (Stage 8)
//   UDP bridge       STM32 robot client over WiFi                         (Stage 9a)
//
// Robot-side behavior lives behind this interface: follow the newest command up to its permit,
// dwell at stations, safe-stop when commands stop arriving. Ground truth never leaves the
// backend except through reports, exactly as with real robots.
class RobotExecutor {
 public:
  virtual ~RobotExecutor() = default;

  // Reports produced since the last call that survived the link.
  virtual std::vector<RobotReport> collectReports(Timestep now) = 0;

  // Sends commands over the link. A lost command is simply never seen by its robot.
  virtual void dispatch(Timestep now, const std::vector<RobotCommand>& commands) = 0;

  // Advances the robots by one step. Real-time backends may treat this as a no-op.
  virtual void step(Timestep now) = 0;
};

}  // namespace fleet_core
