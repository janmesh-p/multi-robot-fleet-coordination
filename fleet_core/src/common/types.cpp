#include "fleet_core/common/types.hpp"

namespace fleet_core {

std::string_view toString(RobotStatus status) {
  switch (status) {
    case RobotStatus::kIdle:
      return "idle";
    case RobotStatus::kToPickup:
      return "to_pickup";
    case RobotStatus::kPicking:
      return "picking";
    case RobotStatus::kToDropoff:
      return "to_dropoff";
    case RobotStatus::kDropping:
      return "dropping";
    case RobotStatus::kSafeStop:
      return "safe_stop";
    case RobotStatus::kDead:
      return "dead";
  }
  return "unknown";
}

std::string_view toString(TaskStatus status) {
  switch (status) {
    case TaskStatus::kPending:
      return "pending";
    case TaskStatus::kAssigned:
      return "assigned";
    case TaskStatus::kPickedUp:
      return "picked_up";
    case TaskStatus::kDone:
      return "done";
  }
  return "unknown";
}

}  // namespace fleet_core
