#include "fleet_core/faults/fault_injector.hpp"

#include <algorithm>
#include <utility>

namespace fleet_core {

FaultInjector::FaultInjector(std::vector<FaultEvent> schedule) : schedule_(std::move(schedule)) {
  std::stable_sort(schedule_.begin(), schedule_.end(),
                   [](const FaultEvent& a, const FaultEvent& b) { return a.at < b.at; });
}

std::vector<FaultEvent> FaultInjector::due(Timestep now) {
  std::vector<FaultEvent> out;
  while (cursor_ < schedule_.size() && schedule_[cursor_].at <= now) {
    out.push_back(schedule_[cursor_]);
    ++cursor_;
  }
  return out;
}

}  // namespace fleet_core
