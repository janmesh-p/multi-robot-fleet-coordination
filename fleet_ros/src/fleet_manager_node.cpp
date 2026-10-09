// ROS 2 adapter around fleet_core. All coordination logic lives in fleet_core; this node only
// moves data between ROS topics and the coordinator (Stage 8).
//
// Params: config_file (fleet_core run config, YAML), tick_hz (1 step = 1 s, so 1.0 is real time)
// Subs:   /fleet/robot_state (RobotState), /fleet/blocked (BlockedCells)
// Pubs:   /fleet/paths (TimedPath)
#include <chrono>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "fleet_core/common/config.hpp"
#include "fleet_msgs/msg/blocked_cells.hpp"
#include "fleet_msgs/msg/robot_state.hpp"
#include "fleet_msgs/msg/timed_path.hpp"

class FleetManagerNode : public rclcpp::Node {
 public:
  FleetManagerNode() : Node("fleet_manager") {
    const std::string config_file = declare_parameter<std::string>("config_file", "");
    const double tick_hz = declare_parameter<double>("tick_hz", 1.0);
    if (config_file.empty()) throw std::runtime_error("parameter 'config_file' is required");
    if (!(tick_hz > 0.0)) throw std::runtime_error("parameter 'tick_hz' must be > 0");

    config_ = fleet_core::loadConfig(config_file);
    RCLCPP_INFO(get_logger(), "run '%s': %d robots, allocation=%s, planning=%s",
                config_.run.name.c_str(), static_cast<int>(config_.fleet.size),
                std::string(fleet_core::toString(config_.allocation.method)).c_str(),
                std::string(fleet_core::toString(config_.planning.method)).c_str());

    paths_pub_ = create_publisher<fleet_msgs::msg::TimedPath>("/fleet/paths", 10);
    state_sub_ = create_subscription<fleet_msgs::msg::RobotState>(
        "/fleet/robot_state", rclcpp::SensorDataQoS(),
        [this](const fleet_msgs::msg::RobotState::ConstSharedPtr msg) { onRobotState(*msg); });
    blocked_sub_ = create_subscription<fleet_msgs::msg::BlockedCells>(
        "/fleet/blocked", 10,
        [this](const fleet_msgs::msg::BlockedCells::ConstSharedPtr msg) { onBlocked(*msg); });
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / tick_hz), [this] { onTick(); });
  }

 private:
  // TODO(stage8): buffer as fleet_core::RobotReport for the next tick.
  void onRobotState(const fleet_msgs::msg::RobotState& /*msg*/) {}
  // TODO(stage8): convert to fleet_core::CellObservation.
  void onBlocked(const fleet_msgs::msg::BlockedCells& /*msg*/) {}
  // TODO(stage8): FleetManager::tick on the buffered input, publish the commands.
  void onTick() {}

  fleet_core::Config config_;
  rclcpp::Publisher<fleet_msgs::msg::TimedPath>::SharedPtr paths_pub_;
  rclcpp::Subscription<fleet_msgs::msg::RobotState>::SharedPtr state_sub_;
  rclcpp::Subscription<fleet_msgs::msg::BlockedCells>::SharedPtr blocked_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    rclcpp::spin(std::make_shared<FleetManagerNode>());
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("fleet_manager"), "%s", e.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
