// ROS 2 wrapper around fleet_core::FleetManager.
// Subs:  /fleet/robot_state (RobotState), /fleet/blocked (BlockedCells)
// Pubs:  /fleet/paths (TimedPath), /fleet/tasks (Task)
#include <rclcpp/rclcpp.hpp>

#include "fleet_core/fleet_manager.hpp"
#include "fleet_msgs/msg/blocked_cells.hpp"
#include "fleet_msgs/msg/robot_state.hpp"
#include "fleet_msgs/msg/task.hpp"
#include "fleet_msgs/msg/timed_path.hpp"

class FleetManagerNode : public rclcpp::Node {
 public:
  FleetManagerNode() : Node("fleet_manager") {
    cfg_.map_path = declare_parameter<std::string>("map_path", "");
    cfg_.allocator = declare_parameter<std::string>("allocator", "nearest");
    cfg_.planner = declare_parameter<std::string>("planner", "independent_astar");
    cfg_.fleet_size = declare_parameter<int>("fleet_size", 5);
    const double tick_hz = declare_parameter<double>("tick_hz", 10.0);

    // TODO(stage1): construct FleetManager once allocator/planner exist.
    paths_pub_ = create_publisher<fleet_msgs::msg::TimedPath>("/fleet/paths", 10);
    state_sub_ = create_subscription<fleet_msgs::msg::RobotState>(
        "/fleet/robot_state", rclcpp::SensorDataQoS(),
        [this](fleet_msgs::msg::RobotState::ConstSharedPtr /*msg*/) {
          // TODO: convert and call manager_->updateRobot(...)
        });
    blocked_sub_ = create_subscription<fleet_msgs::msg::BlockedCells>(
        "/fleet/blocked", 10,
        [this](fleet_msgs::msg::BlockedCells::ConstSharedPtr /*msg*/) {
          // TODO(stage5/9): update dynamic blocks, trigger replan
        });
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / tick_hz), [this] {
      // TODO: manager_->tick(step_++); publish activePaths()
    });
    RCLCPP_INFO(get_logger(), "fleet_manager up (skeleton)");
  }

 private:
  fleet_core::FleetConfig cfg_;
  std::unique_ptr<fleet_core::FleetManager> manager_;
  int32_t step_{0};
  rclcpp::Publisher<fleet_msgs::msg::TimedPath>::SharedPtr paths_pub_;
  rclcpp::Subscription<fleet_msgs::msg::RobotState>::SharedPtr state_sub_;
  rclcpp::Subscription<fleet_msgs::msg::BlockedCells>::SharedPtr blocked_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FleetManagerNode>());
  rclcpp::shutdown();
  return 0;
}
