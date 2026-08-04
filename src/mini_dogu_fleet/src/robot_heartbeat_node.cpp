#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "mini_dogu_interfaces/msg/robot_heartbeat.hpp"

using namespace std::chrono_literals;

class RobotHeartbeatNode : public rclcpp::Node
{
public:
  RobotHeartbeatNode()
    : Node("robot_heartbeat")
  {
    robot_id_ = declare_parameter<std::string>(
      "robot_id",
      "robot1");

    battery_percentage_ = declare_parameter<double>(
      "battery_percentage",
      100.0);

    state_ = declare_parameter<int>(
      "state",
      mini_dogu_interfaces::msg::RobotHeartbeat::STATE_IDLE);

    current_mission_ = declare_parameter<std::string>(
      "current_mission",
      "");

    publisher_ =
      create_publisher<mini_dogu_interfaces::msg::RobotHeartbeat>(
        "heartbeat",
        rclcpp::QoS(10).reliable());

    timer_ = create_wall_timer(
      1s,
      std::bind(&RobotHeartbeatNode::publish_heartbeat, this));

    RCLCPP_INFO(
      get_logger(),
      "Heartbeat publisher started for robot '%s'",
      robot_id_.c_str());
  }

private:
  void publish_heartbeat()
  {
    mini_dogu_interfaces::msg::RobotHeartbeat message;

    message.robot_id = robot_id_;
    message.stamp = now();

    message.state = static_cast<uint8_t>(
      std::clamp(
        state_,
        static_cast<int>(
          mini_dogu_interfaces::msg::RobotHeartbeat::STATE_UNKNOWN),
        static_cast<int>(
          mini_dogu_interfaces::msg::RobotHeartbeat::STATE_ERROR)));

    message.battery_percentage = static_cast<float>(
      std::clamp(battery_percentage_, 0.0, 100.0));

    message.current_mission = current_mission_;

    publisher_->publish(message);
  }

  std::string robot_id_;
  double battery_percentage_;
  int state_;
  std::string current_mission_;

  rclcpp::Publisher<
    mini_dogu_interfaces::msg::RobotHeartbeat>::SharedPtr publisher_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  try
  {
    rclcpp::spin(std::make_shared<RobotHeartbeatNode>());
  }
  catch (const std::exception& exception)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("robot_heartbeat"),
      "Unhandled exception: %s",
      exception.what());

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}