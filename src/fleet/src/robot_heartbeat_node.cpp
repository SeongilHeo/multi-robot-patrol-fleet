#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include "interfaces/msg/robot_heartbeat.hpp"

using namespace std::chrono_literals;

class RobotHeartbeatNode : public rclcpp::Node
{
public:
  using RobotHeartbeat =
    interfaces::msg::RobotHeartbeat;

  RobotHeartbeatNode()
  : Node("robot_heartbeat")
  {
    robot_id_ = declare_parameter<std::string>(
      "robot_id",
      "robot1");

    battery_percentage_ = declare_parameter<double>(
      "battery_percentage",
      100.0);

    default_state_ = declare_parameter<int>(
      "default_state",
      RobotHeartbeat::STATE_IDLE);

    default_mission_ = declare_parameter<std::string>(
      "default_mission",
      "");

    state_ = clamp_state(default_state_);
    current_mission_ = default_mission_;

    publisher_ =
      create_publisher<RobotHeartbeat>(
        "heartbeat",
        rclcpp::QoS(10).reliable());

    rclcpp::QoS patrol_status_qos(1);
    patrol_status_qos.reliable();
    patrol_status_qos.transient_local();

    patrol_status_subscription_ =
      create_subscription<std_msgs::msg::String>(
        "patrol/status",
        patrol_status_qos,
        std::bind(
          &RobotHeartbeatNode::patrol_status_callback,
          this,
          std::placeholders::_1));

    mission_type_subscription_ =
      create_subscription<std_msgs::msg::String>(
        "patrol/mission_type",
        patrol_status_qos,
        [this](const std_msgs::msg::String::SharedPtr message)
        {
          last_reported_mission_type_ = message->data;
        });

    rclcpp::QoS localization_ok_qos(1);
    localization_ok_qos.reliable();
    localization_ok_qos.transient_local();

    localization_ok_subscription_ =
      create_subscription<std_msgs::msg::Bool>(
        "localization_ok",
        localization_ok_qos,
        [this](const std_msgs::msg::Bool::SharedPtr message)
        {
          localization_ok_ = message->data;
        });

    timer_ = create_wall_timer(
      1s,
      std::bind(
        &RobotHeartbeatNode::publish_heartbeat,
        this));

    RCLCPP_INFO(
      get_logger(),
      "Heartbeat state aggregator started for robot '%s'",
      robot_id_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "Monitoring patrol status on '/%s/patrol/status'",
      robot_id_.c_str());
  }

private:
  static uint8_t clamp_state(int state)
  {
    return static_cast<uint8_t>(
      std::clamp(
        state,
        static_cast<int>(
          RobotHeartbeat::STATE_UNKNOWN),
        static_cast<int>(
          RobotHeartbeat::STATE_ERROR)));
  }

  void patrol_status_callback(
    const std_msgs::msg::String::SharedPtr message)
  {
    const std::string & status = message->data;

    if (
      status == "waiting_for_nav2" ||
      status == "sending_goal" ||
      status == "active" ||
      status == "canceling")
    {
      state_ = RobotHeartbeat::STATE_PATROLLING;
      current_mission_ =
        last_reported_mission_type_.empty()
        ? "patrol"
        : last_reported_mission_type_;
    }
    else if (
      status == "idle" ||
      status == "completed" ||
      status == "canceled")
    {
      state_ = RobotHeartbeat::STATE_IDLE;
      current_mission_.clear();
    }
    else if (
      status == "aborted" ||
      status == "rejected" ||
      status == "error")
    {
      state_ = RobotHeartbeat::STATE_ERROR;
      current_mission_ =
        last_reported_mission_type_.empty()
        ? "patrol"
        : last_reported_mission_type_;
    }
    else
    {
      RCLCPP_WARN(
        get_logger(),
        "Unknown patrol status '%s' for robot '%s'",
        status.c_str(),
        robot_id_.c_str());

      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "Robot '%s' local state updated from patrol status '%s'",
      robot_id_.c_str(),
      status.c_str());
  }

  void publish_heartbeat()
  {
    RobotHeartbeat message;

    message.robot_id = robot_id_;
    message.stamp = now();

    message.state = state_;

    message.battery_percentage =
      static_cast<float>(
        std::clamp(
          battery_percentage_,
          0.0,
          100.0));

    message.current_mission =
      current_mission_;

    message.localization_ok =
      localization_ok_;

    publisher_->publish(message);
  }

  std::string robot_id_;
  double battery_percentage_;

  int default_state_;
  std::string default_mission_;

  uint8_t state_{
    RobotHeartbeat::STATE_UNKNOWN};

  std::string current_mission_;
  std::string last_reported_mission_type_;

  bool localization_ok_{true};

  rclcpp::Publisher<RobotHeartbeat>::SharedPtr
    publisher_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    patrol_status_subscription_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    mission_type_subscription_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    localization_ok_subscription_;

  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  try
  {
    rclcpp::spin(
      std::make_shared<RobotHeartbeatNode>());
  }
  catch (const std::exception & exception)
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