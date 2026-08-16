#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

class SafetyGateNode : public rclcpp::Node
{
public:
  SafetyGateNode()
  : Node("safety_gate")
  {
    rclcpp::QoS estop_qos(1);
    estop_qos.reliable();
    estop_qos.transient_local();

    cmd_vel_publisher_ =
      create_publisher<geometry_msgs::msg::Twist>(
      "cmd_vel",
      rclcpp::QoS(10));

    nav_cmd_vel_subscription_ =
      create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_nav",
      rclcpp::QoS(10),
      [this](const geometry_msgs::msg::Twist::SharedPtr message)
      {
        nav_cmd_vel_callback(*message);
      });

    estop_subscription_ =
      create_subscription<std_msgs::msg::Bool>(
      "estop",
      estop_qos,
      [this](const std_msgs::msg::Bool::SharedPtr message)
      {
        estop_callback(*message);
      });

    RCLCPP_INFO(
      get_logger(),
      "Safety gate started: 'cmd_vel_nav' + 'estop' -> 'cmd_vel'");
  }

private:
  void nav_cmd_vel_callback(
    const geometry_msgs::msg::Twist & message)
  {
    if (estop_active_) {
      publish_zero();
      return;
    }

    cmd_vel_publisher_->publish(message);
  }

  void estop_callback(
    const std_msgs::msg::Bool & message)
  {
    const bool was_active = estop_active_;
    estop_active_ = message.data;

    if (estop_active_ && !was_active) {
      RCLCPP_WARN(
        get_logger(),
        "E-STOP engaged: forcing cmd_vel to zero");

      publish_zero();
    } else if (!estop_active_ && was_active) {
      RCLCPP_INFO(
        get_logger(),
        "E-STOP released: passing through Nav2 velocity commands");
    }
  }

  void publish_zero()
  {
    geometry_msgs::msg::Twist zero;
    cmd_vel_publisher_->publish(zero);
  }

  bool estop_active_{false};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
    cmd_vel_publisher_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
    nav_cmd_vel_subscription_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    estop_subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(
      std::make_shared<SafetyGateNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("safety_gate"),
      "Unhandled exception: %s",
      exception.what());

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
