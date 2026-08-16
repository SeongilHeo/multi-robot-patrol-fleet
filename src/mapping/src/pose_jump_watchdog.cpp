#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/time.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

/*
 * This is a cheap proxy for localization confidence, not a real one.
 * slam_toolbox's online async mode does not expose a covariance the way
 * AMCL does, so instead of pretending to measure confidence, this watches
 * for map->base_link teleporting further than the robot could physically
 * have moved between checks and reports that as "localization not ok".
 * A steady lock does not mean the map is metrically correct, only that it
 * has not visibly jumped.
 */
class PoseJumpWatchdog : public rclcpp::Node
{
public:
  PoseJumpWatchdog()
  : Node("pose_jump_watchdog"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    map_frame_ = declare_parameter<std::string>(
      "map_frame",
      "robot1/map");

    base_frame_ = declare_parameter<std::string>(
      "base_frame",
      "robot1/base_link");

    check_period_seconds_ = declare_parameter<double>(
      "check_period_seconds",
      0.2);

    max_linear_jump_meters_ = declare_parameter<double>(
      "max_linear_jump_meters",
      1.0);

    hold_seconds_ = declare_parameter<double>(
      "hold_seconds",
      5.0);

    rclcpp::QoS status_qos(1);
    status_qos.reliable();
    status_qos.transient_local();

    localization_ok_publisher_ =
      create_publisher<std_msgs::msg::Bool>(
      "localization_ok",
      status_qos);

    publish_status(true);

    const auto period =
      std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(check_period_seconds_));

    check_timer_ =
      create_wall_timer(
      period,
      std::bind(&PoseJumpWatchdog::check, this));

    RCLCPP_INFO(
      get_logger(),
      "Pose jump watchdog watching '%s' -> '%s'",
      map_frame_.c_str(),
      base_frame_.c_str());
  }

private:
  void check()
  {
    const rclcpp::Time current_ros_time = now();

    if (
      localization_ok_ == false &&
      last_jump_at_.has_value() &&
      (current_ros_time - *last_jump_at_).seconds() >=
      hold_seconds_)
    {
      publish_status(true);
    }

    if (
      !tf_buffer_.canTransform(
        map_frame_,
        base_frame_,
        tf2::TimePointZero))
    {
      return;
    }

    geometry_msgs::msg::TransformStamped transform;

    try {
      transform =
        tf_buffer_.lookupTransform(
        map_frame_,
        base_frame_,
        tf2::TimePointZero);
    } catch (const tf2::TransformException & exception) {
      RCLCPP_DEBUG(
        get_logger(),
        "Transform lookup failed: %s",
        exception.what());
      return;
    }

    const double x = transform.transform.translation.x;
    const double y = transform.transform.translation.y;

    if (last_position_.has_value()) {
      const double dx = x - last_position_->first;
      const double dy = y - last_position_->second;

      const double distance =
        std::sqrt(dx * dx + dy * dy);

      if (distance > max_linear_jump_meters_) {
        last_jump_at_ = current_ros_time;

        if (localization_ok_) {
          RCLCPP_WARN(
            get_logger(),
            "Pose jumped %.2f m in one check period (limit %.2f m); "
            "flagging localization as not ok",
            distance,
            max_linear_jump_meters_);
        }

        publish_status(false);
      }
    }

    last_position_ = std::make_pair(x, y);
  }

  void publish_status(bool ok)
  {
    localization_ok_ = ok;

    std_msgs::msg::Bool message;
    message.data = ok;

    localization_ok_publisher_->publish(message);
  }

  std::string map_frame_;
  std::string base_frame_;

  double check_period_seconds_;
  double max_linear_jump_meters_;
  double hold_seconds_;

  bool localization_ok_{true};

  std::optional<std::pair<double, double>> last_position_;
  std::optional<rclcpp::Time> last_jump_at_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
    localization_ok_publisher_;

  rclcpp::TimerBase::SharedPtr check_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(
      std::make_shared<PoseJumpWatchdog>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("pose_jump_watchdog"),
      "Unhandled exception: %s",
      exception.what());

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
