#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/time.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

class SystemReadinessSupervisor : public rclcpp::Node
{
public:
  using ManageLifecycleNodes =
    nav2_msgs::srv::ManageLifecycleNodes;

  SystemReadinessSupervisor()
  : Node("system_readiness_supervisor"),
    ready_qos_(1),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    robot_ids_ = declare_parameter<std::vector<std::string>>(
      "robot_ids",
      std::vector<std::string>{"robot1", "robot2"});

    readiness_check_period_seconds_ =
      declare_parameter<double>(
      "readiness_check_period_seconds",
      1.0);

    startup_retry_period_seconds_ =
      declare_parameter<double>(
      "startup_retry_period_seconds",
      5.0);

    tf_timeout_seconds_ =
      declare_parameter<double>(
      "tf_timeout_seconds",
      0.2);

    ready_qos_.reliable();
    ready_qos_.transient_local();

    ready_publisher_ =
      create_publisher<std_msgs::msg::Bool>(
      "/system/ready",
      ready_qos_);

    clock_subscription_ =
      create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock",
      rclcpp::QoS(10).best_effort(),
      std::bind(
        &SystemReadinessSupervisor::clock_callback,
        this,
        std::placeholders::_1));

    for (const auto & robot_id : robot_ids_) {
      robot_readiness_[robot_id] = RobotReadiness{};

      const std::string map_topic =
        "/" + robot_id + "/map";

      auto map_subscription =
        create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic,
        rclcpp::QoS(1)
        .reliable()
        .transient_local(),
        [this, robot_id](
          const nav_msgs::msg::OccupancyGrid::SharedPtr message)
        {
          map_callback(robot_id, *message);
        });

      map_subscriptions_.push_back(map_subscription);

      const std::string lifecycle_service =
        "/" + robot_id +
        "/lifecycle_manager_navigation/manage_nodes";

      lifecycle_clients_[robot_id] =
        create_client<ManageLifecycleNodes>(
        lifecycle_service);

      RCLCPP_INFO(
        get_logger(),
        "Monitoring robot '%s': map='%s', lifecycle='%s'",
        robot_id.c_str(),
        map_topic.c_str(),
        lifecycle_service.c_str());
    }

    publish_ready(false);

    const auto check_period =
      std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(
        readiness_check_period_seconds_));

    readiness_timer_ =
      create_wall_timer(
      check_period,
      std::bind(
        &SystemReadinessSupervisor::check_readiness,
        this));

    RCLCPP_INFO(
      get_logger(),
      "System readiness supervisor started");
  }

private:
  struct RobotReadiness
  {
    bool map_received{false};
    bool tf_available{false};
    bool lifecycle_service_available{false};
    bool startup_requested{false};
    bool startup_succeeded{false};
    rclcpp::Time last_startup_attempt{
      0,
      0,
      RCL_ROS_TIME};
  };

  void clock_callback(
    const rosgraph_msgs::msg::Clock::SharedPtr message)
  {
    clock_received_ = true;
    last_clock_ = message->clock;
  }

  void map_callback(
    const std::string & robot_id,
    const nav_msgs::msg::OccupancyGrid & message)
  {
    auto iterator = robot_readiness_.find(robot_id);

    if (iterator == robot_readiness_.end()) {
      return;
    }

    const bool valid_map =
      message.info.width > 0 &&
      message.info.height > 0 &&
      message.info.resolution > 0.0;

    iterator->second.map_received = valid_map;

    if (!valid_map) {
      RCLCPP_WARN(
        get_logger(),
        "Robot '%s' published an invalid map",
        robot_id.c_str());
    }
  }

  bool update_tf_readiness(
    const std::string & robot_id)
  {
    const std::string odom_frame =
      robot_id + "/odom";

    const std::string base_frame =
      robot_id + "/base_link";

    try {
      const bool available =
        tf_buffer_.canTransform(
        odom_frame,
        base_frame,
        tf2::TimePointZero,
        tf2::durationFromSec(
          tf_timeout_seconds_));

      robot_readiness_[robot_id].tf_available =
        available;

      return available;
    } catch (const tf2::TransformException & exception) {
      robot_readiness_[robot_id].tf_available =
        false;

      RCLCPP_DEBUG(
        get_logger(),
        "TF unavailable for '%s': %s",
        robot_id.c_str(),
        exception.what());

      return false;
    }
  }

  bool update_lifecycle_service_readiness(
    const std::string & robot_id)
  {
    const auto client_iterator =
      lifecycle_clients_.find(robot_id);

    if (client_iterator == lifecycle_clients_.end()) {
      return false;
    }

    const bool ready =
      client_iterator->second->service_is_ready();

    robot_readiness_[robot_id]
      .lifecycle_service_available = ready;

    return ready;
  }

  bool robot_inputs_ready(
    const std::string & robot_id)
  {
    auto & readiness = robot_readiness_[robot_id];

    update_tf_readiness(robot_id);
    update_lifecycle_service_readiness(robot_id);

    return
      readiness.map_received &&
      readiness.tf_available &&
      readiness.lifecycle_service_available;
  }

  bool should_retry_startup(
    const RobotReadiness & readiness) const
  {
    if (!readiness.startup_requested) {
      return true;
    }

    if (readiness.startup_succeeded) {
      return false;
    }

    const double elapsed_seconds =
      (now() - readiness.last_startup_attempt)
      .seconds();

    return
      elapsed_seconds >= startup_retry_period_seconds_;
  }

  void request_nav2_startup(
    const std::string & robot_id)
  {
    auto client_iterator =
      lifecycle_clients_.find(robot_id);

    if (client_iterator == lifecycle_clients_.end()) {
      return;
    }

    auto & readiness =
      robot_readiness_[robot_id];

    if (!should_retry_startup(readiness)) {
      return;
    }

    auto request =
      std::make_shared<
        ManageLifecycleNodes::Request>();

    request->command =
      ManageLifecycleNodes::Request::STARTUP;

    readiness.startup_requested = true;
    readiness.last_startup_attempt = now();

    RCLCPP_INFO(
      get_logger(),
      "Requesting Nav2 startup for robot '%s'",
      robot_id.c_str());

    client_iterator->second->async_send_request(
      request,
      [this, robot_id](
        rclcpp::Client<
          ManageLifecycleNodes>::SharedFuture future)
      {
        try {
          const auto response = future.get();

          auto & robot =
            robot_readiness_[robot_id];

          robot.startup_succeeded =
            response->success;

          if (response->success) {
            RCLCPP_INFO(
              get_logger(),
              "Nav2 startup succeeded for robot '%s'",
              robot_id.c_str());
          } else {
            RCLCPP_WARN(
              get_logger(),
              "Nav2 startup was rejected for robot '%s'",
              robot_id.c_str());
          }
        } catch (const std::exception & exception) {
          robot_readiness_[robot_id]
            .startup_succeeded = false;

          RCLCPP_ERROR(
            get_logger(),
            "Nav2 startup request failed for robot '%s': %s",
            robot_id.c_str(),
            exception.what());
        }
      });
  }

  bool all_robots_started() const
  {
    for (const auto & robot_id : robot_ids_) {
      const auto iterator =
        robot_readiness_.find(robot_id);

      if (
        iterator == robot_readiness_.end() ||
        !iterator->second.startup_succeeded)
      {
        return false;
      }
    }

    return true;
  }

  void check_readiness()
  {
    if (!clock_received_) {
      log_waiting_once(
        "clock",
        "Waiting for simulation clock");
      publish_ready(false);
      return;
    }

    bool all_inputs_ready = true;

    for (const auto & robot_id : robot_ids_) {
      const bool ready =
        robot_inputs_ready(robot_id);

      if (!ready) {
        all_inputs_ready = false;

        const auto & state =
          robot_readiness_[robot_id];

        RCLCPP_DEBUG(
          get_logger(),
          "Robot '%s' readiness: map=%s tf=%s lifecycle=%s",
          robot_id.c_str(),
          state.map_received ? "ready" : "waiting",
          state.tf_available ? "ready" : "waiting",
          state.lifecycle_service_available
            ? "ready"
            : "waiting");
      }
    }

    if (!all_inputs_ready) {
      publish_ready(false);
      return;
    }

    for (const auto & robot_id : robot_ids_) {
      request_nav2_startup(robot_id);
    }

    const bool ready = all_robots_started();
    publish_ready(ready);

    if (ready && !ready_logged_) {
      ready_logged_ = true;

      RCLCPP_INFO(
        get_logger(),
        "System is ready: all Nav2 stacks are active");
    }
  }

  void publish_ready(bool ready)
  {
    if (
      has_published_ready_ &&
      last_published_ready_ == ready)
    {
      return;
    }

    std_msgs::msg::Bool message;
    message.data = ready;

    ready_publisher_->publish(message);

    has_published_ready_ = true;
    last_published_ready_ = ready;

    RCLCPP_INFO(
      get_logger(),
      "System readiness: %s",
      ready ? "READY" : "NOT_READY");
  }

  void log_waiting_once(
    const std::string & key,
    const std::string & message)
  {
    if (waiting_log_keys_.count(key) > 0) {
      return;
    }

    waiting_log_keys_.insert(key);

    RCLCPP_INFO(
      get_logger(),
      "%s",
      message.c_str());
  }

  std::vector<std::string> robot_ids_;

  double readiness_check_period_seconds_;
  double startup_retry_period_seconds_;
  double tf_timeout_seconds_;

  bool clock_received_{false};
  builtin_interfaces::msg::Time last_clock_;

  bool has_published_ready_{false};
  bool last_published_ready_{false};
  bool ready_logged_{false};

  std::unordered_map<
    std::string,
    RobotReadiness>
    robot_readiness_;

  std::unordered_map<
    std::string,
    rclcpp::Client<
      ManageLifecycleNodes>::SharedPtr>
    lifecycle_clients_;

  std::vector<
    rclcpp::Subscription<
      nav_msgs::msg::OccupancyGrid>::SharedPtr>
    map_subscriptions_;

  rclcpp::Subscription<
    rosgraph_msgs::msg::Clock>::SharedPtr
    clock_subscription_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
    ready_publisher_;

  rclcpp::TimerBase::SharedPtr
    readiness_timer_;

  rclcpp::QoS ready_qos_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::unordered_set<std::string>
    waiting_log_keys_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(
      std::make_shared<
        SystemReadinessSupervisor>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger(
        "system_readiness_supervisor"),
      "Unhandled exception: %s",
      exception.what());

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}