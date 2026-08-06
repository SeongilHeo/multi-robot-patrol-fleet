#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
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

  using GetState =
    lifecycle_msgs::srv::GetState;

  SystemReadinessSupervisor()
  : Node("system_readiness_supervisor"),
    ready_qos_(1),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    robot_ids_ =
      declare_parameter<std::vector<std::string>>(
      "robot_ids",
      std::vector<std::string>{
        "robot1",
        "robot2"
      });

    readiness_check_period_seconds_ =
      declare_parameter<double>(
      "readiness_check_period_seconds",
      1.0);

    startup_retry_period_seconds_ =
      declare_parameter<double>(
      "startup_retry_period_seconds",
      5.0);

    reset_retry_period_seconds_ =
      declare_parameter<double>(
      "reset_retry_period_seconds",
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

    for (const auto & robot_id : robot_ids_) {
      robots_.emplace(
        robot_id,
        RobotSupervisor{});

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

      map_subscriptions_.push_back(
        map_subscription);

      const std::string lifecycle_service =
        "/" + robot_id +
        "/lifecycle_manager_navigation/manage_nodes";

      lifecycle_clients_[robot_id] =
        create_client<ManageLifecycleNodes>(
        lifecycle_service);

      for (const auto & node_name : nav2_node_names_) {
        const std::string state_service =
          "/" + robot_id +
          "/" + node_name +
          "/get_state";

        lifecycle_state_clients_[robot_id][node_name] =
          create_client<GetState>(
          state_service);
      }

      RCLCPP_INFO(
        get_logger(),
        "Monitoring robot '%s': map='%s', lifecycle='%s'",
        robot_id.c_str(),
        map_topic.c_str(),
        lifecycle_service.c_str());
    }

    publish_ready(false);

    const auto check_period =
      std::chrono::duration_cast<
      std::chrono::milliseconds>(
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
  /*
   * A robot always has exactly one active phase. Deciding whether to poll
   * lifecycle state or to issue STARTUP/RESET happens at the single point
   * where a poll finishes (finish_poll), never from a second pass over
   * stale flags on the next timer tick. That removes the possibility of
   * a freshly-started poll masking the result of the previous one.
   */
  enum class RobotPhase
  {
    kWaitingForInputs,
    kIdle,
    kPolling,
    kStartupPending,
    kResetPending,
    kReady,
  };

  struct RobotSupervisor
  {
    RobotPhase phase{RobotPhase::kWaitingForInputs};

    bool map_received{false};
    bool tf_available{false};
    bool lifecycle_service_available{false};

    rclcpp::Time last_startup_attempt{
      0,
      0,
      RCL_ROS_TIME};

    rclcpp::Time last_reset_attempt{
      0,
      0,
      RCL_ROS_TIME};
  };

  const std::vector<std::string> nav2_node_names_{
    "controller_server",
    "smoother_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "waypoint_follower"
  };

  RobotSupervisor * find_robot(
    const std::string & robot_id)
  {
    auto iterator =
      robots_.find(robot_id);

    if (iterator == robots_.end()) {
      return nullptr;
    }

    return &iterator->second;
  }

  void map_callback(
    const std::string & robot_id,
    const nav_msgs::msg::OccupancyGrid & message)
  {
    auto * robot =
      find_robot(robot_id);

    if (robot == nullptr) {
      return;
    }

    const bool valid_map =
      message.info.width > 0 &&
      message.info.height > 0 &&
      message.info.resolution > 0.0;

    const bool was_received =
      robot->map_received;

    robot->map_received =
      valid_map;

    if (valid_map && !was_received) {
      RCLCPP_INFO(
        get_logger(),
        "Map received from '%s': "
        "width=%u height=%u resolution=%.3f",
        robot_id.c_str(),
        message.info.width,
        message.info.height,
        message.info.resolution);
    }

    if (!valid_map) {
      RCLCPP_WARN(
        get_logger(),
        "Robot '%s' published an invalid map: "
        "width=%u height=%u resolution=%.3f",
        robot_id.c_str(),
        message.info.width,
        message.info.height,
        message.info.resolution);
    }
  }

  void update_input_readiness(
    const std::string & robot_id,
    RobotSupervisor & robot)
  {
    const std::string odom_frame =
      robot_id + "/odom";

    const std::string base_frame =
      robot_id + "/base_link";

    try {
      robot.tf_available =
        tf_buffer_.canTransform(
        odom_frame,
        base_frame,
        tf2::TimePointZero,
        tf2::durationFromSec(
          tf_timeout_seconds_));
    } catch (
      const tf2::TransformException & exception)
    {
      robot.tf_available = false;

      RCLCPP_DEBUG(
        get_logger(),
        "TF unavailable for '%s': %s",
        robot_id.c_str(),
        exception.what());
    }

    const auto client_iterator =
      lifecycle_clients_.find(robot_id);

    robot.lifecycle_service_available =
      client_iterator != lifecycle_clients_.end() &&
      client_iterator->second->service_is_ready();
  }

  void poll_lifecycle_states(
    const std::string & robot_id)
  {
    auto * robot =
      find_robot(robot_id);

    if (robot == nullptr) {
      return;
    }

    const auto clients_iterator =
      lifecycle_state_clients_.find(robot_id);

    if (
      clients_iterator ==
      lifecycle_state_clients_.end())
    {
      return;
    }

    for (const auto & node_name : nav2_node_names_) {
      const auto client_iterator =
        clients_iterator->second.find(node_name);

      if (
        client_iterator ==
        clients_iterator->second.end() ||
        !client_iterator->second->service_is_ready())
      {
        return;
      }
    }

    robot->phase =
      RobotPhase::kPolling;

    auto remaining =
      std::make_shared<std::size_t>(
      nav2_node_names_.size());

    auto all_active =
      std::make_shared<bool>(true);

    auto all_unconfigured =
      std::make_shared<bool>(true);

    auto query_failed =
      std::make_shared<bool>(false);

    for (const auto & node_name : nav2_node_names_) {
      auto request =
        std::make_shared<GetState::Request>();

      auto client =
        clients_iterator->second.at(node_name);

      client->async_send_request(
        request,
        [
          this,
          robot_id,
          node_name,
          remaining,
          all_active,
          all_unconfigured,
          query_failed
        ](
          rclcpp::Client<GetState>::SharedFuture future)
        {
          handle_get_state_response(
            robot_id,
            node_name,
            future,
            remaining,
            all_active,
            all_unconfigured,
            query_failed);
        });
    }
  }

  void handle_get_state_response(
    const std::string & robot_id,
    const std::string & node_name,
    rclcpp::Client<GetState>::SharedFuture future,
    const std::shared_ptr<std::size_t> & remaining,
    const std::shared_ptr<bool> & all_active,
    const std::shared_ptr<bool> & all_unconfigured,
    const std::shared_ptr<bool> & query_failed)
  {
    try {
      const auto response =
        future.get();

      const auto state_id =
        response->current_state.id;

      const bool active =
        state_id ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;

      const bool unconfigured =
        state_id ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;

      if (!active) {
        *all_active = false;

        RCLCPP_DEBUG(
          get_logger(),
          "Robot '%s': node '%s' is '%s'",
          robot_id.c_str(),
          node_name.c_str(),
          response->current_state.label.c_str());
      }

      if (!unconfigured) {
        *all_unconfigured = false;
      }
    } catch (
      const std::exception & exception)
    {
      *all_active = false;
      *all_unconfigured = false;
      *query_failed = true;

      RCLCPP_WARN(
        get_logger(),
        "Failed to query state of '%s/%s': %s",
        robot_id.c_str(),
        node_name.c_str(),
        exception.what());
    }

    if (*remaining > 0) {
      --(*remaining);
    }

    if (*remaining != 0) {
      return;
    }

    finish_poll(
      robot_id,
      *all_active,
      *all_unconfigured,
      *query_failed);
  }

  void finish_poll(
    const std::string & robot_id,
    bool all_active,
    bool all_unconfigured,
    bool query_failed)
  {
    auto * robot =
      find_robot(robot_id);

    if (robot == nullptr) {
      return;
    }

    if (query_failed) {
      robot->phase =
        RobotPhase::kIdle;
      return;
    }

    if (all_active) {
      const bool was_ready =
        robot->phase == RobotPhase::kReady;

      robot->phase =
        RobotPhase::kReady;

      if (!was_ready) {
        RCLCPP_INFO(
          get_logger(),
          "All Nav2 nodes are active for robot '%s'",
          robot_id.c_str());
      }

      return;
    }

    if (all_unconfigured) {
      request_nav2_startup(robot_id);
      return;
    }

    RCLCPP_WARN(
      get_logger(),
      "Robot '%s' has mixed Nav2 lifecycle states",
      robot_id.c_str());

    request_nav2_reset(robot_id);
  }

  void request_nav2_startup(
    const std::string & robot_id)
  {
    auto * robot =
      find_robot(robot_id);

    if (robot == nullptr) {
      return;
    }

    const double elapsed_seconds =
      robot->last_startup_attempt.nanoseconds() > 0
      ? (now() - robot->last_startup_attempt).seconds()
      : startup_retry_period_seconds_;

    if (elapsed_seconds < startup_retry_period_seconds_) {
      robot->phase =
        RobotPhase::kIdle;
      return;
    }

    const auto client_iterator =
      lifecycle_clients_.find(robot_id);

    if (
      client_iterator ==
      lifecycle_clients_.end() ||
      !client_iterator->second->service_is_ready())
    {
      robot->phase =
        RobotPhase::kIdle;
      return;
    }

    robot->phase =
      RobotPhase::kStartupPending;

    robot->last_startup_attempt =
      now();

    auto request =
      std::make_shared<
        ManageLifecycleNodes::Request>();

    request->command =
      ManageLifecycleNodes::Request::STARTUP;

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
        auto * robot =
          find_robot(robot_id);

        if (robot == nullptr) {
          return;
        }

        try {
          const auto response =
            future.get();

          /*
           * success only means the lifecycle manager accepted the
           * command, not that every managed node reached ACTIVE.
           * The next poll is the source of truth for that.
           */
          RCLCPP_INFO(
            get_logger(),
            "Nav2 startup command for robot '%s' returned success=%s; "
            "verifying lifecycle states",
            robot_id.c_str(),
            response->success ? "true" : "false");
        } catch (
          const std::exception & exception)
        {
          RCLCPP_ERROR(
            get_logger(),
            "Nav2 startup request failed for robot '%s': %s",
            robot_id.c_str(),
            exception.what());
        }

        robot->phase =
          RobotPhase::kIdle;
      });
  }

  void request_nav2_reset(
    const std::string & robot_id)
  {
    auto * robot =
      find_robot(robot_id);

    if (robot == nullptr) {
      return;
    }

    const double elapsed_seconds =
      robot->last_reset_attempt.nanoseconds() > 0
      ? (now() - robot->last_reset_attempt).seconds()
      : reset_retry_period_seconds_;

    if (elapsed_seconds < reset_retry_period_seconds_) {
      robot->phase =
        RobotPhase::kIdle;
      return;
    }

    const auto client_iterator =
      lifecycle_clients_.find(robot_id);

    if (
      client_iterator ==
      lifecycle_clients_.end() ||
      !client_iterator->second->service_is_ready())
    {
      robot->phase =
        RobotPhase::kIdle;
      return;
    }

    robot->phase =
      RobotPhase::kResetPending;

    robot->last_reset_attempt =
      now();

    auto request =
      std::make_shared<
        ManageLifecycleNodes::Request>();

    request->command =
      ManageLifecycleNodes::Request::RESET;

    RCLCPP_WARN(
      get_logger(),
      "Requesting Nav2 reset for robot '%s' "
      "because lifecycle states are mixed",
      robot_id.c_str());

    client_iterator->second->async_send_request(
      request,
      [this, robot_id](
        rclcpp::Client<
          ManageLifecycleNodes>::SharedFuture future)
      {
        auto * robot =
          find_robot(robot_id);

        if (robot == nullptr) {
          return;
        }

        try {
          const auto response =
            future.get();

          if (!response->success) {
            RCLCPP_WARN(
              get_logger(),
              "Nav2 reset was rejected for robot '%s'",
              robot_id.c_str());
          }
        } catch (
          const std::exception & exception)
        {
          RCLCPP_ERROR(
            get_logger(),
            "Nav2 reset request failed for robot '%s': %s",
            robot_id.c_str(),
            exception.what());
        }

        robot->phase =
          RobotPhase::kIdle;
      });
  }

  void check_readiness()
  {
    const rclcpp::Time current_ros_time =
      now();

    if (current_ros_time.nanoseconds() <= 0) {
      log_waiting_once(
        "clock",
        "Waiting for simulation clock");

      publish_ready(false);
      return;
    }

    if (!clock_received_) {
      clock_received_ = true;

      RCLCPP_INFO(
        get_logger(),
        "Simulation clock is active: %.3f seconds",
        current_ros_time.seconds());
    }

    bool all_ready = true;

    for (const auto & robot_id : robot_ids_) {
      auto * robot =
        find_robot(robot_id);

      if (robot == nullptr) {
        all_ready = false;
        continue;
      }

      update_input_readiness(
        robot_id,
        *robot);

      const bool inputs_ready =
        robot->map_received &&
        robot->tf_available &&
        robot->lifecycle_service_available;

      if (!inputs_ready) {
        robot->phase =
          RobotPhase::kWaitingForInputs;

        all_ready = false;

        RCLCPP_INFO(
          get_logger(),
          "Robot '%s' readiness: "
          "map=%s tf=%s lifecycle=%s",
          robot_id.c_str(),
          robot->map_received
          ? "ready"
          : "waiting",
          robot->tf_available
          ? "ready"
          : "waiting",
          robot->lifecycle_service_available
          ? "ready"
          : "waiting");

        continue;
      }

      if (robot->phase == RobotPhase::kWaitingForInputs) {
        robot->phase =
          RobotPhase::kIdle;
      }

      if (robot->phase == RobotPhase::kIdle) {
        poll_lifecycle_states(robot_id);
      }

      if (robot->phase != RobotPhase::kReady) {
        all_ready = false;
      }
    }

    publish_ready(all_ready);

    if (
      all_ready &&
      !ready_logged_)
    {
      ready_logged_ = true;

      RCLCPP_INFO(
        get_logger(),
        "System is ready: all Nav2 nodes are active");
    }
  }

  void publish_ready(bool ready)
  {
    if (!ready) {
      ready_logged_ = false;
    }

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
    if (
      waiting_log_keys_.count(key) > 0)
    {
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
  double reset_retry_period_seconds_;
  double tf_timeout_seconds_;

  bool clock_received_{false};

  bool has_published_ready_{false};
  bool last_published_ready_{false};
  bool ready_logged_{false};

  std::unordered_map<
    std::string,
    RobotSupervisor>
    robots_;

  std::unordered_map<
    std::string,
    rclcpp::Client<
      ManageLifecycleNodes>::SharedPtr>
    lifecycle_clients_;

  std::unordered_map<
    std::string,
    std::unordered_map<
      std::string,
      rclcpp::Client<GetState>::SharedPtr>>
    lifecycle_state_clients_;

  std::vector<
    rclcpp::Subscription<
      nav_msgs::msg::OccupancyGrid>::SharedPtr>
    map_subscriptions_;

  rclcpp::Publisher<
    std_msgs::msg::Bool>::SharedPtr
    ready_publisher_;

  rclcpp::TimerBase::SharedPtr
    readiness_timer_;

  rclcpp::QoS ready_qos_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::unordered_set<std::string>
    waiting_log_keys_;
};

int main(
  int argc,
  char * argv[])
{
  rclcpp::init(argc, argv);

  try {
    auto node =
      std::make_shared<
      SystemReadinessSupervisor>();

    rclcpp::spin(node);
  } catch (
    const std::exception & exception)
  {
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
