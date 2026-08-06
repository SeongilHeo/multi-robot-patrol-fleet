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
      robot_readiness_.emplace(
        robot_id,
        RobotReadiness{});

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
  struct RobotReadiness
  {
    bool map_received{false};
    bool tf_available{false};
    bool lifecycle_service_available{false};

    bool state_check_in_progress{false};

    bool all_nav2_nodes_active{false};
    bool all_nav2_nodes_unconfigured{false};
    bool nav2_nodes_mixed_state{false};

    bool startup_requested{false};
    bool startup_in_progress{false};
    bool startup_succeeded{false};

    bool reset_in_progress{false};

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

  void map_callback(
    const std::string & robot_id,
    const nav_msgs::msg::OccupancyGrid & message)
  {
    auto iterator =
      robot_readiness_.find(robot_id);

    if (iterator == robot_readiness_.end()) {
      return;
    }

    auto & readiness =
      iterator->second;

    const bool valid_map =
      message.info.width > 0 &&
      message.info.height > 0 &&
      message.info.resolution > 0.0;

    const bool was_received =
      readiness.map_received;

    readiness.map_received =
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

  bool update_tf_readiness(
    const std::string & robot_id)
  {
    auto readiness_iterator =
      robot_readiness_.find(robot_id);

    if (
      readiness_iterator ==
      robot_readiness_.end())
    {
      return false;
    }

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

      readiness_iterator->second.tf_available =
        available;

      return available;
    } catch (
      const tf2::TransformException & exception)
    {
      readiness_iterator->second.tf_available =
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
    auto readiness_iterator =
      robot_readiness_.find(robot_id);

    const auto client_iterator =
      lifecycle_clients_.find(robot_id);

    if (
      readiness_iterator ==
      robot_readiness_.end() ||
      client_iterator ==
      lifecycle_clients_.end())
    {
      return false;
    }

    const bool available =
      client_iterator->second->service_is_ready();

    readiness_iterator->second
      .lifecycle_service_available =
      available;

    return available;
  }

  bool robot_inputs_ready(
    const std::string & robot_id)
  {
    auto iterator =
      robot_readiness_.find(robot_id);

    if (iterator == robot_readiness_.end()) {
      return false;
    }

    update_tf_readiness(robot_id);
    update_lifecycle_service_readiness(robot_id);

    const auto & readiness =
      iterator->second;

    return
      readiness.map_received &&
      readiness.tf_available &&
      readiness.lifecycle_service_available;
  }

  void clear_lifecycle_state_summary(
    RobotReadiness & readiness)
  {
    readiness.all_nav2_nodes_active = false;
    readiness.all_nav2_nodes_unconfigured = false;
    readiness.nav2_nodes_mixed_state = false;
  }

  bool should_retry_startup(
    const RobotReadiness & readiness) const
  {
    if (
      readiness.startup_in_progress ||
      readiness.reset_in_progress ||
      readiness.state_check_in_progress)
    {
      return false;
    }

    if (readiness.startup_succeeded) {
      return false;
    }

    if (!readiness.startup_requested) {
      return true;
    }

    if (
      readiness.last_startup_attempt.nanoseconds() <= 0)
    {
      return true;
    }

    const double elapsed_seconds =
      (now() - readiness.last_startup_attempt)
      .seconds();

    return
      elapsed_seconds >=
      startup_retry_period_seconds_;
  }

  bool should_retry_reset(
    const RobotReadiness & readiness) const
  {
    if (
      readiness.reset_in_progress ||
      readiness.startup_in_progress ||
      readiness.state_check_in_progress)
    {
      return false;
    }

    if (
      readiness.last_reset_attempt.nanoseconds() <= 0)
    {
      return true;
    }

    const double elapsed_seconds =
      (now() - readiness.last_reset_attempt)
      .seconds();

    return
      elapsed_seconds >=
      reset_retry_period_seconds_;
  }

  void check_all_nav2_nodes_active(
    const std::string & robot_id)
  {
    auto readiness_iterator =
      robot_readiness_.find(robot_id);

    if (
      readiness_iterator ==
      robot_readiness_.end())
    {
      return;
    }

    auto & readiness =
      readiness_iterator->second;

    if (
      readiness.state_check_in_progress ||
      readiness.startup_in_progress ||
      readiness.reset_in_progress)
    {
      return;
    }

    auto robot_clients_iterator =
      lifecycle_state_clients_.find(robot_id);

    if (
      robot_clients_iterator ==
      lifecycle_state_clients_.end())
    {
      clear_lifecycle_state_summary(
        readiness);
      return;
    }

    for (const auto & node_name : nav2_node_names_) {
      const auto client_iterator =
        robot_clients_iterator->second.find(
        node_name);

      if (
        client_iterator ==
        robot_clients_iterator->second.end() ||
        !client_iterator->second->service_is_ready())
      {
        clear_lifecycle_state_summary(
          readiness);
        return;
      }
    }

    readiness.state_check_in_progress = true;

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
        lifecycle_state_clients_[robot_id][node_name];

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
          try {
            const auto response =
              future.get();

            const auto state_id =
              response->current_state.id;

            const bool active =
              state_id ==
              lifecycle_msgs::msg::State::
              PRIMARY_STATE_ACTIVE;

            const bool unconfigured =
              state_id ==
              lifecycle_msgs::msg::State::
              PRIMARY_STATE_UNCONFIGURED;

            if (!active) {
              *all_active = false;
            }

            if (!unconfigured) {
              *all_unconfigured = false;
            }

            if (!active) {
              RCLCPP_DEBUG(
                get_logger(),
                "Robot '%s': node '%s' is '%s'",
                robot_id.c_str(),
                node_name.c_str(),
                response->current_state.label.c_str());
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

          auto robot_iterator =
            robot_readiness_.find(robot_id);

          if (
            robot_iterator ==
            robot_readiness_.end())
          {
            return;
          }

          auto & robot =
            robot_iterator->second;

          const bool was_all_active =
            robot.all_nav2_nodes_active;

          const bool was_mixed =
            robot.nav2_nodes_mixed_state;

          robot.state_check_in_progress = false;

          if (*query_failed) {
            clear_lifecycle_state_summary(
              robot);
            robot.startup_succeeded = false;
            return;
          }

          robot.all_nav2_nodes_active =
            *all_active;

          robot.all_nav2_nodes_unconfigured =
            *all_unconfigured;

          robot.nav2_nodes_mixed_state =
            !*all_active &&
            !*all_unconfigured;

          if (*all_active) {
            robot.startup_requested = true;
            robot.startup_succeeded = true;

            if (!was_all_active) {
              RCLCPP_INFO(
                get_logger(),
                "All Nav2 nodes are active for robot '%s'",
                robot_id.c_str());
            }

            return;
          }

          robot.startup_succeeded = false;

          if (
            robot.nav2_nodes_mixed_state &&
            !was_mixed)
          {
            RCLCPP_WARN(
              get_logger(),
              "Robot '%s' has mixed Nav2 lifecycle states",
              robot_id.c_str());
          }

          if (*all_unconfigured) {
            RCLCPP_DEBUG(
              get_logger(),
              "All Nav2 nodes are unconfigured for robot '%s'",
              robot_id.c_str());
          }
        });
    }
  }

  void request_nav2_startup(
    const std::string & robot_id)
  {
    const auto client_iterator =
      lifecycle_clients_.find(robot_id);

    auto readiness_iterator =
      robot_readiness_.find(robot_id);

    if (
      client_iterator ==
      lifecycle_clients_.end() ||
      readiness_iterator ==
      robot_readiness_.end())
    {
      return;
    }

    auto & readiness =
      readiness_iterator->second;

    if (!should_retry_startup(readiness)) {
      return;
    }

    if (
      !client_iterator->second->service_is_ready())
    {
      return;
    }

    auto request =
      std::make_shared<
        ManageLifecycleNodes::Request>();

    request->command =
      ManageLifecycleNodes::Request::STARTUP;

    readiness.startup_requested = true;
    readiness.startup_in_progress = true;
    readiness.startup_succeeded = false;
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
        auto robot_iterator =
          robot_readiness_.find(robot_id);

        if (
          robot_iterator ==
          robot_readiness_.end())
        {
          return;
        }

        auto & robot =
          robot_iterator->second;

        robot.startup_in_progress = false;

        try {
          const auto response =
            future.get();

          if (response->success) {
            /*
             * STARTUP 응답 성공만으로 READY를 확정하지 않는다.
             * 다음 lifecycle 상태 검사에서 6개 노드가 모두
             * active인지 확인한 후 startup_succeeded가 true가 된다.
             */
            robot.startup_succeeded = false;

            RCLCPP_INFO(
              get_logger(),
              "Nav2 startup command completed for robot '%s'; "
              "verifying lifecycle states",
              robot_id.c_str());
          } else {
            robot.startup_succeeded = false;

            RCLCPP_WARN(
              get_logger(),
              "Nav2 startup was rejected for robot '%s'",
              robot_id.c_str());
          }
        } catch (
          const std::exception & exception)
        {
          robot.startup_succeeded = false;

          RCLCPP_ERROR(
            get_logger(),
            "Nav2 startup request failed for robot '%s': %s",
            robot_id.c_str(),
            exception.what());
        }
      });
  }

  void request_nav2_reset(
    const std::string & robot_id)
  {
    const auto client_iterator =
      lifecycle_clients_.find(robot_id);

    auto readiness_iterator =
      robot_readiness_.find(robot_id);

    if (
      client_iterator ==
      lifecycle_clients_.end() ||
      readiness_iterator ==
      robot_readiness_.end())
    {
      return;
    }

    auto & readiness =
      readiness_iterator->second;

    if (!should_retry_reset(readiness)) {
      return;
    }

    if (
      !client_iterator->second->service_is_ready())
    {
      return;
    }

    auto request =
      std::make_shared<
        ManageLifecycleNodes::Request>();

    request->command =
      ManageLifecycleNodes::Request::RESET;

    readiness.reset_in_progress = true;
    readiness.startup_succeeded = false;
    readiness.last_reset_attempt = now();

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
        auto robot_iterator =
          robot_readiness_.find(robot_id);

        if (
          robot_iterator ==
          robot_readiness_.end())
        {
          return;
        }

        auto & robot =
          robot_iterator->second;

        robot.reset_in_progress = false;

        try {
          const auto response =
            future.get();

          if (response->success) {
            robot.startup_requested = false;
            robot.startup_in_progress = false;
            robot.startup_succeeded = false;

            clear_lifecycle_state_summary(
              robot);

            RCLCPP_INFO(
              get_logger(),
              "Nav2 reset succeeded for robot '%s'; "
              "waiting for all nodes to become unconfigured",
              robot_id.c_str());
          } else {
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

    bool all_inputs_ready = true;

    for (const auto & robot_id : robot_ids_) {
      const bool inputs_ready =
        robot_inputs_ready(robot_id);

      if (inputs_ready) {
        continue;
      }

      all_inputs_ready = false;

      const auto iterator =
        robot_readiness_.find(robot_id);

      if (
        iterator ==
        robot_readiness_.end())
      {
        continue;
      }

      const auto & readiness =
        iterator->second;

      RCLCPP_INFO(
        get_logger(),
        "Robot '%s' readiness: "
        "map=%s tf=%s lifecycle=%s",
        robot_id.c_str(),
        readiness.map_received
        ? "ready"
        : "waiting",
        readiness.tf_available
        ? "ready"
        : "waiting",
        readiness.lifecycle_service_available
        ? "ready"
        : "waiting");
    }

    if (!all_inputs_ready) {
      publish_ready(false);
      return;
    }

    /*
     * 먼저 모든 로봇의 lifecycle 상태를 비동기로 요청한다.
     */
    for (const auto & robot_id : robot_ids_) {
      check_all_nav2_nodes_active(
        robot_id);
    }

    bool all_nav2_active = true;

    for (const auto & robot_id : robot_ids_) {
      auto iterator =
        robot_readiness_.find(robot_id);

      if (
        iterator ==
        robot_readiness_.end())
      {
        all_nav2_active = false;
        continue;
      }

      auto & readiness =
        iterator->second;

      if (readiness.all_nav2_nodes_active) {
        continue;
      }

      all_nav2_active = false;

      /*
       * 현재 상태 조회나 lifecycle 전환이 진행 중이면
       * 이번 주기에서는 추가 요청을 보내지 않는다.
       */
      if (
        readiness.state_check_in_progress ||
        readiness.startup_in_progress ||
        readiness.reset_in_progress)
      {
        continue;
      }

      /*
       * 일부 노드는 active이고 일부는 inactive/unconfigured라면
       * STARTUP을 다시 보내면 transition 오류가 발생한다.
       * 먼저 RESET으로 전체 노드를 unconfigured 상태로 정리한다.
       */
      if (readiness.nav2_nodes_mixed_state) {
        if (should_retry_reset(readiness)) {
          request_nav2_reset(robot_id);
        }

        continue;
      }

      /*
       * 모든 Nav2 노드가 unconfigured인 경우에만
       * 정상적인 STARTUP 시퀀스를 요청한다.
       */
      if (
        readiness.all_nav2_nodes_unconfigured &&
        should_retry_startup(readiness))
      {
        request_nav2_startup(robot_id);
      }
    }

    publish_ready(all_nav2_active);

    if (
      all_nav2_active &&
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
    RobotReadiness>
    robot_readiness_;

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