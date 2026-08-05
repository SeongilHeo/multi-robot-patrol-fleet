#include "mini_dogu_patrol/patrol_manager.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <utility>

using namespace std::chrono_literals;

namespace mini_dogu_patrol
{

PatrolManager::PatrolManager(const rclcpp::NodeOptions & options)
: Node("patrol_manager", options)
{
  robot_namespace_ = this->declare_parameter<std::string>(
    "robot_namespace",
    "robot1");

  frame_id_ = this->declare_parameter<std::string>(
    "frame_id",
    robot_namespace_ + "/map");

  number_of_loops_ = this->declare_parameter<int64_t>(
    "number_of_loops",
    0);

  goal_index_ = this->declare_parameter<int64_t>(
    "goal_index",
    0);

  server_wait_timeout_ = this->declare_parameter<double>(
    "server_wait_timeout",
    10.0);

  autostart_ = this->declare_parameter<bool>(
    "autostart",
    true);

  waypoint_values_ = this->declare_parameter<std::vector<double>>(
    "waypoints",
    std::vector<double>{
      0.5, 0.0, 0.0,
      0.5, 0.5, 1.57079632679,
      0.0, 0.5, 3.14159265359,
      0.0, 0.0, -1.57079632679
    });

  action_name_ =
    "/" + robot_namespace_ + "/follow_waypoints";

  action_client_ =
    rclcpp_action::create_client<FollowWaypoints>(
    this,
    action_name_);

  const std::string patrol_prefix =
    "/" + robot_namespace_ + "/patrol";

  rclcpp::QoS status_qos(1);
  status_qos.reliable();
  status_qos.transient_local();

  status_publisher_ =
    this->create_publisher<std_msgs::msg::String>(
      patrol_prefix + "/status",
      status_qos);

  start_service_ =
    this->create_service<std_srvs::srv::Trigger>(
    patrol_prefix + "/start",
    std::bind(
      &PatrolManager::handle_start,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  stop_service_ =
    this->create_service<std_srvs::srv::Trigger>(
    patrol_prefix + "/stop",
    std::bind(
      &PatrolManager::handle_stop,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  publish_status("idle");

  RCLCPP_INFO(
    this->get_logger(),
    "Patrol manager configured for robot '%s'",
    robot_namespace_.c_str());

  RCLCPP_INFO(
    this->get_logger(),
    "FollowWaypoints action: %s",
    action_name_.c_str());

  if (autostart_) {
    // Constructor가 끝난 뒤 action 요청을 시작한다.
    start_timer_ = this->create_wall_timer(
      1s,
      [this]() {
        start_timer_->cancel();
        start_patrol();
      });
  } else {
    RCLCPP_INFO(
      this->get_logger(),
      "Autostart disabled; no patrol goal will be sent");
  }
}

std::vector<geometry_msgs::msg::PoseStamped>
PatrolManager::build_waypoints() const
{
  if (waypoint_values_.empty()) {
    throw std::runtime_error(
            "The waypoint list is empty");
  }

  if (waypoint_values_.size() % 3 != 0) {
    throw std::runtime_error(
            "Waypoints must be flattened as [x, y, yaw, ...]");
  }

  std::vector<geometry_msgs::msg::PoseStamped> poses;
  poses.reserve(waypoint_values_.size() / 3);

  const auto stamp = this->now();

  for (std::size_t i = 0; i < waypoint_values_.size(); i += 3) {
    const double x = waypoint_values_.at(i);
    const double y = waypoint_values_.at(i + 1);
    const double yaw = waypoint_values_.at(i + 2);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = frame_id_;

    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;

    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = std::sin(yaw * 0.5);
    pose.pose.orientation.w = std::cos(yaw * 0.5);

    poses.push_back(pose);
  }

  return poses;
}

void PatrolManager::handle_start(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (patrol_active_) {
    response->success = false;
    response->message = "Patrol mission is already active";
    return;
  }

  response->success = true;
  response->message = "Patrol mission start requested";

  start_patrol();
}

void PatrolManager::handle_stop(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!patrol_active_ || !active_goal_handle_) {
    response->success = false;
    response->message = "No active patrol mission";
    return;
  }

  publish_status("canceling");

  action_client_->async_cancel_goal(active_goal_handle_);

  response->success = true;
  response->message = "Patrol cancellation requested";
}

void PatrolManager::publish_status(const std::string & status)
{
  std_msgs::msg::String message;
  message.data = status;

  status_publisher_->publish(message);

  RCLCPP_INFO(
    this->get_logger(),
    "Patrol status: %s",
    status.c_str());
}

void PatrolManager::start_patrol()
{
  if (patrol_active_) {
    RCLCPP_WARN(
      this->get_logger(),
      "Patrol mission is already active");
    return;
  }

  publish_status("waiting_for_nav2");

  const auto wait_duration =
    std::chrono::duration<double>(server_wait_timeout_);

  RCLCPP_INFO(
    this->get_logger(),
    "Waiting for action server '%s'...",
    action_name_.c_str());

  if (!action_client_->wait_for_action_server(wait_duration)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "FollowWaypoints action server was not available after %.1f seconds",
      server_wait_timeout_);
    return;
  }

  publish_status("sending_goal");

  FollowWaypoints::Goal goal;

  try {
    goal.poses = build_waypoints();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Invalid waypoint configuration: %s",
      error.what());
    return;
  }

  if (number_of_loops_ < 0 || goal_index_ < 0) {
    RCLCPP_ERROR(
      this->get_logger(),
      "number_of_loops and goal_index cannot be negative");
    return;
  }

  if (
    static_cast<std::size_t>(goal_index_) >= goal.poses.size())
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "goal_index %ld exceeds waypoint count %zu",
      goal_index_,
      goal.poses.size());
    return;
  }

  goal.number_of_loops =
    static_cast<std::uint32_t>(number_of_loops_);

  goal.goal_index =
    static_cast<std::uint32_t>(goal_index_);

  RCLCPP_INFO(
    this->get_logger(),
    "Sending patrol mission: %zu waypoints, loops=%ld, start_index=%ld",
    goal.poses.size(),
    number_of_loops_,
    goal_index_);

  rclcpp_action::Client<FollowWaypoints>::SendGoalOptions options;

  options.goal_response_callback =
    std::bind(
    &PatrolManager::goal_response_callback,
    this,
    std::placeholders::_1);

  options.feedback_callback =
    std::bind(
    &PatrolManager::feedback_callback,
    this,
    std::placeholders::_1,
    std::placeholders::_2);

  options.result_callback =
    std::bind(
    &PatrolManager::result_callback,
    this,
    std::placeholders::_1);

  action_client_->async_send_goal(goal, options);
}

void PatrolManager::goal_response_callback(
  const GoalHandleFollowWaypoints::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    patrol_active_ = false;

    publish_status("rejected");

    RCLCPP_ERROR(
      this->get_logger(),
      "Patrol goal was rejected");

    return;
  }

  active_goal_handle_ = goal_handle;
  patrol_active_ = true;

  publish_status("active");

  RCLCPP_INFO(
    this->get_logger(),
    "Patrol goal accepted");
}

void PatrolManager::feedback_callback(
  GoalHandleFollowWaypoints::SharedPtr,
  const std::shared_ptr<const FollowWaypoints::Feedback> feedback)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Navigating to waypoint index: %u",
    feedback->current_waypoint);
}

void PatrolManager::result_callback(
  const GoalHandleFollowWaypoints::WrappedResult & result)
{
  patrol_active_ = false;
  active_goal_handle_.reset();

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      publish_status("completed");

      RCLCPP_INFO(
        this->get_logger(),
        "Patrol mission completed");
      break;

    case rclcpp_action::ResultCode::ABORTED:
      publish_status("aborted");

      RCLCPP_ERROR(
        this->get_logger(),
        "Patrol mission was aborted");
      break;

    case rclcpp_action::ResultCode::CANCELED:
      publish_status("canceled");

      RCLCPP_WARN(
        this->get_logger(),
        "Patrol mission was canceled");
      break;

    default:
      publish_status("error");

      RCLCPP_ERROR(
        this->get_logger(),
        "Patrol mission returned an unknown result");
      break;
  }
}

}  // namespace mini_dogu_patrol


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<mini_dogu_patrol::PatrolManager>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}