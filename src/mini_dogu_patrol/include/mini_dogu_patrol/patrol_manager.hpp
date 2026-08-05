#ifndef MINI_DOGU_PATROL__PATROL_MANAGER_HPP_
#define MINI_DOGU_PATROL__PATROL_MANAGER_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace mini_dogu_patrol
{

class PatrolManager : public rclcpp::Node
{
public:
  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
  using GoalHandleFollowWaypoints =
    rclcpp_action::ClientGoalHandle<FollowWaypoints>;

  explicit PatrolManager(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void start_patrol();

  std::vector<geometry_msgs::msg::PoseStamped>
  build_waypoints() const;

  void goal_response_callback(
    const GoalHandleFollowWaypoints::SharedPtr & goal_handle);

  void feedback_callback(
    GoalHandleFollowWaypoints::SharedPtr,
    const std::shared_ptr<const FollowWaypoints::Feedback> feedback);

  void result_callback(
    const GoalHandleFollowWaypoints::WrappedResult & result);

  std::string robot_namespace_;
  std::string frame_id_;
  std::string action_name_;

  std::vector<double> waypoint_values_;

  int64_t number_of_loops_;
  int64_t goal_index_;
  double server_wait_timeout_;
  bool autostart_;

  rclcpp_action::Client<FollowWaypoints>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr start_timer_;
};

}  // namespace mini_dogu_patrol

#endif  // MINI_DOGU_PATROL__PATROL_MANAGER_HPP_