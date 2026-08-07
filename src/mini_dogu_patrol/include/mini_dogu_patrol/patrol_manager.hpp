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

#include "mini_dogu_interfaces/srv/start_mission.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace mini_dogu_patrol
{

class PatrolManager : public rclcpp::Node
{
public:
  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
  using GoalHandleFollowWaypoints =
    rclcpp_action::ClientGoalHandle<FollowWaypoints>;
  using StartMission = mini_dogu_interfaces::srv::StartMission;

  explicit PatrolManager(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void start_patrol();

  bool execute_mission(
    uint8_t mission_type,
    const geometry_msgs::msg::PoseStamped & target_pose,
    bool has_target,
    std::string & error);

  void send_goal(
    std::vector<geometry_msgs::msg::PoseStamped> poses,
    uint32_t number_of_loops,
    uint32_t goal_index);

  std::vector<geometry_msgs::msg::PoseStamped>
  build_waypoints() const;

  geometry_msgs::msg::PoseStamped
  pose_from_xyz_yaw(double x, double y, double yaw) const;

  geometry_msgs::msg::PoseStamped
  finalize_target_pose(
    const geometry_msgs::msg::PoseStamped & requested) const;

  void goal_response_callback(
    const GoalHandleFollowWaypoints::SharedPtr & goal_handle);

  void feedback_callback(
    GoalHandleFollowWaypoints::SharedPtr,
    const std::shared_ptr<const FollowWaypoints::Feedback> feedback);

  void result_callback(
    const GoalHandleFollowWaypoints::WrappedResult & result);

  void handle_start(
    const std::shared_ptr<StartMission::Request> request,
    std::shared_ptr<StartMission::Response> response);

  void handle_stop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void publish_status(const std::string & status);

  std::string robot_namespace_;
  std::string frame_id_;
  std::string action_name_;

  std::vector<double> waypoint_values_;
  std::vector<double> home_pose_values_;
  std::vector<double> charge_dock_pose_values_;

  int64_t number_of_loops_;
  int64_t goal_index_;
  double server_wait_timeout_;
  bool autostart_;

  rclcpp_action::Client<FollowWaypoints>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr start_timer_;

  bool patrol_active_{false};

  GoalHandleFollowWaypoints::SharedPtr active_goal_handle_;

  rclcpp::Service<StartMission>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
};

}  // namespace mini_dogu_patrol

#endif  // MINI_DOGU_PATROL__PATROL_MANAGER_HPP_
