#include <algorithm>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "mini_dogu_interfaces/msg/fleet_state.hpp"
#include "mini_dogu_interfaces/msg/robot_heartbeat.hpp"
#include "mini_dogu_interfaces/srv/assign_mission.hpp"

class MissionSchedulerNode : public rclcpp::Node
{
public:
  using FleetState =
    mini_dogu_interfaces::msg::FleetState;

  using RobotHeartbeat =
    mini_dogu_interfaces::msg::RobotHeartbeat;

  using AssignMission =
    mini_dogu_interfaces::srv::AssignMission;

  using Trigger =
    std_srvs::srv::Trigger;

  MissionSchedulerNode()
  : Node("mission_scheduler")
  {
    fleet_state_topic_ =
      declare_parameter<std::string>(
      "fleet_state_topic",
      "/fleet/state");

    assign_mission_service_name_ =
      declare_parameter<std::string>(
      "assign_mission_service",
      "/fleet/assign_mission");

    minimum_battery_percentage_ =
      declare_parameter<double>(
      "minimum_battery_percentage",
      20.0);

    fleet_state_subscription_ =
      create_subscription<FleetState>(
      fleet_state_topic_,
      rclcpp::QoS(10).reliable(),
      std::bind(
        &MissionSchedulerNode::fleet_state_callback,
        this,
        std::placeholders::_1));

    assign_mission_client_ =
      create_client<AssignMission>(
      assign_mission_service_name_);

    dispatch_patrol_service_ =
      create_service<Trigger>(
      "/scheduler/dispatch_patrol",
      std::bind(
        &MissionSchedulerNode::handle_dispatch_patrol,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "Mission scheduler started");

    RCLCPP_INFO(
      get_logger(),
      "Fleet state topic: %s",
      fleet_state_topic_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "Assign mission service: %s",
      assign_mission_service_name_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "Minimum battery requirement: %.1f%%",
      minimum_battery_percentage_);
  }

private:
  struct CandidateRobot
  {
    std::string robot_id;
    float battery_percentage;
  };

  void fleet_state_callback(
    const FleetState::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(fleet_state_mutex_);
    latest_fleet_state_ = *message;
    received_fleet_state_ = true;
  }

  std::optional<CandidateRobot>
  select_patrol_robot() const
  {
    std::lock_guard<std::mutex> lock(fleet_state_mutex_);

    if (!received_fleet_state_) {
      return std::nullopt;
    }

    std::vector<CandidateRobot> candidates;

    for (const auto & robot : latest_fleet_state_.robots) {
      const bool is_idle =
        robot.state == RobotHeartbeat::STATE_IDLE;

      const bool battery_sufficient =
        robot.battery_percentage >=
        minimum_battery_percentage_;

      if (
        robot.online &&
        is_idle &&
        battery_sufficient)
      {
        candidates.push_back(
          CandidateRobot{
            robot.robot_id,
            robot.battery_percentage
          });
      }
    }

    if (candidates.empty()) {
      return std::nullopt;
    }

    const auto best_robot =
      std::max_element(
      candidates.begin(),
      candidates.end(),
      [](
        const CandidateRobot & left,
        const CandidateRobot & right)
      {
        return left.battery_percentage <
               right.battery_percentage;
      });

    return *best_robot;
  }

  void handle_dispatch_patrol(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    {
      std::lock_guard<std::mutex> lock(request_mutex_);

      if (dispatch_in_progress_) {
        response->success = false;
        response->message =
          "A mission dispatch request is already in progress";
        return;
      }
    }

    if (!assign_mission_client_->service_is_ready()) {
      response->success = false;
      response->message =
        "Fleet assign_mission service is unavailable";
      return;
    }

    const auto selected_robot =
      select_patrol_robot();

    if (!selected_robot.has_value()) {
      response->success = false;
      response->message =
        "No online idle robot satisfies the battery requirement";
      return;
    }

    auto request =
      std::make_shared<AssignMission::Request>();

    request->robot_id =
      selected_robot->robot_id;

    request->mission_type =
      "patrol";

    request->command =
      AssignMission::Request::COMMAND_START;

    {
      std::lock_guard<std::mutex> lock(request_mutex_);
      dispatch_in_progress_ = true;
    }

    RCLCPP_INFO(
      get_logger(),
      "Selected robot '%s' for patrol "
      "(battery %.1f%%)",
      selected_robot->robot_id.c_str(),
      selected_robot->battery_percentage);

    assign_mission_client_->async_send_request(
      request,
      [this, robot_id = selected_robot->robot_id](
        rclcpp::Client<AssignMission>::SharedFuture future)
      {
        try {
          const auto fleet_response = future.get();

          if (fleet_response->success) {
            RCLCPP_INFO(
              get_logger(),
              "Fleet accepted patrol request for robot '%s': %s",
              robot_id.c_str(),
              fleet_response->message.c_str());
          } else {
            RCLCPP_WARN(
              get_logger(),
              "Fleet rejected patrol request for robot '%s': %s",
              robot_id.c_str(),
              fleet_response->message.c_str());
          }
        } catch (const std::exception & exception) {
          RCLCPP_ERROR(
            get_logger(),
            "Mission assignment failed for '%s': %s",
            robot_id.c_str(),
            exception.what());
        }

        std::lock_guard<std::mutex> lock(request_mutex_);
        dispatch_in_progress_ = false;
      });

    response->success = true;
    response->message =
      "Patrol dispatch request accepted for robot '" +
      selected_robot->robot_id +
      "'; final execution status is available on /fleet/state";
  }

  std::string fleet_state_topic_;
  std::string assign_mission_service_name_;
  double minimum_battery_percentage_;

  mutable std::mutex fleet_state_mutex_;
  FleetState latest_fleet_state_;
  bool received_fleet_state_{false};

  std::mutex request_mutex_;
  bool dispatch_in_progress_{false};

  rclcpp::Subscription<FleetState>::SharedPtr
    fleet_state_subscription_;

  rclcpp::Client<AssignMission>::SharedPtr
    assign_mission_client_;

  rclcpp::Service<Trigger>::SharedPtr
    dispatch_patrol_service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(
      std::make_shared<MissionSchedulerNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("mission_scheduler"),
      "Unhandled exception: %s",
      exception.what());

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}