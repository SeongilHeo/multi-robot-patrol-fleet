#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "mini_dogu_interfaces/msg/fleet_state.hpp"
#include "mini_dogu_interfaces/msg/mission_queue_state.hpp"
#include "mini_dogu_interfaces/msg/robot_heartbeat.hpp"
#include "mini_dogu_interfaces/srv/assign_mission.hpp"
#include "mini_dogu_interfaces/srv/queue_mission.hpp"

using namespace std::chrono_literals;

class MissionSchedulerNode : public rclcpp::Node
{
public:
  using FleetState =
    mini_dogu_interfaces::msg::FleetState;

  using MissionQueueState =
    mini_dogu_interfaces::msg::MissionQueueState;

  using RobotHeartbeat =
    mini_dogu_interfaces::msg::RobotHeartbeat;

  using AssignMission =
    mini_dogu_interfaces::srv::AssignMission;

  using QueueMission =
    mini_dogu_interfaces::srv::QueueMission;

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

    reservation_timeout_seconds_ =
      declare_parameter<double>(
      "reservation_timeout_seconds",
      10.0);

    fleet_state_subscription_ =
      create_subscription<FleetState>(
      fleet_state_topic_,
      rclcpp::QoS(10).reliable(),
      std::bind(
        &MissionSchedulerNode::fleet_state_callback,
        this,
        std::placeholders::_1));

    rclcpp::QoS ready_qos(1);
    ready_qos.reliable();
    ready_qos.transient_local();

    system_ready_subscription_ =
      create_subscription<std_msgs::msg::Bool>(
      "/system/ready",
      ready_qos,
      std::bind(
        &MissionSchedulerNode::system_ready_callback,
        this,
        std::placeholders::_1));

    assign_mission_client_ =
      create_client<AssignMission>(
      assign_mission_service_name_);

    queue_mission_service_ =
      create_service<QueueMission>(
      "/scheduler/queue_mission",
      std::bind(
        &MissionSchedulerNode::handle_queue_mission,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    dispatch_patrol_service_ =
      create_service<Trigger>(
      "/scheduler/dispatch_patrol",
      std::bind(
        &MissionSchedulerNode::handle_dispatch_patrol,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    rclcpp::QoS queue_qos(1);
    queue_qos.reliable();
    queue_qos.transient_local();

    queue_state_publisher_ =
      create_publisher<MissionQueueState>(
      "/scheduler/queue_state",
      queue_qos);

    dispatch_timer_ =
      create_wall_timer(
      500ms,
      std::bind(
        &MissionSchedulerNode::process_queue,
        this));

    publish_queue_state();

    RCLCPP_INFO(
      get_logger(),
      "Mission queue scheduler started");

    RCLCPP_INFO(
      get_logger(),
      "Minimum battery requirement: %.1f%%",
      minimum_battery_percentage_);
  }

private:
  struct QueuedMission
  {
    std::string mission_id;
    std::string mission_type;
    int32_t priority{0};
    uint64_t sequence{0};
  };

  struct CandidateRobot
  {
    std::string robot_id;
    float battery_percentage{0.0F};
  };

  void system_ready_callback(
    const std_msgs::msg::Bool::SharedPtr message)
  {
    system_ready_ = message->data;

    RCLCPP_INFO(
      get_logger(),
      "System readiness received: %s",
      system_ready_ ? "READY" : "NOT_READY");
  }

  void fleet_state_callback(
    const FleetState::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);

    latest_fleet_state_ = *message;
    received_fleet_state_ = true;

    update_robot_reservations_locked();
  }

  void update_robot_reservations_locked()
  {
    const rclcpp::Time current_time = now();

    for (const auto & robot : latest_fleet_state_.robots) {
      auto reservation_iterator =
        reservation_times_.find(robot.robot_id);

      if (reservation_iterator ==
          reservation_times_.end())
      {
        continue;
      }

      if (robot.state != RobotHeartbeat::STATE_IDLE) {
        reserved_robots_.erase(robot.robot_id);
        reservation_times_.erase(reservation_iterator);
        continue;
      }

      const double reservation_age =
        (current_time -
        reservation_iterator->second).seconds();

      if (reservation_age >
          reservation_timeout_seconds_)
      {
        RCLCPP_WARN(
          get_logger(),
          "Reservation for robot '%s' expired after %.1f seconds",
          robot.robot_id.c_str(),
          reservation_age);

        reserved_robots_.erase(robot.robot_id);
        reservation_times_.erase(reservation_iterator);
      }
    }
  }

  std::string create_mission_id()
  {
    ++mission_counter_;

    return
      "mission-" +
      std::to_string(mission_counter_);
  }

  void enqueue_mission(
    const std::string & mission_type,
    int32_t priority,
    std::string & mission_id)
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);

    mission_id = create_mission_id();

    mission_queue_.push_back(
      QueuedMission{
        mission_id,
        mission_type,
        priority,
        next_sequence_++
      });

    sort_queue_locked();
  }

  void sort_queue_locked()
  {
    std::stable_sort(
      mission_queue_.begin(),
      mission_queue_.end(),
      [](
        const QueuedMission & left,
        const QueuedMission & right)
      {
        if (left.priority != right.priority) {
          return left.priority > right.priority;
        }

        return left.sequence < right.sequence;
      });
  }

  void handle_queue_mission(
    const std::shared_ptr<QueueMission::Request> request,
    std::shared_ptr<QueueMission::Response> response)
  {
    if (request->mission_type != "patrol") {
      response->accepted = false;
      response->message =
        "Unsupported mission type: " +
        request->mission_type;
      return;
    }

    std::string mission_id;

    enqueue_mission(
      request->mission_type,
      request->priority,
      mission_id);

    publish_queue_state();

    response->accepted = true;
    response->mission_id = mission_id;
    response->message =
      "Mission '" + mission_id +
      "' added to the scheduling queue";

    RCLCPP_INFO(
      get_logger(),
      "Queued mission '%s': type='%s', priority=%d",
      mission_id.c_str(),
      request->mission_type.c_str(),
      request->priority);
  }

  void handle_dispatch_patrol(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    std::string mission_id;

    enqueue_mission(
      "patrol",
      0,
      mission_id);

    publish_queue_state();

    response->success = true;
    response->message =
      "Patrol mission '" + mission_id +
      "' added to the scheduling queue";
  }

  std::optional<CandidateRobot>
  select_robot() const
  {
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (!received_fleet_state_) {
      return std::nullopt;
    }

    std::vector<CandidateRobot> candidates;

    for (const auto & robot :
         latest_fleet_state_.robots)
    {
      const bool idle =
        robot.state ==
        RobotHeartbeat::STATE_IDLE;

      const bool battery_sufficient =
        robot.battery_percentage >=
        minimum_battery_percentage_;

      const bool reserved =
        reserved_robots_.count(
          robot.robot_id) > 0;

      if (
        robot.online &&
        idle &&
        battery_sufficient &&
        !reserved)
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
        return
          left.battery_percentage <
          right.battery_percentage;
      });

    return *best_robot;
  }

  std::optional<QueuedMission>
  get_next_mission() const
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);

    if (mission_queue_.empty()) {
      return std::nullopt;
    }

    return mission_queue_.front();
  }

  void remove_mission(
    const std::string & mission_id)
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);

    mission_queue_.erase(
      std::remove_if(
        mission_queue_.begin(),
        mission_queue_.end(),
        [&mission_id](
          const QueuedMission & mission)
        {
          return mission.mission_id ==
                 mission_id;
        }),
      mission_queue_.end());
  }

  void reserve_robot(
    const std::string & robot_id)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);

    reserved_robots_.insert(robot_id);
    reservation_times_[robot_id] = now();
  }

  void release_robot_reservation(
    const std::string & robot_id)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);

    reserved_robots_.erase(robot_id);
    reservation_times_.erase(robot_id);
  }

  void process_queue()
  {
    if (!system_ready_) {
      return;
    }

    {
      std::lock_guard<std::mutex> lock(request_mutex_);

      if (dispatch_in_progress_) {
        return;
      }
    }

    if (!assign_mission_client_->service_is_ready()) {
      return;
    }

    const auto mission = get_next_mission();

    if (!mission.has_value()) {
      return;
    }

    const auto robot = select_robot();

    if (!robot.has_value()) {
      return;
    }

    auto request =
      std::make_shared<AssignMission::Request>();

    request->robot_id = robot->robot_id;
    request->mission_type = mission->mission_type;
    request->command =
      AssignMission::Request::COMMAND_START;

    {
      std::lock_guard<std::mutex> lock(request_mutex_);
      dispatch_in_progress_ = true;
    }

    reserve_robot(robot->robot_id);

    RCLCPP_INFO(
      get_logger(),
      "Dispatching mission '%s' to robot '%s' "
      "(priority=%d, battery=%.1f%%)",
      mission->mission_id.c_str(),
      robot->robot_id.c_str(),
      mission->priority,
      robot->battery_percentage);

    assign_mission_client_->async_send_request(
      request,
      [
        this,
        mission_id = mission->mission_id,
        robot_id = robot->robot_id
      ](
        rclcpp::Client<
          AssignMission>::SharedFuture future)
      {
        bool accepted = false;

        try {
          const auto response = future.get();
          accepted = response->success;

          if (accepted) {
            RCLCPP_INFO(
              get_logger(),
              "Fleet accepted mission '%s' for robot '%s': %s",
              mission_id.c_str(),
              robot_id.c_str(),
              response->message.c_str());

            remove_mission(mission_id);
          } else {
            RCLCPP_WARN(
              get_logger(),
              "Fleet rejected mission '%s' for robot '%s': %s",
              mission_id.c_str(),
              robot_id.c_str(),
              response->message.c_str());

            release_robot_reservation(robot_id);
          }
        } catch (const std::exception & exception) {
          RCLCPP_ERROR(
            get_logger(),
            "Mission '%s' dispatch failed for robot '%s': %s",
            mission_id.c_str(),
            robot_id.c_str(),
            exception.what());

          release_robot_reservation(robot_id);
        }

        publish_queue_state();

        std::lock_guard<std::mutex> lock(request_mutex_);
        dispatch_in_progress_ = false;
      });
  }

  void publish_queue_state()
  {
    MissionQueueState message;
    message.stamp = now();

    {
      std::lock_guard<std::mutex> lock(queue_mutex_);

      message.pending_count =
        static_cast<uint32_t>(
          mission_queue_.size());

      message.mission_ids.reserve(
        mission_queue_.size());

      message.mission_types.reserve(
        mission_queue_.size());

      message.priorities.reserve(
        mission_queue_.size());

      for (const auto & mission :
           mission_queue_)
      {
        message.mission_ids.push_back(
          mission.mission_id);

        message.mission_types.push_back(
          mission.mission_type);

        message.priorities.push_back(
          mission.priority);
      }
    }

    queue_state_publisher_->publish(message);
  }

  std::string fleet_state_topic_;
  std::string assign_mission_service_name_;

  double minimum_battery_percentage_;
  double reservation_timeout_seconds_;

  bool system_ready_{false};

  mutable std::mutex state_mutex_;
  FleetState latest_fleet_state_;
  bool received_fleet_state_{false};

  mutable std::mutex queue_mutex_;
  std::vector<QueuedMission> mission_queue_;
  uint64_t next_sequence_{0};
  uint64_t mission_counter_{0};

  std::unordered_set<std::string>
    reserved_robots_;

  std::unordered_map<
    std::string,
    rclcpp::Time>
    reservation_times_;

  std::mutex request_mutex_;
  bool dispatch_in_progress_{false};

  rclcpp::Subscription<FleetState>::SharedPtr
    fleet_state_subscription_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    system_ready_subscription_;

  rclcpp::Client<AssignMission>::SharedPtr
    assign_mission_client_;

  rclcpp::Service<QueueMission>::SharedPtr
    queue_mission_service_;

  rclcpp::Service<Trigger>::SharedPtr
    dispatch_patrol_service_;

  rclcpp::Publisher<MissionQueueState>::SharedPtr
    queue_state_publisher_;

  rclcpp::TimerBase::SharedPtr
    dispatch_timer_;
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