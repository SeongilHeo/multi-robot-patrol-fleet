#include <algorithm>
#include <array>
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

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "mini_dogu_interfaces/msg/fleet_state.hpp"
#include "mini_dogu_interfaces/msg/mission_queue_state.hpp"
#include "mini_dogu_interfaces/msg/robot_heartbeat.hpp"
#include "mini_dogu_interfaces/srv/assign_mission.hpp"
#include "mini_dogu_interfaces/srv/queue_mission.hpp"
#include "mini_dogu_interfaces/srv/cancel_mission.hpp"

using namespace std::chrono_literals;

namespace
{
const std::array<const char *, 5> kSupportedMissionTypes{
  "patrol",
  "go_to",
  "return_home",
  "charge",
  "inspect"
};

bool is_supported_mission_type(const std::string & mission_type)
{
  return std::find(
    kSupportedMissionTypes.begin(),
    kSupportedMissionTypes.end(),
    mission_type) != kSupportedMissionTypes.end();
}
}  // namespace

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

using CancelMission =
  mini_dogu_interfaces::srv::CancelMission;

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

    dispatch_timeout_seconds_ =
      declare_parameter<double>(
      "dispatch_timeout_seconds",
      15.0);

    mission_start_timeout_seconds_ =
      declare_parameter<double>(
      "mission_start_timeout_seconds",
      15.0);

    max_mission_attempts_ =
      declare_parameter<int>(
      "max_mission_attempts",
      3);

    recurring_patrol_period_seconds_ =
      declare_parameter<double>(
      "recurring_patrol_period_seconds",
      0.0);

    recurring_patrol_priority_ =
      declare_parameter<int>(
      "recurring_patrol_priority",
      50);

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

    cancel_mission_service_ =
      create_service<CancelMission>(
      "/scheduler/cancel_mission",
      std::bind(
        &MissionSchedulerNode::handle_cancel_mission,
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

    if (recurring_patrol_period_seconds_ > 0.0) {
      const auto recurring_period =
        std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(
          recurring_patrol_period_seconds_));

      recurring_patrol_timer_ =
        create_wall_timer(
        recurring_period,
        std::bind(
          &MissionSchedulerNode::enqueue_recurring_patrol,
          this));

      RCLCPP_INFO(
        get_logger(),
        "Recurring patrol enabled: every %.1f seconds at priority %d",
        recurring_patrol_period_seconds_,
        recurring_patrol_priority_);
    }

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
    geometry_msgs::msg::PoseStamped target_pose;
    bool has_target{false};

    /*
     * How many times this mission (same type/target, regardless of
     * mission_id) has already been dispatched and failed. A mission
     * that's inherently undispatchable - an unreachable target, not a
     * robot problem - would otherwise bounce between robots forever;
     * see enqueue_mission() and reconcile_active_missions_locked().
     */
    uint32_t attempt_count{0};
  };

  struct CandidateRobot
  {
    std::string robot_id;
    float battery_percentage{0.0F};
  };

  struct ActiveMission
  {
    std::string mission_id;
    std::string mission_type;
    int32_t priority{0};
    geometry_msgs::msg::PoseStamped target_pose;
    bool has_target{false};

    /*
     * fleet_manager acks a dispatch before the robot itself confirms
     * it, and /fleet/state is republished on its own 1s timer
     * independent of the dispatch. So the very next fleet_state
     * snapshot after a dispatch can still carry the robot's pre-
     * dispatch IDLE reading. robot_confirmed_active only flips true
     * once we've actually observed the robot doing something for this
     * mission, so a stale IDLE can't be mistaken for completion.
     */
    bool robot_confirmed_active{false};

    rclcpp::Time dispatched_at{
      0,
      0,
      RCL_ROS_TIME};

    uint32_t attempt_count{0};
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
    reconcile_active_missions_locked();
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

  /*
   * Called with state_mutex_ already held by fleet_state_callback. A
   * robot that goes confirmed-offline or reports STATE_ERROR while it
   * has an active mission is treated as a failed mission and the same
   * mission (new id, same type/priority/target) goes back on the queue
   * for another robot to pick up. A robot that returns to IDLE with an
   * active mission on record is treated as having completed it.
   */
  void reconcile_active_missions_locked()
  {
    std::vector<std::string> completed_robot_ids;
    std::vector<std::pair<std::string, ActiveMission>> failed;

    {
      std::lock_guard<std::mutex> lock(active_missions_mutex_);

      if (active_missions_.empty()) {
        return;
      }

      const rclcpp::Time current_time = now();

      for (auto & entry : active_missions_) {
        const auto & robot_id = entry.first;
        auto & active = entry.second;

        const auto robot_iterator =
          std::find_if(
            latest_fleet_state_.robots.begin(),
            latest_fleet_state_.robots.end(),
            [&robot_id](const auto & robot)
            {
              return robot.robot_id == robot_id;
            });

        if (robot_iterator == latest_fleet_state_.robots.end()) {
          continue;
        }

        const bool robot_failed =
          robot_iterator->confirmed_offline ||
          robot_iterator->state == RobotHeartbeat::STATE_ERROR;

        if (robot_failed) {
          failed.emplace_back(robot_id, active);
          continue;
        }

        const bool robot_idle =
          robot_iterator->state == RobotHeartbeat::STATE_IDLE;

        if (!robot_idle) {
          active.robot_confirmed_active = true;
          continue;
        }

        if (active.robot_confirmed_active) {
          completed_robot_ids.push_back(robot_id);
          continue;
        }

        /*
         * Still IDLE and never observed otherwise for this mission.
         * Either fleet_state hasn't caught up with the dispatch yet,
         * or the START request was silently dropped (fleet_manager
         * acks dispatch before the robot itself confirms it). Only
         * treat it as a failed dispatch once it's had a fair chance
         * to actually start.
         */
        const double seconds_since_dispatch =
          (current_time - active.dispatched_at).seconds();

        if (seconds_since_dispatch > mission_start_timeout_seconds_) {
          failed.emplace_back(robot_id, active);
        }
      }

      for (const auto & robot_id : completed_robot_ids) {
        active_missions_.erase(robot_id);
      }

      for (const auto & entry : failed) {
        active_missions_.erase(entry.first);
      }
    }

    for (const auto & robot_id : completed_robot_ids) {
      RCLCPP_INFO(
        get_logger(),
        "Robot '%s' completed its active mission",
        robot_id.c_str());
    }

    for (const auto & entry : failed) {
      const auto & robot_id = entry.first;
      const auto & mission = entry.second;

      const uint32_t next_attempt_count = mission.attempt_count + 1;

      if (
        next_attempt_count >=
        static_cast<uint32_t>(max_mission_attempts_))
      {
        /*
         * A mission that fails on every robot that tries it isn't a
         * robot problem (an unreachable target, for example) and
         * re-queuing it again would just cycle every robot in the
         * fleet through STATE_ERROR forever. Drop it instead of
         * retrying indefinitely.
         */
        RCLCPP_ERROR(
          get_logger(),
          "Mission '%s' (type='%s') failed on robot '%s' after %u "
          "attempt(s); dropping it instead of re-queuing again",
          mission.mission_id.c_str(),
          mission.mission_type.c_str(),
          robot_id.c_str(),
          next_attempt_count);

        continue;
      }

      RCLCPP_WARN(
        get_logger(),
        "Mission '%s' on robot '%s' failed (offline, error, or never "
        "started); re-queuing for reassignment (attempt %u/%d)",
        mission.mission_id.c_str(),
        robot_id.c_str(),
        next_attempt_count,
        max_mission_attempts_);

      std::string new_mission_id;

      enqueue_mission(
        mission.mission_type,
        mission.priority,
        mission.target_pose,
        mission.has_target,
        new_mission_id,
        next_attempt_count);
    }

    if (!failed.empty()) {
      publish_queue_state();
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
    const geometry_msgs::msg::PoseStamped & target_pose,
    bool has_target,
    std::string & mission_id,
    uint32_t attempt_count = 0)
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);

    mission_id = create_mission_id();

    mission_queue_.push_back(
      QueuedMission{
        mission_id,
        mission_type,
        priority,
        next_sequence_++,
        target_pose,
        has_target,
        attempt_count
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

  void enqueue_recurring_patrol()
  {
    std::string mission_id;

    enqueue_mission(
      "patrol",
      recurring_patrol_priority_,
      geometry_msgs::msg::PoseStamped{},
      false,
      mission_id);

    publish_queue_state();

    RCLCPP_INFO(
      get_logger(),
      "Recurring patrol enqueued mission '%s'",
      mission_id.c_str());
  }

  void handle_queue_mission(
    const std::shared_ptr<QueueMission::Request> request,
    std::shared_ptr<QueueMission::Response> response)
  {
    if (!is_supported_mission_type(request->mission_type)) {
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
      request->target_pose,
      request->has_target,
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

  void handle_cancel_mission(
    const std::shared_ptr<CancelMission::Request> request,
    std::shared_ptr<CancelMission::Response> response)
  {
    if (request->mission_id.empty()) {
      response->success = false;
      response->message =
        "mission_id cannot be empty";
      return;
    }

    bool removed = false;

    {
      std::lock_guard<std::mutex> lock(queue_mutex_);

      const auto iterator =
        std::find_if(
          mission_queue_.begin(),
          mission_queue_.end(),
          [&request](const QueuedMission & mission)
          {
            return mission.mission_id ==
                  request->mission_id;
          });

      if (iterator != mission_queue_.end()) {
        mission_queue_.erase(iterator);
        removed = true;
      }
    }

    if (!removed) {
      response->success = false;
      response->message =
        "Pending mission not found: " +
        request->mission_id;

      RCLCPP_WARN(
        get_logger(),
        "%s",
        response->message.c_str());

      return;
    }

    publish_queue_state();

    response->success = true;
    response->message =
      "Pending mission canceled: " +
      request->mission_id;

    RCLCPP_INFO(
      get_logger(),
      "%s",
      response->message.c_str());
  }

  void handle_dispatch_patrol(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    std::string mission_id;

    enqueue_mission(
      "patrol",
      0,
      geometry_msgs::msg::PoseStamped{},
      false,
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
      /*
       * STATE_ERROR only ever means "the robot's last mission attempt
       * failed" in this codebase (see robot_heartbeat_node's
       * patrol_status_callback) - there's no separate hardware/health
       * fault source for it. Excluding it here would let one
       * unreachable target permanently strand every robot that
       * happens to draw it, since nothing ever moves them back to
       * STATE_IDLE. Treat it the same as idle: available for a fresh,
       * possibly different, mission.
       */
      const bool available =
        robot.state == RobotHeartbeat::STATE_IDLE ||
        robot.state == RobotHeartbeat::STATE_ERROR;

      const bool battery_sufficient =
        robot.battery_percentage >=
        minimum_battery_percentage_;

      const bool reserved =
        reserved_robots_.count(
          robot.robot_id) > 0;

      if (
        robot.online &&
        !robot.confirmed_offline &&
        robot.localization_ok &&
        available &&
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

  void set_active_mission(
    const std::string & robot_id,
    ActiveMission mission)
  {
    mission.robot_confirmed_active = false;
    mission.dispatched_at = now();

    std::lock_guard<std::mutex> lock(active_missions_mutex_);

    active_missions_[robot_id] = std::move(mission);
  }

  void process_queue()
  {
    if (!system_ready_) {
      return;
    }

    std::string timed_out_robot_id;

    {
      std::lock_guard<std::mutex> lock(request_mutex_);

      if (dispatch_in_progress_) {
        const double elapsed_seconds =
          (now() - dispatch_started_at_).seconds();

        if (elapsed_seconds < dispatch_timeout_seconds_) {
          return;
        }

        /*
         * The AssignMission response never arrived (dropped DDS
         * message, fleet manager restart, ...). dispatch_in_progress_
         * only ever gets cleared from that response's callback, so
         * without this the scheduler would stay locked out of
         * dispatching anything, forever, even with idle robots and a
         * full queue.
         */
        RCLCPP_ERROR(
          get_logger(),
          "Dispatch to robot '%s' timed out after %.1f seconds with "
          "no response; releasing the dispatch lock so the queue can "
          "proceed",
          dispatch_robot_id_.c_str(),
          elapsed_seconds);

        ++dispatch_generation_;
        dispatch_in_progress_ = false;

        timed_out_robot_id = dispatch_robot_id_;
        dispatch_robot_id_.clear();
      }
    }

    if (!timed_out_robot_id.empty()) {
      release_robot_reservation(timed_out_robot_id);
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
    request->mission_id = mission->mission_id;
    request->target_pose = mission->target_pose;
    request->has_target = mission->has_target;

    uint64_t generation = 0;

    {
      std::lock_guard<std::mutex> lock(request_mutex_);
      dispatch_in_progress_ = true;
      dispatch_started_at_ = now();
      dispatch_robot_id_ = robot->robot_id;
      generation = ++dispatch_generation_;
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
        mission_type = mission->mission_type,
        priority = mission->priority,
        target_pose = mission->target_pose,
        has_target = mission->has_target,
        attempt_count = mission->attempt_count,
        robot_id = robot->robot_id,
        generation
      ](
        rclcpp::Client<
          AssignMission>::SharedFuture future)
      {
        {
          std::lock_guard<std::mutex> lock(request_mutex_);

          if (generation != dispatch_generation_) {
            RCLCPP_WARN(
              get_logger(),
              "Ignoring stale dispatch response for mission '%s' / "
              "robot '%s' (a timeout already reclaimed this slot)",
              mission_id.c_str(),
              robot_id.c_str());
            return;
          }
        }

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

            ActiveMission new_active_mission{
              mission_id,
              mission_type,
              priority,
              target_pose,
              has_target
            };
            new_active_mission.attempt_count = attempt_count;

            set_active_mission(
              robot_id,
              new_active_mission);
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
  double dispatch_timeout_seconds_;
  double mission_start_timeout_seconds_;
  int max_mission_attempts_;
  double recurring_patrol_period_seconds_;
  int recurring_patrol_priority_;

  bool system_ready_{false};

  mutable std::mutex state_mutex_;
  FleetState latest_fleet_state_;
  bool received_fleet_state_{false};

  mutable std::mutex queue_mutex_;
  std::vector<QueuedMission> mission_queue_;
  uint64_t next_sequence_{0};
  uint64_t mission_counter_{0};

  std::mutex active_missions_mutex_;
  std::unordered_map<std::string, ActiveMission> active_missions_;

  std::unordered_set<std::string>
    reserved_robots_;

  std::unordered_map<
    std::string,
    rclcpp::Time>
    reservation_times_;

  std::mutex request_mutex_;
  bool dispatch_in_progress_{false};

  rclcpp::Time dispatch_started_at_{
    0,
    0,
    RCL_ROS_TIME};

  std::string dispatch_robot_id_;
  uint64_t dispatch_generation_{0};

  rclcpp::Subscription<FleetState>::SharedPtr
    fleet_state_subscription_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    system_ready_subscription_;

  rclcpp::Client<AssignMission>::SharedPtr
    assign_mission_client_;

  rclcpp::Service<QueueMission>::SharedPtr
    queue_mission_service_;

  rclcpp::Service<CancelMission>::SharedPtr
    cancel_mission_service_;

  rclcpp::Service<Trigger>::SharedPtr
    dispatch_patrol_service_;

  rclcpp::Publisher<MissionQueueState>::SharedPtr
    queue_state_publisher_;

  rclcpp::TimerBase::SharedPtr
    dispatch_timer_;

  rclcpp::TimerBase::SharedPtr
    recurring_patrol_timer_;
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
