#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "interfaces/msg/fleet_robot_state.hpp"
#include "interfaces/msg/fleet_state.hpp"
#include "interfaces/msg/robot_heartbeat.hpp"
#include "interfaces/msg/robot_incident.hpp"

#include "interfaces/srv/assign_mission.hpp"
#include "interfaces/srv/queue_mission.hpp"
#include "interfaces/srv/report_incident.hpp"
#include "interfaces/srv/start_mission.hpp"

#include "fleet/mission_store.hpp"

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

std::optional<uint8_t> mission_type_to_enum(
  const std::string & mission_type)
{
  using StartMission = interfaces::srv::StartMission;

  if (mission_type == "patrol") {
    return StartMission::Request::MISSION_PATROL;
  }
  if (mission_type == "go_to") {
    return StartMission::Request::MISSION_GO_TO;
  }
  if (mission_type == "return_home") {
    return StartMission::Request::MISSION_RETURN_HOME;
  }
  if (mission_type == "charge") {
    return StartMission::Request::MISSION_CHARGE;
  }
  if (mission_type == "inspect") {
    return StartMission::Request::MISSION_INSPECT;
  }

  return std::nullopt;
}

bool mission_type_requires_target(const std::string & mission_type)
{
  return mission_type == "go_to" || mission_type == "inspect";
}
}  // namespace

class FleetManagerNode : public rclcpp::Node
{
public:
  using Trigger = std_srvs::srv::Trigger;

  using AssignMission =
    interfaces::srv::AssignMission;

  using StartMission =
    interfaces::srv::StartMission;

  using QueueMission =
    interfaces::srv::QueueMission;

  using ReportIncident =
    interfaces::srv::ReportIncident;

  using RobotIncident =
    interfaces::msg::RobotIncident;

  FleetManagerNode()
    : Node("fleet_manager")
  {
    robot_ids_ = declare_parameter<std::vector<std::string>>(
      "robot_ids",
      std::vector<std::string>{"robot1", "robot2"});

    heartbeat_timeout_seconds_ = declare_parameter<double>(
      "heartbeat_timeout_seconds",
      3.0);

    offline_grace_seconds_ = declare_parameter<double>(
      "offline_grace_seconds",
      20.0);

    auto_estop_after_offline_seconds_ = declare_parameter<double>(
      "auto_estop_after_offline_seconds",
      0.0);

    patrol_robot_ = declare_parameter<std::string>(
      "patrol_robot",
      "robot1");

    mission_db_path_ = declare_parameter<std::string>(
      "mission_db_path",
      "/tmp/fleet.db");

    incident_auto_dispatch_min_severity_ =
      declare_parameter<int>(
      "incident_auto_dispatch_min_severity",
      RobotIncident::SEVERITY_HIGH);

    incident_auto_dispatch_priority_ =
      declare_parameter<int>(
      "incident_auto_dispatch_priority",
      100);

    mission_store_ =
      std::make_unique<fleet::MissionStore>(
      mission_db_path_);

    fleet_state_publisher_ =
      create_publisher<interfaces::msg::FleetState>(
        "/fleet/state",
        rclcpp::QoS(10).reliable());

    system_ready_subscription_ =
      create_subscription<std_msgs::msg::Bool>(
        "/system/ready",
        rclcpp::QoS(1)
          .reliable()
          .transient_local(),
        [this](const std_msgs::msg::Bool::SharedPtr message)
        {
          system_ready_callback(*message);
        });

    configure_robot_monitoring();
    configure_patrol_control();
    configure_safety();
    configure_incident_handling();

    publish_timer_ = create_wall_timer(
      1s,
      std::bind(&FleetManagerNode::publish_fleet_state, this));

    RCLCPP_INFO(
      get_logger(),
      "Fleet manager started with %zu robots",
      robot_ids_.size());
  }

private:
  struct RobotRecord
  {
    std::string robot_id;
    bool received_heartbeat{false};

    uint8_t state{
      interfaces::msg::RobotHeartbeat::STATE_UNKNOWN};

    float battery_percentage{0.0F};
    std::string current_mission;
    bool localization_ok{true};

    rclcpp::Time last_heartbeat{
      0,
      0,
      RCL_ROS_TIME};

    // Zero/invalid while online; set once when the robot is first
    // observed offline so confirmed_offline can require a sustained
    // dropout instead of reacting to a single missed heartbeat.
    rclcpp::Time offline_since{
      0,
      0,
      RCL_ROS_TIME};

    bool auto_estopped{false};
  };

  void configure_robot_monitoring()
  {
    for (const auto & robot_id : robot_ids_)
    {
      RobotRecord record;
      record.robot_id = robot_id;
      records_.emplace(robot_id, record);

      const std::string heartbeat_topic =
        "/" + robot_id + "/heartbeat";

      auto heartbeat_subscription =
        create_subscription<
          interfaces::msg::RobotHeartbeat>(
          heartbeat_topic,
          rclcpp::QoS(10).reliable(),
          [this, robot_id](
            const interfaces::msg::
              RobotHeartbeat::SharedPtr message)
          {
            heartbeat_callback(
              robot_id,
              *message);
          });

      heartbeat_subscriptions_.push_back(
        heartbeat_subscription);

      RCLCPP_INFO(
        get_logger(),
        "Monitoring heartbeat for robot '%s' on '%s'",
        robot_id.c_str(),
        heartbeat_topic.c_str());
    }
  }

  void configure_patrol_control()
  {
    for (const auto & robot_id : robot_ids_)
    {
      const std::string patrol_prefix =
        "/" + robot_id + "/patrol";

      patrol_start_clients_.emplace(
        robot_id,
        create_client<StartMission>(
          patrol_prefix + "/start"));

      patrol_stop_clients_.emplace(
        robot_id,
        create_client<Trigger>(
          patrol_prefix + "/stop"));

      RCLCPP_INFO(
        get_logger(),
        "Configured patrol clients for robot '%s'",
        robot_id.c_str());
    }

    assign_mission_service_ =
      create_service<AssignMission>(
        "/fleet/assign_mission",
        std::bind(
          &FleetManagerNode::handle_assign_mission,
          this,
          std::placeholders::_1,
          std::placeholders::_2));

    const auto start_it =
      patrol_start_clients_.find(patrol_robot_);

    const auto stop_it =
      patrol_stop_clients_.find(patrol_robot_);

    if (
      start_it == patrol_start_clients_.end() ||
      stop_it == patrol_stop_clients_.end())
    {
      throw std::runtime_error(
        "Invalid patrol_robot parameter: " +
        patrol_robot_);
    }

    patrol_start_client_ =
      start_it->second;

    patrol_stop_client_ =
      stop_it->second;

    dispatch_patrol_service_ =
      create_service<Trigger>(
        "/fleet/dispatch_patrol",
        std::bind(
          &FleetManagerNode::handle_dispatch_patrol,
          this,
          std::placeholders::_1,
          std::placeholders::_2));

    stop_patrol_service_ =
      create_service<Trigger>(
        "/fleet/stop_patrol",
        std::bind(
          &FleetManagerNode::handle_stop_patrol,
          this,
          std::placeholders::_1,
          std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "Mission assignment service ready at /fleet/assign_mission");
  }

  void configure_safety()
  {
    rclcpp::QoS estop_qos(1);
    estop_qos.reliable();
    estop_qos.transient_local();

    for (const auto & robot_id : robot_ids_)
    {
      estop_publishers_.emplace(
        robot_id,
        create_publisher<std_msgs::msg::Bool>(
          "/" + robot_id + "/estop",
          estop_qos));
    }

    estop_all_service_ =
      create_service<Trigger>(
        "/fleet/estop_all",
        std::bind(
          &FleetManagerNode::handle_estop_all,
          this,
          std::placeholders::_1,
          std::placeholders::_2));

    estop_release_all_service_ =
      create_service<Trigger>(
        "/fleet/estop_release_all",
        std::bind(
          &FleetManagerNode::handle_estop_release_all,
          this,
          std::placeholders::_1,
          std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "Safety gate wired for %zu robots; e-stop via /fleet/estop_all",
      robot_ids_.size());
  }

  void configure_incident_handling()
  {
    incident_publisher_ =
      create_publisher<RobotIncident>(
        "/fleet/incidents",
        rclcpp::QoS(20).reliable());

    queue_mission_client_ =
      create_client<QueueMission>(
        "/scheduler/queue_mission");

    report_incident_service_ =
      create_service<ReportIncident>(
        "/fleet/report_incident",
        std::bind(
          &FleetManagerNode::handle_report_incident,
          this,
          std::placeholders::_1,
          std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "Incident reporting ready at /fleet/report_incident "
      "(auto-dispatch at severity >= %d)",
      incident_auto_dispatch_min_severity_);
  }

  void heartbeat_callback(
    const std::string & configured_robot_id,
    const interfaces::msg::RobotHeartbeat & message)
  {
    std::lock_guard<std::mutex> lock(records_mutex_);

    auto record_iterator =
      records_.find(configured_robot_id);

    if (record_iterator == records_.end())
    {
      RCLCPP_WARN(
        get_logger(),
        "Heartbeat received for unregistered robot '%s'",
        configured_robot_id.c_str());
      return;
    }

    auto & record = record_iterator->second;

    if (
      !message.robot_id.empty() &&
      message.robot_id != configured_robot_id)
    {
      RCLCPP_WARN(
        get_logger(),
        "Heartbeat topic robot '%s' reported robot_id '%s'",
        configured_robot_id.c_str(),
        message.robot_id.c_str());
    }

    record.received_heartbeat = true;
    record.state = message.state;
    record.battery_percentage =
      message.battery_percentage;
    record.current_mission =
      message.current_mission;
    record.localization_ok =
      message.localization_ok;
    record.last_heartbeat = now();
  }

  void handle_dispatch_patrol(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    if (!system_ready_.load())
    {
      response->success = false;
      response->message =
        "System is not ready for patrol dispatch";

      RCLCPP_WARN(
        get_logger(),
        "%s",
        response->message.c_str());

      return;
    }
    if (!is_robot_online(patrol_robot_))
    {
      response->success = false;
      response->message =
        "Robot '" + patrol_robot_ + "' is offline";

      RCLCPP_WARN(
        get_logger(),
        "%s",
        response->message.c_str());
      return;
    }

    if (!patrol_start_client_->service_is_ready())
    {
      response->success = false;
      response->message =
        "Patrol start service is unavailable for " +
        patrol_robot_;

      RCLCPP_WARN(
        get_logger(),
        "%s",
        response->message.c_str());
      return;
    }

    call_patrol_start_service(
      patrol_start_client_,
      patrol_robot_,
      "manual-dispatch",
      "patrol",
      geometry_msgs::msg::PoseStamped{},
      false);

    response->success = true;
    response->message =
      "Patrol request accepted and forwarded to " +
      patrol_robot_;
  }

  void handle_stop_patrol(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    if (!patrol_stop_client_->service_is_ready())
    {
      response->success = false;
      response->message =
        "Patrol stop service is unavailable for " +
        patrol_robot_;

      RCLCPP_WARN(
        get_logger(),
        "%s",
        response->message.c_str());
      return;
    }

    call_patrol_stop_service(
      patrol_stop_client_,
      patrol_robot_,
      "manual-dispatch");

    response->success = true;
    response->message =
      "Patrol cancellation request accepted and forwarded to " +
      patrol_robot_;
  }

  void call_patrol_start_service(
    const rclcpp::Client<StartMission>::SharedPtr & client,
    const std::string & robot_id,
    const std::string & mission_id,
    const std::string & mission_type,
    const geometry_msgs::msg::PoseStamped & target_pose,
    bool has_target)
  {
    auto request =
      std::make_shared<StartMission::Request>();

    request->mission_type =
      mission_type_to_enum(mission_type).value_or(
        StartMission::Request::MISSION_PATROL);

    request->target_pose = target_pose;
    request->has_target = has_target;

    client->async_send_request(
      request,
      [this, robot_id, mission_id, mission_type](
        rclcpp::Client<StartMission>::SharedFuture future)
      {
        try
        {
          const auto response = future.get();

          if (response->success)
          {
            RCLCPP_INFO(
              get_logger(),
              "Robot '%s' patrol start request accepted by Patrol Manager: %s",
              robot_id.c_str(),
              response->message.c_str());

            mission_store_->record_mission_event(
              mission_id,
              robot_id,
              mission_type,
              "START",
              "ACCEPTED_BY_ROBOT",
              response->message);
          }
          else
          {
            RCLCPP_WARN(
              get_logger(),
              "Robot '%s' patrol start request rejected by Patrol Manager: %s",
              robot_id.c_str(),
              response->message.c_str());

            mission_store_->record_mission_event(
              mission_id,
              robot_id,
              mission_type,
              "START",
              "REJECTED_BY_ROBOT",
              response->message);
          }
        }
        catch (const std::exception & exception)
        {
          RCLCPP_ERROR(
            get_logger(),
            "Robot '%s' patrol start failed: %s",
            robot_id.c_str(),
            exception.what());

          mission_store_->record_mission_event(
            mission_id,
            robot_id,
            mission_type,
            "START",
            "FAILED",
            exception.what());
        }
      });
  }

  void call_patrol_stop_service(
    const rclcpp::Client<Trigger>::SharedPtr & client,
    const std::string & robot_id,
    const std::string & mission_id)
  {
    auto request =
      std::make_shared<Trigger::Request>();

    client->async_send_request(
      request,
      [this, robot_id, mission_id](
        rclcpp::Client<Trigger>::SharedFuture future)
      {
        try
        {
          const auto response = future.get();

          if (response->success)
          {
            RCLCPP_INFO(
              get_logger(),
              "Robot '%s' patrol stop request accepted by Patrol Manager: %s",
              robot_id.c_str(),
              response->message.c_str());

            mission_store_->record_mission_event(
              mission_id,
              robot_id,
              "",
              "STOP",
              "ACCEPTED_BY_ROBOT",
              response->message);
          }
          else
          {
            RCLCPP_WARN(
              get_logger(),
              "Robot '%s' patrol stop request rejected by Patrol Manager: %s",
              robot_id.c_str(),
              response->message.c_str());

            mission_store_->record_mission_event(
              mission_id,
              robot_id,
              "",
              "STOP",
              "REJECTED_BY_ROBOT",
              response->message);
          }
        }
        catch (const std::exception & exception)
        {
          RCLCPP_ERROR(
            get_logger(),
            "Robot '%s' patrol stop failed: %s",
            robot_id.c_str(),
            exception.what());

          mission_store_->record_mission_event(
            mission_id,
            robot_id,
            "",
            "STOP",
            "FAILED",
            exception.what());
        }
      });
  }

  bool is_robot_online(const std::string & robot_id)
  {
    std::lock_guard<std::mutex> lock(records_mutex_);

    const auto iterator = records_.find(robot_id);

    if (iterator == records_.end())
    {
      return false;
    }

    const auto & record = iterator->second;

    if (!record.received_heartbeat)
    {
      return false;
    }

    const double age_seconds =
      (now() - record.last_heartbeat).seconds();

    return age_seconds <= heartbeat_timeout_seconds_;
  }

  void publish_fleet_state()
  {
    interfaces::msg::FleetState fleet_message;
    fleet_message.stamp = now();

    const rclcpp::Time current_time = now();

    std::lock_guard<std::mutex> lock(records_mutex_);

    fleet_message.robots.reserve(records_.size());

    for (const auto & robot_id : robot_ids_)
    {
      const auto record_iterator =
        records_.find(robot_id);

      if (record_iterator == records_.end())
      {
        continue;
      }

      auto & record =
        record_iterator->second;

      interfaces::msg::FleetRobotState
        robot_state;

      robot_state.robot_id =
        record.robot_id;

      robot_state.state =
        record.state;

      robot_state.battery_percentage =
        record.battery_percentage;

      robot_state.current_mission =
        record.current_mission;

      robot_state.last_heartbeat =
        record.last_heartbeat;

      robot_state.localization_ok =
        record.localization_ok;

      bool online = false;

      if (record.received_heartbeat)
      {
        const double age_seconds =
          (current_time -
          record.last_heartbeat).seconds();

        online = age_seconds <= heartbeat_timeout_seconds_;
      }

      robot_state.online = online;

      if (online)
      {
        record.offline_since = rclcpp::Time(0, 0, RCL_ROS_TIME);
        record.auto_estopped = false;
      }
      else if (record.offline_since.nanoseconds() <= 0)
      {
        record.offline_since = current_time;
      }

      const double offline_seconds =
        online
        ? 0.0
        : (current_time - record.offline_since).seconds();

      robot_state.confirmed_offline =
        !online && offline_seconds >= offline_grace_seconds_;

      if (
        !online &&
        auto_estop_after_offline_seconds_ > 0.0 &&
        offline_seconds >= auto_estop_after_offline_seconds_ &&
        !record.auto_estopped)
      {
        record.auto_estopped = true;
        publish_estop(robot_id, true);

        RCLCPP_WARN(
          get_logger(),
          "Robot '%s' unreachable for %.1f seconds; "
          "engaging e-stop automatically (requires manual release)",
          robot_id.c_str(),
          offline_seconds);
      }

      fleet_message.robots.push_back(
        std::move(robot_state));
    }

    fleet_state_publisher_->publish(fleet_message);
  }

  bool robot_exists(const std::string & robot_id)
  {
    std::lock_guard<std::mutex> lock(records_mutex_);

    return records_.find(robot_id) != records_.end();
  }

  void reject_assign_mission(
    std::shared_ptr<AssignMission::Response> & response,
    const std::string & mission_id,
    const std::string & robot_id,
    const std::string & mission_type,
    const std::string & message)
  {
    response->success = false;
    response->message = message;

    mission_store_->record_mission_event(
      mission_id,
      robot_id,
      mission_type,
      "START",
      "REJECTED_BY_FLEET",
      message);
  }

  void handle_assign_mission(
    const std::shared_ptr<AssignMission::Request> request,
    std::shared_ptr<AssignMission::Response> response)
  {
    if (!robot_exists(request->robot_id))
    {
      response->success = false;
      response->message =
        "Unknown robot_id: " + request->robot_id;
      return;
    }

    if (!is_supported_mission_type(request->mission_type))
    {
      response->success = false;
      response->message =
        "Unsupported mission_type: " +
        request->mission_type;
      return;
    }

    if (
      request->command !=
        AssignMission::Request::COMMAND_START &&
      request->command !=
        AssignMission::Request::COMMAND_STOP)
    {
      response->success = false;
      response->message =
        "Unsupported mission command";
      return;
    }

    if (
      request->command ==
      AssignMission::Request::COMMAND_START)
    {
      if (!system_ready_.load())
      {
        reject_assign_mission(
          response,
          request->mission_id,
          request->robot_id,
          request->mission_type,
          "System is not ready for mission dispatch");

        RCLCPP_WARN(
          get_logger(),
          "Mission rejected for robot '%s': system is NOT_READY",
          request->robot_id.c_str());

        return;
      }

      if (!is_robot_online(request->robot_id))
      {
        reject_assign_mission(
          response,
          request->mission_id,
          request->robot_id,
          request->mission_type,
          "Robot '" + request->robot_id + "' is offline");
        return;
      }

      if (
        mission_type_requires_target(request->mission_type) &&
        !request->has_target)
      {
        reject_assign_mission(
          response,
          request->mission_id,
          request->robot_id,
          request->mission_type,
          "mission_type '" + request->mission_type +
          "' requires a target_pose");
        return;
      }

      const auto client_iterator =
        patrol_start_clients_.find(
          request->robot_id);

      if (
        client_iterator ==
        patrol_start_clients_.end())
      {
        reject_assign_mission(
          response,
          request->mission_id,
          request->robot_id,
          request->mission_type,
          "No patrol start client for " + request->robot_id);
        return;
      }

      if (!client_iterator->second->service_is_ready())
      {
        reject_assign_mission(
          response,
          request->mission_id,
          request->robot_id,
          request->mission_type,
          "Patrol start service unavailable for " +
          request->robot_id);
        return;
      }

      call_patrol_start_service(
        client_iterator->second,
        request->robot_id,
        request->mission_id,
        request->mission_type,
        request->target_pose,
        request->has_target);

      mission_store_->record_mission_event(
        request->mission_id,
        request->robot_id,
        request->mission_type,
        "START",
        "DISPATCHED",
        "forwarded to robot by fleet manager");

      response->success = true;
      response->message =
        "Mission request accepted and forwarded to " +
        request->robot_id +
        "; execution status is available on /fleet/state";
      return;
    }

    const auto client_iterator =
      patrol_stop_clients_.find(
        request->robot_id);

    if (
      client_iterator ==
      patrol_stop_clients_.end())
    {
      response->success = false;
      response->message =
        "No patrol stop client for " +
        request->robot_id;
      return;
    }

    if (!client_iterator->second->service_is_ready())
    {
      response->success = false;
      response->message =
        "Patrol stop service unavailable for " +
        request->robot_id;
      return;
    }

    call_patrol_stop_service(
      client_iterator->second,
      request->robot_id,
      request->mission_id);

    response->success = true;
    response->message =
      "Mission cancellation request accepted and forwarded to " +
      request->robot_id +
      "; final status is available on /fleet/state";
  }

  void publish_estop(const std::string & robot_id, bool active)
  {
    const auto iterator = estop_publishers_.find(robot_id);

    if (iterator == estop_publishers_.end())
    {
      return;
    }

    std_msgs::msg::Bool message;
    message.data = active;

    iterator->second->publish(message);
  }

  void handle_estop_all(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    for (const auto & robot_id : robot_ids_)
    {
      publish_estop(robot_id, true);
    }

    RCLCPP_WARN(
      get_logger(),
      "E-STOP engaged fleet-wide via /fleet/estop_all");

    response->success = true;
    response->message =
      "E-stop engaged for all robots";
  }

  void handle_estop_release_all(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    for (const auto & robot_id : robot_ids_)
    {
      publish_estop(robot_id, false);

      std::lock_guard<std::mutex> lock(records_mutex_);

      const auto iterator = records_.find(robot_id);

      if (iterator != records_.end())
      {
        iterator->second.auto_estopped = false;
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "E-STOP released fleet-wide via /fleet/estop_release_all");

    response->success = true;
    response->message =
      "E-stop released for all robots";
  }

  std::string generate_incident_id()
  {
    ++incident_counter_;

    return "incident-" + std::to_string(incident_counter_);
  }

  void handle_report_incident(
    const std::shared_ptr<ReportIncident::Request> request,
    std::shared_ptr<ReportIncident::Response> response)
  {
    if (!robot_exists(request->robot_id))
    {
      response->accepted = false;
      response->message =
        "Unknown robot_id: " + request->robot_id;
      return;
    }

    const std::string incident_id = generate_incident_id();

    mission_store_->record_incident(
      incident_id,
      request->robot_id,
      request->severity,
      request->category,
      request->description);

    RobotIncident incident_message;
    incident_message.incident_id = incident_id;
    incident_message.robot_id = request->robot_id;
    incident_message.stamp = now();
    incident_message.severity = request->severity;
    incident_message.category = request->category;
    incident_message.description = request->description;

    incident_publisher_->publish(incident_message);

    RCLCPP_WARN(
      get_logger(),
      "Incident '%s' reported for robot '%s' (severity=%d): %s",
      incident_id.c_str(),
      request->robot_id.c_str(),
      request->severity,
      request->description.c_str());

    response->accepted = true;
    response->incident_id = incident_id;
    response->message =
      "Incident '" + incident_id + "' recorded";

    if (
      static_cast<int>(request->severity) >=
      incident_auto_dispatch_min_severity_)
    {
      dispatch_incident_inspection(incident_id, *request);
    }
  }

  void dispatch_incident_inspection(
    const std::string & incident_id,
    const ReportIncident::Request & incident)
  {
    if (!queue_mission_client_->service_is_ready())
    {
      RCLCPP_WARN(
        get_logger(),
        "Incident '%s': scheduler queue is unavailable, "
        "cannot auto-dispatch an inspection",
        incident_id.c_str());
      return;
    }

    auto request = std::make_shared<QueueMission::Request>();
    request->mission_type = "inspect";
    request->priority = incident_auto_dispatch_priority_;
    request->has_target = false;

    queue_mission_client_->async_send_request(
      request,
      [this, incident_id](
        rclcpp::Client<QueueMission>::SharedFuture future)
      {
        try
        {
          const auto response = future.get();

          if (response->accepted)
          {
            RCLCPP_WARN(
              get_logger(),
              "Incident '%s' auto-queued inspection mission '%s'",
              incident_id.c_str(),
              response->mission_id.c_str());

            mission_store_->record_mission_event(
              response->mission_id,
              "",
              "inspect",
              "QUEUE",
              "AUTO_QUEUED_FROM_INCIDENT",
              incident_id);
          }
          else
          {
            RCLCPP_ERROR(
              get_logger(),
              "Incident '%s' failed to auto-queue inspection: %s",
              incident_id.c_str(),
              response->message.c_str());
          }
        }
        catch (const std::exception & exception)
        {
          RCLCPP_ERROR(
            get_logger(),
            "Incident '%s' auto-dispatch failed: %s",
            incident_id.c_str(),
            exception.what());
        }
      });
  }

  void system_ready_callback(
    const std_msgs::msg::Bool & message)
  {
    const bool previous_ready =
      system_ready_.exchange(message.data);

    if (message.data != previous_ready)
    {
      RCLCPP_INFO(
        get_logger(),
        "System readiness changed: %s",
        message.data
          ? "READY"
          : "NOT_READY");
    }
  }

  std::vector<std::string> robot_ids_;
  double heartbeat_timeout_seconds_;
  double offline_grace_seconds_;
  double auto_estop_after_offline_seconds_;
  std::string patrol_robot_;
  std::string mission_db_path_;
  int incident_auto_dispatch_min_severity_;
  int incident_auto_dispatch_priority_;

  std::unordered_map<std::string, RobotRecord> records_;
  std::mutex records_mutex_;

  std::unique_ptr<fleet::MissionStore> mission_store_;

  uint64_t incident_counter_{0};

  std::vector<
    rclcpp::Subscription<
      interfaces::msg::
        RobotHeartbeat>::SharedPtr>
    heartbeat_subscriptions_;

  std::unordered_map<
    std::string,
    rclcpp::Client<StartMission>::SharedPtr>
    patrol_start_clients_;

  std::unordered_map<
    std::string,
    rclcpp::Client<Trigger>::SharedPtr>
    patrol_stop_clients_;

  std::unordered_map<
    std::string,
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr>
    estop_publishers_;

  std::atomic_bool system_ready_{false};

  rclcpp::Service<AssignMission>::SharedPtr
    assign_mission_service_;

  rclcpp::Service<ReportIncident>::SharedPtr
    report_incident_service_;

  rclcpp::Publisher<RobotIncident>::SharedPtr
    incident_publisher_;

  rclcpp::Client<QueueMission>::SharedPtr
    queue_mission_client_;

  rclcpp::Publisher<
    interfaces::msg::FleetState>::SharedPtr
    fleet_state_publisher_;

  rclcpp::Client<StartMission>::SharedPtr
    patrol_start_client_;

  rclcpp::Client<Trigger>::SharedPtr
    patrol_stop_client_;

  rclcpp::Service<Trigger>::SharedPtr
    dispatch_patrol_service_;

  rclcpp::Service<Trigger>::SharedPtr
    stop_patrol_service_;

  rclcpp::Service<Trigger>::SharedPtr
    estop_all_service_;

  rclcpp::Service<Trigger>::SharedPtr
    estop_release_all_service_;

  rclcpp::TimerBase::SharedPtr
    publish_timer_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    system_ready_subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  try
  {
    rclcpp::spin(
      std::make_shared<FleetManagerNode>());
  }
  catch (const std::exception & exception)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("fleet_manager"),
      "Unhandled exception: %s",
      exception.what());

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
