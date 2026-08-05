#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "mini_dogu_interfaces/msg/fleet_robot_state.hpp"
#include "mini_dogu_interfaces/msg/fleet_state.hpp"
#include "mini_dogu_interfaces/msg/robot_heartbeat.hpp"

using namespace std::chrono_literals;

class FleetManagerNode : public rclcpp::Node
{
public:
    using Trigger = std_srvs::srv::Trigger;

    FleetManagerNode()
        : Node("fleet_manager")
    {
        robot_ids_ = declare_parameter<std::vector<std::string>>(
            "robot_ids",
            std::vector<std::string>{"robot1", "robot2"});

        heartbeat_timeout_seconds_ = declare_parameter<double>(
            "heartbeat_timeout_seconds",
            3.0);

        patrol_robot_ = declare_parameter<std::string>(
            "patrol_robot",
            "robot1");

        fleet_state_publisher_ =
            create_publisher<mini_dogu_interfaces::msg::FleetState>(
                "/fleet/state",
                rclcpp::QoS(10).reliable());

        configure_robot_monitoring();
        configure_patrol_control();

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
            mini_dogu_interfaces::msg::RobotHeartbeat::STATE_UNKNOWN};

        float battery_percentage{0.0F};
        std::string current_mission;

        rclcpp::Time last_heartbeat{
            0,
            0,
            RCL_ROS_TIME};
    };

    void configure_robot_monitoring()
    {
        for (const auto & robot_id : robot_ids_)
        {
            RobotRecord record;
            record.robot_id = robot_id;
            records_.emplace(robot_id, record);

            const std::string topic =
                "/" + robot_id + "/heartbeat";

            auto subscription =
                create_subscription<
                    mini_dogu_interfaces::msg::RobotHeartbeat>(
                    topic,
                    rclcpp::QoS(10).reliable(),
                    [this, robot_id](
                        const mini_dogu_interfaces::msg::
                            RobotHeartbeat::SharedPtr message)
                    {
                        heartbeat_callback(robot_id, *message);
                    });

            heartbeat_subscriptions_.push_back(subscription);

            RCLCPP_INFO(
                get_logger(),
                "Monitoring robot '%s' on topic '%s'",
                robot_id.c_str(),
                topic.c_str());
        }
    }

    void configure_patrol_control()
    {
        const std::string patrol_prefix =
            "/" + patrol_robot_ + "/patrol";

        patrol_start_client_ =
            create_client<Trigger>(
                patrol_prefix + "/start");

        patrol_stop_client_ =
            create_client<Trigger>(
                patrol_prefix + "/stop");

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
            "Fleet patrol control configured for robot '%s'",
            patrol_robot_.c_str());

        RCLCPP_INFO(
            get_logger(),
            "Patrol start client: %s/start",
            patrol_prefix.c_str());

        RCLCPP_INFO(
            get_logger(),
            "Patrol stop client: %s/stop",
            patrol_prefix.c_str());
    }

    void heartbeat_callback(
        const std::string & configured_robot_id,
        const mini_dogu_interfaces::msg::RobotHeartbeat & message)
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
        record.last_heartbeat = now();
    }

    void handle_dispatch_patrol(
        const std::shared_ptr<Trigger::Request>,
        std::shared_ptr<Trigger::Response> response)
    {
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

        call_patrol_service(
            patrol_start_client_,
            "start");

        response->success = true;
        response->message =
            "Patrol dispatch request sent to " +
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

        call_patrol_service(
            patrol_stop_client_,
            "stop");

        response->success = true;
        response->message =
            "Patrol stop request sent to " +
            patrol_robot_;
    }

    void call_patrol_service(
        const rclcpp::Client<Trigger>::SharedPtr & client,
        const std::string & operation)
    {
        auto request =
            std::make_shared<Trigger::Request>();

        client->async_send_request(
            request,
            [this, operation](
                rclcpp::Client<Trigger>::SharedFuture future)
            {
                try
                {
                    const auto response = future.get();

                    if (response->success)
                    {
                        RCLCPP_INFO(
                            get_logger(),
                            "Patrol %s succeeded: %s",
                            operation.c_str(),
                            response->message.c_str());
                    }
                    else
                    {
                        RCLCPP_WARN(
                            get_logger(),
                            "Patrol %s rejected: %s",
                            operation.c_str(),
                            response->message.c_str());
                    }
                }
                catch (const std::exception & exception)
                {
                    RCLCPP_ERROR(
                        get_logger(),
                        "Patrol %s service call failed: %s",
                        operation.c_str(),
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
        mini_dogu_interfaces::msg::FleetState fleet_message;
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

            const auto & record =
                record_iterator->second;

            mini_dogu_interfaces::msg::FleetRobotState
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

            if (!record.received_heartbeat)
            {
                robot_state.online = false;
            }
            else
            {
                const double age_seconds =
                    (current_time -
                    record.last_heartbeat).seconds();

                robot_state.online =
                    age_seconds <=
                    heartbeat_timeout_seconds_;
            }

            fleet_message.robots.push_back(
                std::move(robot_state));
        }

        fleet_state_publisher_->publish(fleet_message);
    }

    std::vector<std::string> robot_ids_;
    double heartbeat_timeout_seconds_;
    std::string patrol_robot_;

    std::unordered_map<std::string, RobotRecord> records_;
    std::mutex records_mutex_;

    std::vector<
        rclcpp::Subscription<
            mini_dogu_interfaces::msg::
                RobotHeartbeat>::SharedPtr>
        heartbeat_subscriptions_;
        
    rclcpp::Publisher<
        mini_dogu_interfaces::msg::FleetState>::SharedPtr
        fleet_state_publisher_;

    rclcpp::Client<Trigger>::SharedPtr
        patrol_start_client_;

    rclcpp::Client<Trigger>::SharedPtr
        patrol_stop_client_;

    rclcpp::Service<Trigger>::SharedPtr
        dispatch_patrol_service_;

    rclcpp::Service<Trigger>::SharedPtr
        stop_patrol_service_;

    rclcpp::TimerBase::SharedPtr
        publish_timer_;
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