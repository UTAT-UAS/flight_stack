#include "rclcpp/rclcpp.hpp"

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/goto_setpoint.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_land_detected.hpp"
#include "px4_msgs/srv/vehicle_command.hpp"

#include "flight_stack_msgs/srv/core_command.hpp"
#include "flight_stack_msgs/msg/core_status.hpp"

#include <chrono>
#include <optional>
#include <iostream>
#include <thread>
#include <cmath>

enum class CoreMode
{
    GOTO = 0,
    TRAJ,
    TRAJ_PATH,
    OVERRIDE,
    OUT_OF_BOUNDS
};

class FlightCore : public rclcpp::Node
{
public:
    explicit FlightCore(bool is_simulation);

private:
    const bool is_simulation_;

    bool drone_state_initalized_{false};
    CoreMode core_mode_{CoreMode::GOTO};

    rclcpp::CallbackGroup::SharedPtr service_call_group_;
    rclcpp::CallbackGroup::SharedPtr subscriber_call_group_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // PX4 pub/subs ...
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subscription_;
    void vehicle_status_subscription_callback(px4_msgs::msg::VehicleStatus::UniquePtr msg);
    bool vehicle_status_initialized_{false};
    bool is_armed_;
    bool is_offboard_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr vehicle_land_detected_subscription_;
    void vehicle_land_detected_callback(px4_msgs::msg::VehicleLandDetected::UniquePtr msg);
    bool vehicle_land_detected_initialized_{false};
    bool is_landed_;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    void publish_offboard_control_mode();
    rclcpp::Publisher<px4_msgs::msg::GotoSetpoint>::SharedPtr goto_setpoint_publisher_;
    void publish_goto_setpoint(float x, float y, float z, std::optional<float> heading);
    void publish_goto_setpoint_raw(px4_msgs::msg::GotoSetpoint msg);
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    void publish_trajectory_setpoint_raw(px4_msgs::msg::TrajectorySetpoint msg);

    rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicle_command_client_;
    void vehicle_command_request(uint16_t command, float param1, float param2);
    void vehicle_command_callback(rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future);
    void vehicle_command_request(std::shared_ptr<rclcpp::Service<flight_stack_msgs::srv::CoreCommand>> service,
                                 const std::shared_ptr<rmw_request_id_t> request_header,
                                 uint16_t command, float param1, float param2);
    void vehicle_command_callback(std::shared_ptr<rclcpp::Service<flight_stack_msgs::srv::CoreCommand>> service,
                                  const std::shared_ptr<rmw_request_id_t> request_header,
                                  rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future);

    // FlightCore pub/subs ...
    rclcpp::Subscription<px4_msgs::msg::GotoSetpoint>::SharedPtr goto_setpoint_subscriber_;
    void goto_setpoint_callback(px4_msgs::msg::GotoSetpoint::UniquePtr msg);
    px4_msgs::msg::GotoSetpoint::UniquePtr goto_setpoint_{};
    rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_subscriber_;
    void trajectory_setpoint_callback(px4_msgs::msg::TrajectorySetpoint::UniquePtr msg);
    px4_msgs::msg::TrajectorySetpoint::UniquePtr trajectory_setpoint_{};

    rclcpp::Publisher<flight_stack_msgs::msg::CoreStatus>::SharedPtr core_status_publisher_;
    void publish_status();

    rclcpp::Service<flight_stack_msgs::srv::CoreCommand>::SharedPtr core_command_service_;
    void core_command_callback(std::shared_ptr<rclcpp::Service<flight_stack_msgs::srv::CoreCommand>> service,
                               const std::shared_ptr<rmw_request_id_t> request_header,
                               const std::shared_ptr<flight_stack_msgs::srv::CoreCommand::Request> request_msg);

    // FlightCore functions
    void arm();
    void disarm();
    void land();
    void mode(float mode);
    void arm(std::shared_ptr<rclcpp::Service<flight_stack_msgs::srv::CoreCommand>> service,
             const std::shared_ptr<rmw_request_id_t> request_header);
    void disarm(std::shared_ptr<rclcpp::Service<flight_stack_msgs::srv::CoreCommand>> service,
                const std::shared_ptr<rmw_request_id_t> request_header);
    void land(std::shared_ptr<rclcpp::Service<flight_stack_msgs::srv::CoreCommand>> service,
              const std::shared_ptr<rmw_request_id_t> request_header);
    void mode(std::shared_ptr<rclcpp::Service<flight_stack_msgs::srv::CoreCommand>> service,
              const std::shared_ptr<rmw_request_id_t> request_header,
              float mode);

    void control_timer_callback();
};