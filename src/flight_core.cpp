#include "flight_stack/flight_core.hpp"

FlightCore::FlightCore(bool is_simulation = false) : Node("flight_core"), is_simulation_{is_simulation}
{
    if (is_simulation_)
    {
        RCLCPP_WARN(this->get_logger(), "Running in simulation mode!");
    }

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    // PX4
    vehicle_status_subscription_ = this->create_subscription<px4_msgs::msg::VehicleStatus>("/fmu/out/vehicle_status_v1", qos, std::bind(&FlightCore::vehicle_status_subscription_callback, this, std::placeholders::_1));
    vehicle_land_detected_subscription_ = this->create_subscription<px4_msgs::msg::VehicleLandDetected>("/fmu/out/vehicle_land_detected", qos, std::bind(&FlightCore::vehicle_land_detected_callback, this, std::placeholders::_1));

    offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    goto_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::GotoSetpoint>("/fmu/in/goto_setpoint", 10);
    trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);

    // TODO: prune stale service requests
    vehicle_command_client_ = this->create_client<px4_msgs::srv::VehicleCommand>("/fmu/vehicle_command", rmw_qos_profile_services_default);

    // FlightCore
    goto_setpoint_subscriber_ = this->create_subscription<px4_msgs::msg::GotoSetpoint>("/uas/core/goto_setpoint", qos, std::bind(&FlightCore::goto_setpoint_callback, this, std::placeholders::_1));
    trajectory_setpoint_subscriber_ = this->create_subscription<px4_msgs::msg::TrajectorySetpoint>("/uas/core/trajectory_setpoint", qos, std::bind(&FlightCore::trajectory_setpoint_callback, this, std::placeholders::_1));

    core_status_publisher_ = this->create_publisher<flight_stack_msgs::msg::CoreStatus>("/uas/core/status", 10);

    core_command_service_ = this->create_service<flight_stack_msgs::srv::CoreCommand>("/uas/core/command", std::bind(&FlightCore::core_command_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    // Startup
    RCLCPP_INFO_STREAM(this->get_logger(), "Setting up FlightCore");
    RCLCPP_INFO_STREAM(this->get_logger(), "Waiting for " << "/fmu/" << "vehicle_command service");
    while (!vehicle_command_client_->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Starting control loop");

    control_timer_ = this->create_wall_timer(std::chrono::milliseconds(20),
                                             [this]() -> void
                                             {
                                                 this->control_timer_callback();
                                             });
}

void FlightCore::vehicle_status_subscription_callback(px4_msgs::msg::VehicleStatus::UniquePtr msg)
{
    switch (msg->arming_state)
    {
    case px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED:
        is_armed_ = false;
        break;
    case px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED:
        is_armed_ = true;
        break;
    }
    switch (msg->nav_state)
    {
    case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD:
        is_offboard_ = true;
        break;
    default:
        is_offboard_ = false;
    }
    vehicle_status_initialized_ = true;
}

void FlightCore::vehicle_land_detected_callback(px4_msgs::msg::VehicleLandDetected::UniquePtr msg)
{
    is_landed_ = msg->landed;
    vehicle_land_detected_initialized_ = true;
}

void FlightCore::publish_offboard_control_mode()
{
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = true;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

void FlightCore::publish_goto_setpoint(float x, float y, float z, std::optional<float> heading)
{
    px4_msgs::msg::GotoSetpoint msg{};
    msg.position = {x, y, z};
    if (heading.has_value())
    {
        msg.flag_control_heading = true;
        msg.heading = heading.value();
    }
    else
    {
        msg.flag_control_heading = false;
    }
    // msg.flag_set_max_horizontal_speed = true;
    // msg.max_horizontal_speed = 1.0;
    // msg.flag_set_max_vertical_speed = false;
    // msg.max_horizontal_speed = 1.0;
    // msg.flag_set_max_heading_rate = false;
    // msg.max_heading_rate = 0.5;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    goto_setpoint_publisher_->publish(msg);
}

void FlightCore::publish_goto_setpoint_raw(px4_msgs::msg::GotoSetpoint msg)
{
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    goto_setpoint_publisher_->publish(msg);
}

void FlightCore::publish_trajectory_setpoint_raw(px4_msgs::msg::TrajectorySetpoint msg)
{
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

void FlightCore::vehicle_command_request(uint16_t command, float param1 = 0.0, float param2 = 0.0)
{
    auto request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();

    px4_msgs::msg::VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    request->request = msg;

    auto result = vehicle_command_client_->async_send_request(request, std::bind(static_cast<void (FlightCore::*)(rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture)>(&FlightCore::vehicle_command_callback), this, std::placeholders::_1));
}

void FlightCore::vehicle_command_callback(rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future)
{
    auto status = future.wait_for(std::chrono::seconds(1));
    if (status == std::future_status::ready)
    {
        auto reply = future.get()->reply;
        switch (reply.result)
        {
        case reply.VEHICLE_CMD_RESULT_ACCEPTED:
            RCLCPP_INFO(this->get_logger(), "command accepted");
            break;
        case reply.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
            RCLCPP_WARN(this->get_logger(), "command temporarily rejected");
            break;
        case reply.VEHICLE_CMD_RESULT_DENIED:
            RCLCPP_WARN(this->get_logger(), "command denied");
            break;
        case reply.VEHICLE_CMD_RESULT_UNSUPPORTED:
            RCLCPP_WARN(this->get_logger(), "command unsupported");
            break;
        case reply.VEHICLE_CMD_RESULT_FAILED:
            RCLCPP_WARN(this->get_logger(), "command failed");
            break;
        case reply.VEHICLE_CMD_RESULT_IN_PROGRESS:
            RCLCPP_WARN(this->get_logger(), "command in progress");
            break;
        case reply.VEHICLE_CMD_RESULT_CANCELLED:
            RCLCPP_WARN(this->get_logger(), "command cancelled");
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "command reply unknown");
            break;
        }
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
}

void FlightCore::vehicle_command_request(std::shared_ptr<rclcpp::Service<flight_stack_msgs::srv::CoreCommand>> service,
                                         const std::shared_ptr<rmw_request_id_t> request_header,
                                         uint16_t command, float param1 = 0.0, float param2 = 0.0)
{
    auto request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();

    px4_msgs::msg::VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    request->request = msg;

    auto async_cb = [this, service, request_header](rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future)
    {
        this->vehicle_command_callback(service, request_header, future);
    };

    auto result = vehicle_command_client_->async_send_request(request, async_cb);
}

void FlightCore::vehicle_command_callback(std::shared_ptr<rclcpp::Service<flight_stack_msgs::srv::CoreCommand>> service,
                                          const std::shared_ptr<rmw_request_id_t> request_header,
                                          rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future)
{
    auto status = future.wait_for(std::chrono::seconds(1));
    if (status == std::future_status::ready)
    {
        auto reply = future.get()->reply;
        flight_stack_msgs::srv::CoreCommand::Response response;
        switch (reply.result)
        {
        case reply.VEHICLE_CMD_RESULT_ACCEPTED:
            RCLCPP_INFO(this->get_logger(), "command accepted");
            response.reply.result = response.reply.RESULT_SUCCESS;
            service->send_response(*request_header, response);
            return;
        case reply.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
            RCLCPP_WARN(this->get_logger(), "command temporarily rejected");
            break;
        case reply.VEHICLE_CMD_RESULT_DENIED:
            RCLCPP_WARN(this->get_logger(), "command denied");
            break;
        case reply.VEHICLE_CMD_RESULT_UNSUPPORTED:
            RCLCPP_WARN(this->get_logger(), "command unsupported");
            break;
        case reply.VEHICLE_CMD_RESULT_FAILED:
            RCLCPP_WARN(this->get_logger(), "command failed");
            break;
        case reply.VEHICLE_CMD_RESULT_IN_PROGRESS:
            RCLCPP_WARN(this->get_logger(), "command in progress");
            break;
        case reply.VEHICLE_CMD_RESULT_CANCELLED:
            RCLCPP_WARN(this->get_logger(), "command cancelled");
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "command reply unknown");
            break;
        }
        response.reply.result = response.reply.RESULT_FAILURE;
        service->send_response(*request_header, response);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
}

void FlightCore::core_command_callback(std::shared_ptr<rclcpp::Service<flight_stack_msgs::srv::CoreCommand>> service,
                                       const std::shared_ptr<rmw_request_id_t> request_header,
                                       const std::shared_ptr<flight_stack_msgs::srv::CoreCommand::Request> request_msg)
{
    auto request = request_msg->request;

    switch (request.command)
    {
    case request.ARM:
        if (is_simulation_ == false)
        {
            flight_stack_msgs::srv::CoreCommand::Response response;
            response.reply.result = response.reply.RESULT_FAILURE;
            service->send_response(*request_header, response);
            return;
        }
        arm(service, request_header);
        return;
    case request.DISARM:
        disarm(service, request_header);
        return;
    case request.MODE_OFFBOARD:
        mode(service, request_header, 6.0);
        return;
    case request.MODE_POSCTL:
        mode(service, request_header, 3.0);
        return;
    case request.MODE_ALTCTL:
        mode(service, request_header, 2.0);
        return;
    case request.MODE_LAND:
        land(service, request_header);
        return;
    case request.CORE_GOTO:
        core_mode_ = CoreMode::GOTO;
        break;
    case request.CORE_TRAJ:
        core_mode_ = CoreMode::TRAJ;
        break;
    case request.CORE_TRAJ_PATH:
        core_mode_ = CoreMode::TRAJ_PATH;
        break;
    case request.CORE_OVERRIDE:
        core_mode_ = CoreMode::OVERRIDE;
        break;
    default:
        flight_stack_msgs::srv::CoreCommand::Response response;
        response.reply.result = response.reply.RESULT_FAILURE;
        service->send_response(*request_header, response);
        return;
    }

    flight_stack_msgs::srv::CoreCommand::Response response;
    response.reply.result = response.reply.RESULT_SUCCESS;
    service->send_response(*request_header, response);
}

void FlightCore::arm()
{
    vehicle_command_request(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
}

void FlightCore::disarm()
{
    vehicle_command_request(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
}

void FlightCore::land()
{
    vehicle_command_request(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND, 1.0);
}

void FlightCore::mode(float mode)
{
    // Best explanation I could find here: https://discuss.px4.io/t/switching-modes-in-px4-using-ros2-and-uxrce-dds/37137
    // param1 = 1 for px4 custom modes
    // param2 = PX4_CUSTOM_MAIN_MODE -- https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/px4_custom_mode.h
    // 2 -- ALTCTL
    // 3 -- POSCTL
    // 6 -- OFFBOARD
    vehicle_command_request(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, mode);
}

void FlightCore::arm(std::shared_ptr<rclcpp::Service<flight_stack_msgs::srv::CoreCommand>> service,
                     const std::shared_ptr<rmw_request_id_t> request_header)
{
    vehicle_command_request(service, request_header, px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
}

void FlightCore::disarm(std::shared_ptr<rclcpp::Service<flight_stack_msgs::srv::CoreCommand>> service,
                        const std::shared_ptr<rmw_request_id_t> request_header)
{
    vehicle_command_request(service, request_header, px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
}

void FlightCore::land(std::shared_ptr<rclcpp::Service<flight_stack_msgs::srv::CoreCommand>> service,
                      const std::shared_ptr<rmw_request_id_t> request_header)
{
    vehicle_command_request(service, request_header, px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND, 1.0);
}

void FlightCore::mode(std::shared_ptr<rclcpp::Service<flight_stack_msgs::srv::CoreCommand>> service,
                      const std::shared_ptr<rmw_request_id_t> request_header,
                      float mode)
{
    vehicle_command_request(service, request_header, px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, mode);
}

void FlightCore::goto_setpoint_callback(px4_msgs::msg::GotoSetpoint::UniquePtr msg)
{
    goto_setpoint_ = std::move(msg);
}
void FlightCore::trajectory_setpoint_callback(px4_msgs::msg::TrajectorySetpoint::UniquePtr msg)
{
    trajectory_setpoint_ = std::move(msg);
}

void FlightCore::publish_status()
{
    flight_stack_msgs::msg::CoreStatus msg{};
    msg.core_mode = (uint8_t)core_mode_;
    msg.status = flight_stack_msgs::msg::CoreStatus::STATUS_NOMINAL;
    core_status_publisher_->publish(msg);
}

void FlightCore::control_timer_callback()
{
    publish_offboard_control_mode();
    publish_status();
    if (is_offboard_ == false)
    {
        // THIS IS VERY IMPORTANT NEVER PUBLISH ANY SETPOINTS IF NOT IN OFFBOARD
        // https://github.com/PX4/PX4-Autopilot/issues/22512
        // https://github.com/PX4/PX4-Autopilot/pull/22530
        return;
    }
    switch (core_mode_)
    {
    case CoreMode::GOTO:
    {
        if (goto_setpoint_)
        {
            px4_msgs::msg::GotoSetpoint temp = *goto_setpoint_;
            publish_goto_setpoint_raw(temp);
        }
        break;
    }
    case CoreMode::TRAJ:
    {
        if (trajectory_setpoint_)
        {
            px4_msgs::msg::TrajectorySetpoint temp = *trajectory_setpoint_;
            publish_trajectory_setpoint_raw(temp);
        }
        break;
    }
    case CoreMode::TRAJ_PATH:
        break;
    case CoreMode::OVERRIDE:
        break;
    case CoreMode::OUT_OF_BOUNDS:
        break;
    }
}

/*
Possible Autopilot States: (that we concern ourselves with)

CONNECTION STATUS    | AUTOPILOT STATES
Connected | GCS Link | Armed | Offboard | Flying |
0         | X        | X     | X        | X      | We have no idea what is happening right now (we may have last known drone state, autopilot failsafe most likely has kicked in if drone is alive)
1         | 0        | X     | X        | X      | We can maybe control the drone but the pilot cannot monitor our activities (autopilot failsafe most likely has kicked in)
1         | 1        | 0     | X        | X      | Drone is not armed we cannot do anything (transitions to other states are still important)
1         | 1        | 1     | 0        | 0      | Not in offboard, pilot/autopilot control from the ground
1         | 1        | 1     | 0        | 1      | Not in offboard, pilot/autopilot control in the air
1         | 1        | 1     | 1        | 0      | We have control on the ground
1         | 1        | 1     | 1        | 1      | We have control and are flying

flight_core start states:
Cold start - Discard all previous state that may exist (drone not necessarily grounded)
Warm start - Use previous state when starting (e.g. load saved home, etc.)

flight_manager start states:
Cold start  - Discard all previous state that may exist. Effectively run assuming no other flight manager has been run yet (do not assume the state of the drone, e.g. drone may already be flying)
Resume      - Use previous state when starting, assume same flight manager was running before (previous process was either killed or crashed)
Warm start  - Use previous state when starting, known (not same) flight manager was running before
*/
