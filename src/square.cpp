/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

enum FlightState {
	TAKEOFF,
	CORNER0,
	CORNER1,
	CORNER2,
	CORNER3,
	LAND
};

#define SLEN 10
#define HOV -5
#define CORNER_TIME 10

class OffboardControl : public rclcpp::Node
{
public:
	explicit OffboardControl() : Node("hover")
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		subscription_ = this->create_subscription<px4_msgs::msg::VehicleStatus>("/fmu/out/vehicle_status_v1", qos,
		[this](const px4_msgs::msg::VehicleStatus::UniquePtr msg) {
			if (msg->arming_state == 1)
			{
				armed_ = false;
			}
			else if (msg->arming_state == 2)
			{
				armed_ = true;
			}
		});

		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

		offboard_setpoint_counter_ = 0;

		auto timer_callback = [this]() -> void {
			if (!armed_) {
				if (!wait_arm_latch_) {
					wait_arm_latch_ = true;
					RCLCPP_INFO(this->get_logger(), "Waiting for arm");
				}
				offboard_setpoint_counter_ = 0;
				start_time_ = this->get_clock()->now();
				return;
			}
			if (armed_ and !arming_latch_) {
				arming_latch_ = true;
				RCLCPP_INFO(this->get_logger(), "Drone armed");
			}

			if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
			}

			// stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			}

			rclcpp::Time now = this->get_clock()->now();
			rclcpp::Duration elapsed = now - start_time_;

			switch (flight_state_)
			{
			case FlightState::TAKEOFF:
				if (elapsed.seconds() < 8.0) {
					publish_offboard_control_mode();
					publish_trajectory_setpoint(0, 0, HOV);
				} else {
					start_time_ = this->get_clock()->now();
					flight_state_ = FlightState::CORNER0;
				}
				break;
			case FlightState::CORNER0:
				if (elapsed.seconds() < CORNER_TIME) {
					publish_offboard_control_mode();
					publish_trajectory_setpoint(-SLEN, 0, HOV);
				} else {
					start_time_ = this->get_clock()->now();
					flight_state_ = FlightState::CORNER1;
				}
				break;
			case FlightState::CORNER1:
				if (elapsed.seconds() < CORNER_TIME) {
					publish_offboard_control_mode();
					publish_trajectory_setpoint(-SLEN, -SLEN, HOV);
				} else {
					start_time_ = this->get_clock()->now();
					flight_state_ = FlightState::CORNER2;
				}
				break;
			case FlightState::CORNER2:
				if (elapsed.seconds() < CORNER_TIME) {
					publish_offboard_control_mode();
					publish_trajectory_setpoint(0, -SLEN, HOV);
				} else {
					start_time_ = this->get_clock()->now();
					flight_state_ = FlightState::CORNER3;
				}
				break;
			case FlightState::CORNER3:
				if (elapsed.seconds() < CORNER_TIME) {
					publish_offboard_control_mode();
					publish_trajectory_setpoint(0, 0, HOV);
				} else {
					start_time_ = this->get_clock()->now();
					flight_state_ = FlightState::LAND;
				}
				break;
			case FlightState::LAND:
				if (elapsed.seconds() < CORNER_TIME) {
					publish_offboard_control_mode();
					publish_trajectory_setpoint(0, 0, HOV);
				} else {
					land();
					rclcpp::shutdown();
				}
				break;
			default:
				break;
			}
		};
		timer_ = this->create_wall_timer(100ms, timer_callback);
	}

	void arm();
	void disarm();
	void land();

private:
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr subscription_;
	bool armed_ {false};
	bool wait_arm_latch_ {false};
	bool arming_latch_ {false};
	enum FlightState flight_state_ { FlightState::TAKEOFF };

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent
	rclcpp::Time start_time_;

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint(float x, float y, float z);
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
};

void OffboardControl::land()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND, 1.0);

	RCLCPP_INFO(this->get_logger(), "Land command send");
}

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint(float x, float y, float z)
{
	TrajectorySetpoint msg{};
	msg.position = {x, y, z};
	msg.yaw = NAN;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}
