#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from px4_msgs.msg import ActuatorServos, ManualControlSetpoint, VehicleCommand
from std_srvs.srv import SetBool
import time


class PumpControl(Node):
    def __init__(self):
        super().__init__("pump_control")

        self.declare_parameter("debug", False)
        self.declare_parameter("pump_index", 1)
        self.declare_parameter("servo_index", 0)
        self.declare_parameter("servo_max_angle", 32.0)
        self.declare_parameter("pump_on_throttle", 1.0)  # mapping 1.0 to full forward
        self.declare_parameter(
            "pump_off_throttle", -1.0
        )  # mapping -1.0 to full disarm/zero

        self.debug = self.get_parameter("debug").value
        self.pump_idx = self.get_parameter("pump_index").value
        self.servo_idx = self.get_parameter("servo_index").value
        self.max_angle = self.get_parameter("servo_max_angle").value
        self.pump_on_val = self.get_parameter("pump_on_throttle").value
        self.pump_off_val = self.get_parameter("pump_off_throttle").value

        self.pump_sub = self.create_subscription(
            Int32, "/set_pump_time", self.pump_callback, 10
        )
        self.servo_sub = self.create_subscription(
            Int32, "/set_servo_angle", self.servo_callback, 10
        )

        # Publisher for VehicleCommand (DO_SET_ACTUATOR)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", 10
        )

        # Service to toggle parameter routing
        self.toggle_srv = self.create_service(
            SetBool, "/toggle_peripheral_mode", self.toggle_peripheral_callback
        )

        if self.debug:
            self.manual_pub = self.create_publisher(
                ManualControlSetpoint, "/fmu/in/manual_control_input", 10
            )

        self.pump_timer = None
        self.pump_off_time = 0.0

        self.current_servo_val = 0.0
        self.current_pump_val = self.pump_off_val

        # publisher loop
        self.control_timer = self.create_timer(0.05, self.publish_controls)  # 20 Hz

    def pump_callback(self, msg):
        time_ms = msg.data
        if time_ms == 0:
            self.current_pump_val = self.pump_off_val
            self.pump_off_time = 0.0
        else:
            self.current_pump_val = self.pump_on_val
            self.pump_off_time = (
                self.get_clock().now().nanoseconds / 1e9 + time_ms / 1000.0
            )

    def servo_callback(self, msg):
        angle = msg.data
        angle = max(min(angle, self.max_angle), -self.max_angle)
        self.current_servo_val = angle / self.max_angle

    def toggle_peripheral_callback(self, request, response):
        cmd1 = VehicleCommand()
        cmd1.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        cmd1.command = VehicleCommand.VEHICLE_CMD_DO_SET_PARAMETER
        cmd1.target_system = 1
        cmd1.target_component = 1
        cmd1.source_system = 1
        cmd1.source_component = 1
        cmd1.from_external = True

        cmd2 = VehicleCommand()
        cmd2.timestamp = cmd1.timestamp
        cmd2.command = VehicleCommand.VEHICLE_CMD_DO_SET_PARAMETER
        cmd2.target_system = 1
        cmd2.target_component = 1
        cmd2.source_system = 1
        cmd2.source_component = 1
        cmd2.from_external = True

        if request.data:
            # Switch to Peripheral via Actuator Set
            cmd1.param1 = 1134.0 # PWM_MAIN_FUNC1 index
            cmd1.param2 = 301.0 # Peripheral via Actuator Set 1
            
            cmd2.param1 = 1136.0 # PWM_MAIN_FUNC3 index
            cmd2.param2 = 302.0 # Peripheral via Actuator Set 2
            
            self.get_logger().info("Toggling PWM_MAIN_FUNC1 to 301 and PWM_MAIN_FUNC3 to 302.")
            self.vehicle_command_pub.publish(cmd1)
            self.vehicle_command_pub.publish(cmd2)
            response.success = True
            response.message = "Enabled peripheral (Actuator) mode"
        else:
            # Switch back to RC AUX
            cmd1.param1 = 1134.0 # PWM_MAIN_FUNC1 index
            cmd1.param2 = 409.0 # RC AUX 3
            
            cmd2.param1 = 1136.0 # PWM_MAIN_FUNC3 index
            cmd2.param2 = 410.0 # RC AUX 4
            
            self.get_logger().info("Toggling PWM_MAIN_FUNC1 to 409 and PWM_MAIN_FUNC3 to 410.")
            self.vehicle_command_pub.publish(cmd1)
            self.vehicle_command_pub.publish(cmd2)
            response.success = True
            response.message = "Enabled RC AUX mode"

        return response

    def publish_controls(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.pump_off_time > 0 and now >= self.pump_off_time:
            self.current_pump_val = self.pump_off_val
            self.pump_off_time = 0.0

        msg = VehicleCommand()
        msg.timestamp = int(now * 1e6)
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_ACTUATOR
        # Using 187 (or user suggested 180, but constant maps correctly to PX4 spec)
        # Actuator indices are typically 1-based mapping to param1..param6
        params = [float("nan")] * 6
        if 0 <= self.pump_idx < 6:
            params[self.pump_idx] = float(self.current_pump_val)
        if 0 <= self.servo_idx < 6:
            params[self.servo_idx] = float(self.current_servo_val)

        msg.param1 = params[0]
        msg.param2 = params[1]
        msg.param3 = params[2]
        msg.param4 = params[3]
        msg.param5 = params[4]
        msg.param6 = params[5]
        msg.param7 = 0.0 # index
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        self.vehicle_command_pub.publish(msg)

        if self.debug:
            man_msg = ManualControlSetpoint()
            man_msg.timestamp = msg.timestamp
            man_msg.timestamp_sample = msg.timestamp
            man_msg.valid = False
            man_msg.data_source = ManualControlSetpoint.SOURCE_RC
            man_msg.aux3 = self.current_servo_val
            self.manual_pub.publish(man_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PumpControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
