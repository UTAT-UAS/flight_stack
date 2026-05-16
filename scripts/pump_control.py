#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from std_msgs.msg import Int32, Float32, Bool
from px4_msgs.msg import ActuatorServos, ManualControlSetpoint, VehicleCommand
from std_srvs.srv import SetBool, Empty
import time


class PumpControl(Node):
    def __init__(self):
        super().__init__("pump_control")

        self.declare_parameter("debug", False)
        self.declare_parameter("pump_index", 1)
        self.declare_parameter("servo_index", 0)
        self.declare_parameter("servo_max_angle", 32.0)
        self.declare_parameter("servo_max_jiggle_deg", 4.0)  # 4deg above and below
        self.declare_parameter("servo_jiggle_freq", 3.0)  # 3hz
        self.declare_parameter("pump_on_throttle", 1.0)  # mapping 1.0 to full forward
        self.declare_parameter(
            "pump_off_throttle", -1.0
        )  # mapping -1.0 to full disarm/zero

        self.debug = self.get_parameter("debug").value
        self.pump_idx = self.get_parameter("pump_index").value
        self.servo_idx = self.get_parameter("servo_index").value
        self.max_angle = self.get_parameter("servo_max_angle").value
        self.max_jiggle = self.get_parameter("servo_max_jiggle_deg").value / self.max_angle
        self.jiggle_freq = self.get_parameter("servo_jiggle_freq").value
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

        self.reset_srv = self.create_service(
            Empty, "/uas/pump/reset_timers", self.reset_timers_callback
        )

        self.time_pub = self.create_publisher(Float32, "/uas/pump/time", 10)
        self.time_primed_pub = self.create_publisher(Float32, "/uas/pump/time_primed", 10)

        self.manual_sub = self.create_subscription(
            ManualControlSetpoint, "/fmu/out/manual_control_setpoint", self.manual_input_callback, QoSPresetProfiles.SENSOR_DATA.value,
        )

        self.pass_through_enabled = False

        self.total_pump_time = 0.0
        self.total_pump_time_primed = 0.0
        self.prime_time = 0.3
        self.is_firing = False
        self.firing_start_time = 0.0
        self.manual_firing = False
        self.auto_firing = False

        if self.debug:
            self.manual_pub = self.create_publisher(
                ManualControlSetpoint, "/fmu/in/manual_control_input", 10
            )

        self.pump_timer = None
        self.pump_off_time = 0.0

        self.current_servo_val = 0.0
        self.jiggle_servo_val = 0.0
        self.current_pump_val = self.pump_off_val

        # publisher loop
        self.control_rate = 0.05
        self.control_timer = self.create_timer(self.control_rate, self.publish_controls)  # 20 Hz

        self.jiggle_servo_rate = 4 * self.max_jiggle * self.jiggle_freq * self.control_rate

    def pump_callback(self, msg):
        time_ms = msg.data
        if time_ms == 0:
            self.current_pump_val = self.pump_off_val
            self.pump_off_time = 0.0
        else:
            self.current_pump_val = self.pump_on_val
            self.jiggle_servo_val = 0
            self.pump_off_time = (
                self.get_clock().now().nanoseconds / 1e9 + time_ms / 1000.0
            )

    def servo_callback(self, msg):
        angle = msg.data
        angle = max(min(angle, self.max_angle), -self.max_angle)
        self.current_servo_val = angle / self.max_angle

    def reset_timers_callback(self, request, response):
        self.total_pump_time = 0.0
        self.total_pump_time_primed = 0.0
        now = self.get_clock().now().nanoseconds / 1e9
        if self.is_firing:
            self.firing_start_time = now
        return response

    def manual_input_callback(self, msg: ManualControlSetpoint):
        if self.pass_through_enabled:
            self.manual_firing = (msg.aux4 >= 0.8)
            self.current_servo_val = msg.aux3
        else:
            self.manual_firing = False

    def toggle_peripheral_callback(self, request, response):
        self.pass_through_enabled = request.data
        if request.data:
            self.get_logger().info("Enabled RC AUX pass-through mode.")
            response.message = "Enabled RC AUX pass-through mode"
        else:
            self.get_logger().info("Disabled RC AUX pass-through mode.")
            response.message = "Disabled RC AUX pass-through mode"
        response.success = True
        return response

    def publish_controls(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.pump_off_time > 0 and now >= self.pump_off_time:
            self.current_pump_val = self.pump_off_val
            self.pump_off_time = 0.0
            self.jiggle_servo_val = 0.0

        self.auto_firing = (self.current_pump_val == self.pump_on_val)
        currently_firing = self.manual_firing or self.auto_firing

        if currently_firing and not self.is_firing:
            self.is_firing = True
            self.firing_start_time = now
        elif not currently_firing and self.is_firing:
            self.is_firing = False
            duration = now - self.firing_start_time
            self.total_pump_time += duration
            self.total_pump_time_primed += max(0.0, duration - self.prime_time)
            
        current_firing_time = 0.0
        current_firing_time_primed = 0.0
        if self.is_firing:
            current_firing_time = now - self.firing_start_time
            current_firing_time_primed = max(0.0, current_firing_time - self.prime_time)
            if not self.pass_through_enabled:
                self.jiggle_servo_val += self.jiggle_servo_rate
                self.jiggle_servo_rate = -self.jiggle_servo_rate if abs(self.jiggle_servo_val) >= self.max_jiggle else self.jiggle_servo_rate
            else:
                self.jiggle_servo_val = 0.0

        msg_time = Float32()
        msg_time.data = float(self.total_pump_time + current_firing_time)
        self.time_pub.publish(msg_time)

        msg_primed = Float32()
        msg_primed.data = float(self.total_pump_time_primed + current_firing_time_primed)
        self.time_primed_pub.publish(msg_primed)

        msg = VehicleCommand()
        msg.timestamp = int(now * 1e6)
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_ACTUATOR
        # Using 187 (or user suggested 180, but constant maps correctly to PX4 spec)
        # Actuator indices are typically 1-based mapping to param1..param6
        params = [float("nan")] * 6
        if 0 <= self.pump_idx < 6:
            params[self.pump_idx] = float(self.current_pump_val)
        if 0 <= self.servo_idx < 6:
            params[self.servo_idx] = min(max(float(self.current_servo_val + self.jiggle_servo_val), -1.0), 1.0)

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
