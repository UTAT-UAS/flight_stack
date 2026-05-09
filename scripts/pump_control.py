#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from px4_msgs.msg import ActuatorServos, ManualControlSetpoint
import time


class PumpControl(Node):
    def __init__(self):
        super().__init__("pump_control")

        self.declare_parameter("debug", False)
        self.declare_parameter("pump_index", 2)
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

        self.actuator_pub = self.create_publisher(
            ActuatorServos, "/fmu/in/actuator_servos", 10
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

    def publish_controls(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.pump_off_time > 0 and now >= self.pump_off_time:
            self.current_pump_val = self.pump_off_val
            self.pump_off_time = 0.0

        msg = ActuatorServos()
        msg.timestamp = int(now * 1e6)
        msg.timestamp_sample = msg.timestamp
        msg.control = [float("nan")] * 8
        msg.control[self.pump_idx] = self.current_pump_val
        msg.control[self.servo_idx] = self.current_servo_val

        self.actuator_pub.publish(msg)

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
