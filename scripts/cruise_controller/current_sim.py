#!/usr/bin/env python3

# simulates current draw for simulation testing
# takes in motor outputs from px4 and maps it to a current draw topic using a model in the datasheet 
# just for testing the cruise current controller
# this should be started after the drone starts hovering, the model does not fair well at 0 throttle

# need to add   - topic: /fmu/out/actuator_motors
#    type: px4_msgs::msg::ActuatorMotors
# to /build/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml

import math
import rclpy
from rclpy.qos import QoSPresetProfiles
from rclpy.node import Node
from std_msgs.msg import Float32

from px4_msgs.msg import ActuatorMotors
from px4_msgs.msg import VehicleLocalPosition


class CurrentSim(Node):
    def __init__(self):
        super().__init__("current_sim")

        self._current_publisher = self.create_publisher(Float32, "sim/current_draw", 100)
        self._mah_publisher = self.create_publisher(Float32, "sim/discharged_mah", 100)
        self._motor_outputs = None
        self._v_mag = 0.0
        self._timer = self.create_timer(0.01, self._publish_current_draw)

        # coulomb counting vars
        self._discharged_mah = 0.0
        self._last_time = self.get_clock().now()

        self._actuator_motors_subscriber = self.create_subscription(
            ActuatorMotors,
            "/fmu/out/actuator_motors",
            self._actuator_motors_cb,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

        self._velocity_subscriber = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self._velocity_cb,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

    def _actuator_motors_cb(self, msg: ActuatorMotors) -> None:
        self._motor_outputs = list(msg.control[0:4])

    def _velocity_cb(self, msg: VehicleLocalPosition) -> None:
        self._v_mag = math.sqrt(msg.vx**2 + msg.vy**2)

    def _estimate_current_draw(self) -> float:
        if not self._motor_outputs:
            return 0.0

        # current draw model for 6 motors
        x = [max(0.0, (motor - 0.22) * 100) for motor in self._motor_outputs]
        
        i_motor = [
            1.996e-5 * value**3 + 1.44e-3 * value**2 - (1.64e-2 * value)
            for value in x
        ]
        i_motors = sum(i_motor) + 0.5  # add idle current

        # apply velocity correction factor since gazebo does not simulate this
        i_del = -0.1 * self._v_mag + 0.003 * self._v_mag**3
        i_model = i_motors + i_del

        return abs(i_model)

    def _publish_current_draw(self) -> None:
        current_amps = self._estimate_current_draw()

        # publish current draw
        current_msg = Float32()
        current_msg.data = current_amps
        self._current_publisher.publish(current_msg)

        # publish discharged mAh for coulomb counting
        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds / 1e9
        self._last_time = now
        self._discharged_mah += (current_amps * dt) / 3.6

        mah_msg = Float32()
        mah_msg.data = self._discharged_mah
        self._mah_publisher.publish(mah_msg)



def main(args=None):
    rclpy.init(args=args)

    node = CurrentSim()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
