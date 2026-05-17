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
from std_msgs.msg import Bool

from px4_msgs.msg import ActuatorMotors
from px4_msgs.msg import VehicleLocalPosition


class LapControl(Node):
    def __init__(self):
        super().__init__("current_sim")

        self._stop_laps_publisher = self.create_publisher(Bool, "task1/stop_laps", 100)

    def _publish_stop_laps(self, val) -> None:
        # publish stop laps
        stop_laps_msg = Bool()
        stop_laps_msg.data = True  # or some calculated value
        self._stop_laps_publisher.publish(stop_laps_msg)


def main(args=None):
    rclpy.init(args=args)

    node = LapControl()

    try:
        while rclpy.ok():
            stop_laps = input("Stop laps? (s to stop, c to continue): ")
            if stop_laps == "s":
                node._publish_stop_laps(True)
            elif stop_laps == "c":
                node._publish_stop_laps(False)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
