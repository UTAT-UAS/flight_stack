#!/usr/bin/env python3

import rclpy

from flight_stack.flight_stack import FlightPlanner

from flight_stack_msgs.srv import CoreCommand
from px4_msgs.msg import GotoSetpoint, VehicleStatus

class TestPublisher(FlightPlanner):
    def __init__(self):
        super().__init__()
        self.test_timer = self.create_timer(1.0, self.test_publish)

    def test_publish(self):
        print("Publishing test GotoSetpoint message")
        goto = GotoSetpoint()
        goto.position = [0.0, 0.0, -5.0]
        self._goto_publisher.publish(goto)

        if self._status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            print("Requesting Offboard")
            command = CoreCommand.Request()
            command.request.command = 2
            self._core_command_client.call_async(command)


def main(args=None):
    # Testing script

    rclpy.init(args=args)
    minimal_publisher = TestPublisher()
    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
