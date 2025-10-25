#!/usr/bin/env python3

import math
import time

import rclpy

from flight_stack.flight_stack import FlightPlanner

from flight_stack_msgs.srv import CoreCommand
from geometry_msgs.msg import Point
from px4_msgs.msg import GotoSetpoint, VehicleStatus


class LandingPadStack(FlightPlanner):
    def __init__(self):
        super().__init__()
        self.create_subscription(Point, "/uas/cv/position", self._point_cb, 5)
        self.point = Point()
        self.detection = False

        self.reset = False
        self.requested = False
        self.set_height = -5
        self.time = time.time()

    def main_loop(self):
        if self.reset is False:
            if time.time() - self.time < 2:  # lazy
                return
            goto = GotoSetpoint()
            goto.position = [self._position.x, self._position.y, self._position.z]
            print(goto.position)
            self._goto_publisher.publish(goto)
            self.reset = True
            return
        if self.detection is False:
            return
        if self._status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            if self.requested is False:
                print("offboard")
                command = CoreCommand.Request()
                command.request.command = 2
                self._core_command_client.call_async(command)
                self.requested = True
        if self.set_height == 0:
            print("landing")
            command = CoreCommand.Request()
            command.request.command = 5
            self._core_command_client.call_async(command)
            exit()
        if self.is_veritcalish() and time.time() - self.time > 8:
            print("descend")
            goto = GotoSetpoint()
            goto.position = [self.point.x, self.point.y, float(self.set_height)]
            print(goto.position)
            self._goto_publisher.publish(goto)
            self.set_height += 1
            self.time = time.time()

    def is_veritcalish(self) -> bool:
        rel_tol = 1e-10
        abs_tol = 0.0018
        return math.isclose(
            self._attitude.q[1], 0, rel_tol=rel_tol, abs_tol=abs_tol
        ) and math.isclose(self._attitude.q[2], 0, rel_tol=rel_tol, abs_tol=abs_tol)

    def _point_cb(self, msg: Point) -> None:
        self.detection = True
        self.point = msg


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = LandingPadStack()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
