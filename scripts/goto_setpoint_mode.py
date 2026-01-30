#!/usr/bin/env python3

import math
import time
import numpy as np

import rclpy

from flight_stack.flight_stack import FlightPlanner

from flight_stack_msgs.srv import CoreCommand
from geometry_msgs.msg import Point
from px4_msgs.msg import GotoSetpoint, VehicleStatus

from pather import trajectory

class LandingPadStack(FlightPlanner):
    def __init__(self):
        super().__init__()
        self.point = Point()

        self.reset = False
        self.requested = False
        self.set_height = -5
        self.time = time.time()
        self.start_time = -1
        self.cumulative_error = 0.0
        self.period = 0.2
        self.traj = trajectory.AmongusHigherRes(np.array([self._position.x, self._position.y, self._position.z]), scale=40)
        self.pathtime = 0
        self.goto = GotoSetpoint()

    def main_loop(self):
        if self.reset is False:
            if time.time() - self.time < 2:  # lazy
                return
            
            self.traj = trajectory.AmongusHigherRes(np.array([self._position.x, self._position.y, self._position.z]), scale=40)
            self.goto.position = [self._position.x, self._position.y, self._position.z]
            print(self.goto.position)
            self._goto_publisher.publish(self.goto)
            self.reset = True
            
            print("CoreMode -> GOTO")
            command = CoreCommand.Request()
            command.request.command = 6 # CORE_GOTO request command
            self._core_command_client.call_async(command)

            return
        if self._status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            if self.requested is False:
                print("offboard")
                command = CoreCommand.Request()
                command.request.command = 2
                self._core_command_client.call_async(command)
                self.requested = True
        if self.pathtime > self.traj.duration - 1:
            # Benchmark stats
            flight_time = time.time() - self.start_time
            print("Duration: ", flight_time)
            print("Cumulative error: ", self.cumulative_error)
            print("Average error: ", self.cumulative_error / flight_time)

            print("landing")
            command = CoreCommand.Request()
            command.request.command = 5
            self._core_command_client.call_async(command)
            exit()
        if time.time() - self.time > self.period:
            if(self.start_time == -1):
                self.start_time = time.time()
            
            self.pathtime = self.projection()
            self.goto.position = list(self.traj.path(self.pathtime))
            self._goto_publisher.publish(self.goto)
            self.time = time.time()

    def projection(self) -> bool:
        closest = self.pathtime
        closest_dist = 1e6
        cur_pos = np.array([self._position.x, self._position.y, self._position.z])
        for t in np.arange(max(self.pathtime - 5, 0), min(self.pathtime + 5, self.traj.duration), 0.1):
            pos = self.traj.path(t)
            dist = np.linalg.norm(cur_pos - pos)
            if dist < closest_dist:
                closest_dist = dist
                closest = t
        self.cumulative_error += closest_dist * self.period
        return closest + 2


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
