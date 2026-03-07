#!/usr/bin/env python3
import flight_stack
print("Using flight_stack from:", flight_stack.__file__)
import math
import time

import rclpy

from flight_stack.flight_stack import FlightPlanner
from flight_stack.pather import trajectory
from flight_stack.btree import manager, controls, decorators, actions

from flight_stack_msgs.srv import CoreCommand
from geometry_msgs.msg import Point
from px4_msgs.msg import GotoSetpoint, VehicleStatus


class BTreeFlightPlanner(FlightPlanner):
    def __init__(self):
        super().__init__()
        self.point = Point()

        self.reset = False
        self.requested = False
        self.set_height = -5
        self.time = time.time()
        self.waypoints = []
        self.wpi = 0

        self.btree = manager.BehaviorTree("square_tree")
        self.btree.setroot(
            controls.Sequence(
                name="root",
                children=[
                    decorators.Timeout(
                        name="offboard_timeout",
                        timeout=3,
                        child=actions.SetOffboard(
                            name="set_offboard",
                            fp=self
                        )
                    ),
                    actions.SetGotoMode(
                        name="set_goto",
                        fp=self
                    ),
                    actions.Goto(
                        name="square",
                        fp=self,
                        points=self.waypoints
                    ),
                    actions.Land(
                        name="land",
                        fp=self
                    )
                ]
            )
        )
        self.btree.setup()
        time.sleep(1)  # wait for setup to complete
        self.btree.initialize()

    def main_loop(self):
        if time.time() - self.time < 0.1:  # 10Hz
            return
        self.time = time.time()

        if self.waypoints == [] and self._position.z != 0:
            # Initialize waypoints
            self.waypoints.append([self._position.x, self._position.y, self._position.z])
            self.waypoints.append([self._position.x + 5, self._position.y, self._position.z])
            self.waypoints.append([self._position.x + 5, self._position.y + 5, self._position.z])
            self.waypoints.append([self._position.x, self._position.y + 5, self._position.z])
            self.waypoints.append([self._position.x, self._position.y, self._position.z])

        self.btree.tick()
        if self.btree.status == manager.STATUS.SUCCESS:
            print("mission complete")
            exit()
        elif self.btree.status == manager.STATUS.FAILURE:
            print("mission failed")
            exit()


def main(args=None):
    rclpy.init(args=args)

    btree_fp = BTreeFlightPlanner()

    rclpy.spin(btree_fp)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    btree_fp.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
