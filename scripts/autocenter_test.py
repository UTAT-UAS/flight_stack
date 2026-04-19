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
    """
    Example code, yaws to heading=2rad
    """
    def __init__(self):
        super().__init__()

        self.reset = False
        self.requested = False
        self.set_height = -5

        self.btree = manager.BehaviorTree("square_tree")
        self.btree.setroot(
            controls.Sequence(
                name="root",
                children=[
                    decorators.Timeout(
                        name="offboard_timeout",
                        timeout=5,
                        child=actions.SetOffboard(
                            name="set_offboard",
                            fp=self
                        )
                    ),
                    actions.SetTrajMode(
                        name="set_traj",
                        fp=self
                    ),
                    actions.AutoCenterTraj(
                        name="auto_center",
                        fp=self,
                        child=None,
                        k=-0.005,
                        floor_tol=10,
                        max_rate=0.5,
                    )
                ]
            )
        )
        self.btree.setup()
        time.sleep(1)  # wait for setup to complete
        self.btree.initialize()
        self.target_yaw = 2
        print(self.target_yaw)

    def main_loop(self):
        # Example, to be replaced
        self.btree.blackboard["target_dx"] = -(self.target_yaw - self._position.heading) * 100

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
