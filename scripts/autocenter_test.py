#!/usr/bin/env python3
import flight_stack
print("Using flight_stack from:", flight_stack.__file__)
import math
import time

import rclpy

from flight_stack.flight_stack import FlightPlanner
from flight_stack.pather import trajectory
from flight_stack.btree import manager, controls, decorators, actions, utils

from flight_stack_msgs.srv import CoreCommand
from geometry_msgs.msg import Point
from px4_msgs.msg import GotoSetpoint, VehicleStatus
from std_msgs.msg import Float32
from rclpy.qos import QoSPresetProfiles

class BTreeFlightPlanner(FlightPlanner):
    """
    Example code, autocenters yaw for 20s
    """
    def __init__(self):
        super().__init__()

        self.xerror_subscriber = self.create_subscription(
            Float32,
            "/uas/cv/x_error",
            self.xerror_cb,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

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
                        child=utils.BTNode("indefinite"),
                        k=0.0005,
                        floor_tol=10,
                        max_rate=0.2,
                    )
                ]
            )
        )
        self.btree.setup()
        time.sleep(1)  # wait for setup to complete
        self.btree.initialize()

    def xerror_cb(self, msg):
        self.btree.blackboard["target_dx"] = msg.data

    def main_loop(self):
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
