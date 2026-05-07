#!/usr/bin/env python3
import flight_stack
print("Using flight_stack from:", flight_stack.__file__)
import math
import numpy as np
import time

import rclpy

from flight_stack.flight_stack import FlightPlanner
from flight_stack.pather import trajectory
from flight_stack.btree import manager, controls, decorators, actions, utils

from flight_stack_msgs.srv import CoreCommand
from std_msgs.msg import Float32
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition, BatteryStatus, TrajectorySetpoint
from rclpy.qos import QoSPresetProfiles

hyperparameters = [
    {}
]

class MinJerkTraj(utils.BTNode):
    def __init__(self, name, fp:FlightPlanner, traj:trajectory.Trajectory, target_vel:float, forecast_time:float):
        super().__init__(name)
        self.fp = fp
        self.traj = traj
        self.pathtime = 0
        self.duration = 0
        self.traj_sp = TrajectorySetpoint()

        # parameters
        self.T = forecast_time
        self.target_vel = target_vel
        self.horizon = self.T * self.target_vel
        self.resolution = 0.5
        self.project_ahead = 0.1
        self.constants = []
        self.powers = []

        # slew rate limiter variables
        self.last_target_velocity = 0
        self.max_acceleration = 3.0      # m/s^2 

    def initialize(self):
        super().initialize()
        self.duration = self.traj.duration
        traj_iter = self.traj
        while traj_iter.next != None:
            traj_iter = traj_iter.next
            self.duration += traj_iter.duration
        
        self.constants = [
            3 * np.array([20/self.T, -8, -12]) / (2*self.T**2),
            4 * np.array([-30/self.T, 14, 16]) / (2*self.T**3),
            5 * np.array([12/self.T, -6, -6]) / (2*self.T**4),
        ]
        self.powers = [[(-x)**i for i in range(2, 5)] for x in np.arange(-self.T, 0, self.resolution/self.target_vel)]

    def reset(self):
        super().reset()
        self.pathtime = 0

    def projection(self) -> bool:
        closest = self.pathtime
        closest_dist = 1e6
        cur_pos = np.array([self.fp._position.x, self.fp._position.y, self.fp._position.z])
        # binary search instead of linear intrerpol?
        for t in np.arange(max(self.pathtime - 5, 0), min(self.pathtime + 5, self.duration), 0.1):
            pos = self.traj.path(t)
            dist = np.linalg.norm(cur_pos - pos)
            if dist < closest_dist:
                closest_dist = dist
                closest = t
        return closest + self.project_ahead


    def velocity_scale(self) -> float:
        """
        Min Jerk Position interpolation polynomial:
        c0 = p0
        c1 = v0
        c2 = 0.5 a0
        c3 = (20(pf - p0) - T(8vf + 12v0) - T^2(3a0 - af)) / 2 T^3
        c4 = (-30(pf - p0) - T(14vf + 16v0) + T^2(3a0 - 2af)) / 2 T^4
        c5 = (12(pf - p0) - 6T(vf + v0) - T^2(a0 - af)) / 2 T^5

        assumptions:
        a = 0
        trajectory has unit velocity

        Returns:
            float: 0 to target_vel (sometimes very slightly over)
        """

        sum_v_x = 0
        sum_v_y = 0
        for i, dt in enumerate(np.arange(-self.horizon + self.project_ahead, self.project_ahead, self.resolution)):
            pos0 = self.traj.path(self.pathtime + dt)
            pos1 = self.traj.path(self.pathtime + dt + self.horizon)
            vel0 = self.traj.velocity(self.pathtime + dt)
            vel1 = self.traj.velocity(self.pathtime + dt + self.horizon)

            dpx = pos1[0] - pos0[0]
            vx0 = vel0[0]
            vx1 = vel1[0]

            dpy = pos1[1] - pos0[1]
            vy0 = vel0[1]
            vy1 = vel1[1]

            sum_v_x += vx0 + np.dot(np.matmul(self.constants, [dpx, vx1, vx0]), self.powers[i])
            sum_v_y += vy0 + np.dot(np.matmul(self.constants, [dpy, vy1, vy0]), self.powers[i])
        #return sum_v_x/len(self.powers), sum_v_y/len(self.powers)
        return (sum_v_x**2 + sum_v_y**2) ** 0.5 / len(self.powers)
    
    def tick(self):
        # Action based, tries to clock as fast as btree
        # How to determine failure? built in time out?
        if self.pathtime > self.duration - self.project_ahead:
            self.status = utils.STATUS.SUCCESS
            return self.status

        self.pathtime = self.projection()
        self.target_spd = self.velocity_scale()
        print(self.target_spd)

        '''# slew rate limiter to velocity output
        max_delta = self.max_acceleration * self.inner_dt
        requested_delta = self.target_velocity - self.last_target_velocity
        clamped_delta = max(-max_delta, min(requested_delta, max_delta))
        self.target_velocity = self.last_target_velocity + clamped_delta
        self.last_target_velocity = self.target_velocity'''

        self.traj_sp.position = list(self.traj.path(self.pathtime))
        self.traj_sp.velocity = list(self.traj.velocity(self.pathtime) * self.target_spd)
        #vx, vy = self.velocity_scale()
        #print(vx, vy)
        #self.traj_sp.velocity = [vx, vy, 0.0]
        self.fp._traj_publisher.publish(self.traj_sp)
        return self.status