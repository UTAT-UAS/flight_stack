import collections
import math
import time
import numpy as np

from flight_stack.flight_stack import FlightPlanner
from flight_stack.pather import trajectory

from flight_stack_msgs.srv import CoreCommand
from px4_msgs.msg import GotoSetpoint, VehicleStatus, TrajectorySetpoint, VehicleAttitudeSetpoint

from .utils import BTNode, STATUS

class SetOffboard(BTNode):
    def __init__(self, name, fp:FlightPlanner):
        super().__init__(name)
        self.fp = fp

    def tick(self):
        if self.fp._status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            print("requesting offboard")
            command = CoreCommand.Request()
            command.request.command = 2
            self.fp._core_command_client.call_async(command)
        else: self.status = STATUS.SUCCESS
        return self.status


class SetGotoMode(BTNode):
    def __init__(self, name, fp:FlightPlanner):
        super().__init__(name)
        self.fp = fp

    def tick(self):
        print("CoreMode -> GOTO request sent")
        command = CoreCommand.Request()
        command.request.command = 6 # CORE_GOTO request command
        self.fp._core_command_client.call_async(command)
        self.status = STATUS.SUCCESS
        return self.status


class SetTrajMode(BTNode):
    def __init__(self, name, fp:FlightPlanner):
        super().__init__(name)
        self.fp = fp

    def tick(self):
        print("CoreMode -> TRAJ request sent")
        command = CoreCommand.Request()
        command.request.command = 7 # CORE_TRAJ request command
        self.fp._core_command_client.call_async(command)
        self.status = STATUS.SUCCESS
        return self.status
    

class Land(BTNode):
    def __init__(self, name, fp:FlightPlanner):
        super().__init__(name)
        self.fp = fp

    def tick(self):
        # send land command
        print("land request sent")
        command = CoreCommand.Request()
        command.request.command = 5
        self.fp._core_command_client.call_async(command)
        self.status = STATUS.SUCCESS
        return self.status


class Hover(BTNode):
    # Only works for GOTO mode
    def __init__(self, name, fp:FlightPlanner, duration:float):
        super().__init__(name)
        self.fp = fp
        self.duration = duration
        self.start_time = 0
        self.hold_pos = GotoSetpoint()

    def initialize(self):
        super().initialize()
        self.start_time = time.time()
        self.hold_pos.position = [self.fp._position.x, self.fp._position.y, self.fp._position.z]

    def tick(self):
        self.fp._goto_publisher.publish(self.hold_pos)
        if time.time() - self.start_time > self.duration:
            self.status = STATUS.SUCCESS
        return self.status


class Timer(BTNode):
    """
    Success after a certain time
    """
    def __init__(self, name, duration=5):
        super().__init__(name)
        self.duration = duration
        self.start_time = None

    def initialize(self):
        super().initialize()
        self.start_time = time.time()

    def reset(self):
        super().reset()
        self.start_time = None

    def tick(self):
        if time.time() - self.start_time > self.duration:
            self.status = STATUS.SUCCESS
        return self.status


class Goto(BTNode):
    def __init__(self, name, fp:FlightPlanner, points:list[list[float]]):
        super().__init__(name)
        self.fp = fp
        self.points = points
        self.waypoints = []
        self.wpi = 0
    
    def initialize(self):
        super().initialize()
        self.waypoints = [GotoSetpoint() for _ in self.points]
        for i, point in enumerate(self.points):
            self.waypoints[i].position = point

    def reset(self):
        super().reset()
        self.wpi = 0

    def has_reached(self) -> bool:
        tol = 1
        if (self.fp._position.x - self.waypoints[self.wpi].position[0])**2 + (self.fp._position.y - self.waypoints[self.wpi].position[1])**2 + (self.fp._position.z - self.waypoints[self.wpi].position[2])**2 > tol:
            return False
        return True

    def tick(self):
        # send goto command
        print("goto", self.wpi)
        self.fp._goto_publisher.publish(self.waypoints[self.wpi])
        if self.has_reached():
            print("reached", self.wpi)
            print(self.waypoints[self.wpi].position)
            self.wpi += 1

            if self.wpi == len(self.points):
                self.status = STATUS.SUCCESS
        return self.status


class Traj(BTNode):
    def __init__(self, name, fp:FlightPlanner, traj:trajectory.Trajectory):
        super().__init__(name)
        self.fp = fp
        self.traj = traj
        self.pathtime = 0
        self.goto = TrajectorySetpoint()

    def initialize(self):
        super().initialize()
        self.duration = self.traj.duration
        traj_iter = self.traj
        while traj_iter.next != None:
            traj_iter = traj_iter.next
            self.duration += traj_iter.duration

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
        return closest + 1

    def velocity_scale(self) -> float:
        slowdown = 1
        last_v = self.traj.velocity(self.pathtime - 9)
        for dt in np.arange(-10, 10, 0.1):
            v = self.traj.velocity(self.pathtime + 1 + dt)
            slowdown += 0.4 * np.linalg.norm(v - last_v) * (2 - abs(dt)/5.05)
            last_v = v
        return max(1, 5 / slowdown)

    def tick(self):
        # Action based, tries to clock as fast as btree
        # How to determine failure? built in time out?
        if self.pathtime > self.duration - 1:
            self.status = STATUS.SUCCESS
            return self.status
        self.pathtime = self.projection()
        self.goto.position = list(self.traj.path(self.pathtime))
        vscale = self.velocity_scale()
        print(vscale)
        self.goto.velocity = list(self.traj.velocity(self.pathtime) * vscale)
        self.fp._traj_publisher.publish(self.goto)
        return self.status


class AutoCenter(BTNode):
    """
    Action + Decorator
    """
    def __init__(self, name, fp:FlightPlanner, child:BTNode, k=0.002, floor_tol=10, max_rate=0.1):
        super().__init__(name)
        self.fp = fp
        self.setpoint = VehicleAttitudeSetpoint()
        self.k = k
        self.floor_tol = floor_tol
        self.max_rate = max_rate
        self.child = child

    def setup(self, blackboard:dict):
        super().setup(blackboard)
        if self.child:
            self.child.setup(blackboard)

    def initialize(self):
        super().initialize()
        if self.child:
            self.child.initialize()

    def reset(self):
        if self.child:
            self.child.reset()
        super().reset()

    def tick(self):
        # only works for GOTO mode
        dx = self.blackboard["target_dx"]
        if abs(dx) < self.floor_tol:
            self.setpoint.yaw_sp_move_rate = 0.0
        else:
            self.setpoint.yaw_sp_move_rate = min(max(self.k * dx, -self.max_rate), self.max_rate)

        # Publish a valid attitude setpoint for PX4.
        # Predict the yaw 0.1s into the future using the commanded yaw rate.
        q = [float(x) for x in self.fp._attitude.q]
        q0, q1, q2, q3 = q
        roll = math.atan2(2.0 * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3)
        pitch = math.asin(2.0 * (q0 * q2 - q1 * q3))
        yaw = math.atan2(2.0 * (q0 * q3 + q1 * q2), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3)
        future_yaw = yaw + self.setpoint.yaw_sp_move_rate * 0.1
        cy = math.cos(future_yaw * 0.5)
        sy = math.sin(future_yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        self.setpoint.q_d = [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ]
        #self.setpoint.thrust_body = [None, None, None]
        self.fp._attitude_publisher.publish(self.setpoint)

        if self.child:
            self.status = self.child.tick()
        elif abs(dx) < self.floor_tol:
            self.status = STATUS.SUCCESS

        return self.status


class AutoCenterTraj(BTNode):
    def __init__(self, name, fp:FlightPlanner, child:BTNode=None, k=0.002, floor_tol=10, max_rate=0.1):
        super().__init__(name)
        self.fp = fp
        self.goto = TrajectorySetpoint()
        self.k = k
        self.floor_tol = floor_tol
        self.max_rate = max_rate
        self.child = child

    def setup(self, blackboard:dict):
        super().setup(blackboard)
        if self.child:
            self.child.setup(blackboard)

    def initialize(self):
        super().initialize()
        # Hover
        self.goto.position = [self.fp._position.x, self.fp._position.y, self.fp._position.z]
        self.goto.velocity = [0.0, 0.0, 0.0]
        if self.child:
            self.child.initialize()

    def reset(self):
        if self.child:
            self.child.reset()
        super().reset()

    def tick(self):
        dx = self.blackboard.get("target_dx")
        if dx is None:
            print("warning: target_dx not found in blackboard")
            return self.status
        if abs(dx) < self.floor_tol:
            self.goto.yawspeed = 0.0
        else:
            self.goto.yawspeed = min(max(self.k * dx, -self.max_rate), self.max_rate)
            self.goto.yaw = self.fp._position.heading + self.goto.yawspeed * 0.002
        print(self.goto.yaw, self.goto.yawspeed)

        self.fp._traj_publisher.publish(self.goto)

        if self.child:
            self.status = self.child.tick()
        elif abs(dx) < self.floor_tol:
            self.status = STATUS.SUCCESS

        return self.status
