import time
import numpy as np

from flight_stack.flight_stack import FlightPlanner
from flight_stack.pather import trajectory

from flight_stack_msgs.srv import CoreCommand
from px4_msgs.msg import GotoSetpoint, VehicleStatus, TrajectorySetpoint

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
        print("CoreMode -> GOTO request sent")
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
