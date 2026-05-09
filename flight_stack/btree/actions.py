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


class AwaitOffboard(BTNode):
    def __init__(self, name, fp:FlightPlanner):
        super().__init__(name)
        self.fp = fp

    def tick(self):
        print("awaiting offboard")
        if self.fp._status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            return self.status
        self.status = STATUS.SUCCESS
        return self.status


class SetGotoMode(BTNode):
    def __init__(self, name, fp:FlightPlanner):
        super().__init__(name)
        self.fp = fp

    def tick(self):
        # send current position as goto setpoint before switch to goto mode (otherwise might have large jump)
        goto = GotoSetpoint()
        goto.position = [self.fp._position.x, self.fp._position.y, self.fp._position.z]
        goto.flag_control_heading = False
        self.fp._goto_publisher.publish(goto) # publish multiple times to ensure received before mode switch
        self.fp._goto_publisher.publish(goto)
        self.fp._goto_publisher.publish(goto)
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
        # send current position as traj setpoint to switch to traj mode (otherwise might have large jump)
        traj = TrajectorySetpoint()
        traj.position = [self.fp._position.x, self.fp._position.y, self.fp._position.z]
        traj.velocity = [0.0, 0.0, 0.0]
        traj.yaw = self.fp._position.heading
        traj.yawspeed = 0.0
        self.fp._traj_publisher.publish(traj) # publish multiple times to ensure received before mode switch
        self.fp._traj_publisher.publish(traj)
        self.fp._traj_publisher.publish(traj)
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
        closest_dist = float('inf')
        cur_pos = np.array([self.fp._position.x, self.fp._position.y, self.fp._position.z])
        # binary search instead of linear interpol?
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


class MinJerkTraj(BTNode):
    def __init__(self, name, fp:FlightPlanner, traj:trajectory.Trajectory, target_vel:float, forecast_time:float, resolution:float=0.1, project_ahead:float=0.1):
        super().__init__(name)
        self.fp = fp
        self.traj = traj
        self.pathtime = 0
        self.duration = 0
        self.traj_sp = TrajectorySetpoint()
        self.traj_sp.yaw = math.nan
        self.traj_sp.yawspeed = math.nan

        # parameters
        self.T = forecast_time
        self.target_vel = target_vel
        self.horizon = self.T * self.target_vel
        self.resolution = resolution
        self.project_ahead = project_ahead
        self.constants = []
        self.powers = []

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
        closest_dist = float('inf')
        cur_pos = np.array([self.fp._position.x, self.fp._position.y, self.fp._position.z])
        # binary search instead of linear interpol?
        for t in np.arange(max(self.pathtime - 5, 0), min(self.pathtime + 5, self.duration), 0.1):
            pos = self.traj.path(t)
            dist = np.linalg.norm(cur_pos - pos)
            if dist < closest_dist:
                closest_dist = dist
                closest = t
        return closest

    def position(self) -> np.ndarray:
        return self.traj.path(self.pathtime + self.project_ahead)

    def velocity(self) -> tuple[float, float]:
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
        for i, dt in enumerate(np.arange(self.project_ahead, self.horizon + self.project_ahead, self.resolution)):
            t1 = self.pathtime + dt
            t0 = t1 - self.horizon
            # smoothing start and end by treating as 180s
            if t0 >= 0:
                pos0 = self.traj.path(t0)
                vel0 = self.traj.velocity(t0)
            else:
                pos0 = self.traj.path(-t0)
                vel0 = -self.traj.velocity(-t0)

            if t1 <= self.duration:
                pos1 = self.traj.path(t1)
                vel1 = self.traj.velocity(t1)
            else:
                t1 = self.duration + self.duration - t1
                pos1 = self.traj.path(t1)
                vel1 = -self.traj.velocity(t1)

            dpx = pos1[0] - pos0[0]
            vx0 = vel0[0]
            vx1 = vel1[0]

            dpy = pos1[1] - pos0[1]
            vy0 = vel0[1]
            vy1 = vel1[1]

            sum_v_x += vx0 + np.dot(np.matmul(self.constants, [dpx, vx1, vx0]), self.powers[i])
            sum_v_y += vy0 + np.dot(np.matmul(self.constants, [dpy, vy1, vy0]), self.powers[i])
        return sum_v_x/len(self.powers), sum_v_y/len(self.powers)
        #return (sum_v_x**2 + sum_v_y**2) ** 0.5 / len(self.powers)

    def tick(self):
        # Action based, tries to clock as fast as btree
        # How to determine failure? built in time out?
        if self.pathtime > self.duration - self.project_ahead:
            self.traj_sp.position = list(self.traj.path(self.duration))
            self.traj_sp.velocity = [0.0, 0.0, 0.0]
            self.fp._traj_publisher.publish(self.traj_sp)
            self.status = STATUS.SUCCESS
            return self.status

        self.pathtime = self.projection()
        #self.target_spd = self.velocity_scale()
        #print(f"{self.name} target speed: {self.target_spd}")

        self.traj_sp.position = list(self.traj.path(self.pathtime))
        #self.traj_sp.velocity = list(self.traj.velocity(self.pathtime) * self.target_spd)
        vx, vy = self.velocity()
        print(f"{self.name} target vel: {vx}, {vy}")
        self.traj_sp.velocity = [vx, vy, 0.0]
        self.fp._traj_publisher.publish(self.traj_sp)
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
        self.goto.yaw = self.fp._position.heading
        self.goto.yawspeed = 0
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
            dx = 0
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

class AdjustFromDetection(BTNode):
    def __init__(self, name, fp:FlightPlanner):
        super().__init__(name)
        self.fp = fp
        self.has_requested = False

    def setup(self, blackboard:dict):
        super().setup(blackboard)

    def initialize(self):
        super().initialize()
        self.has_requested = False

    def reset(self):
        super().reset()
        self.has_requested = False

    def tick(self):
        if self.blackboard is None or "target_pos" not in self.blackboard:
            return STATUS.RUNNING
        
        polygon = self.blackboard.get("target_pos")
        if len(polygon.points) < 5:
            return STATUS.RUNNING

        p_c = polygon.points[0]
        p_tl = polygon.points[1]
        p_tr = polygon.points[2]
        p_br = polygon.points[3]
        p_bl = polygon.points[4]

        # Ignore if any point is nan
        if any(math.isnan(p.x) for p in [p_c, p_tl, p_tr, p_br, p_bl]):
            return STATUS.RUNNING

        if self.has_requested:
            return STATUS.SUCCESS

        # Camera frame: x is right, y is down, z is forward
        xl = (p_tl.x + p_bl.x) / 2.0
        zl = (p_tl.z + p_bl.z) / 2.0
        
        xr = (p_tr.x + p_br.x) / 2.0
        zr = (p_tr.z + p_br.z) / 2.0

        xc = p_c.x
        zc = p_c.z

        dx = xr - xl
        dz = zr - zl

        # Normal towards camera
        # If wall is facing camera, (dz, -dx) points to camera
        n_x = dz
        n_z = -dx
        norm = math.hypot(n_x, n_z)
        if norm < 1e-3:
            return STATUS.SUCCESS
        n_x /= norm
        n_z /= norm

        L = math.hypot(xc, zc)
        if L < 1e-3: # too close
            return STATUS.SUCCESS

        P_cam_x = xc + L * n_x
        P_cam_z = zc + L * n_z

        delta_cam_x = P_cam_x
        delta_cam_z = P_cam_z

        # View vector from new position to center
        V_x = xc - P_cam_x
        V_z = zc - P_cam_z
        delta_yaw = math.atan2(V_x, V_z)

        # Transform to NED
        yaw = self.fp._position.heading
        
        delta_N = delta_cam_z * math.cos(yaw) - delta_cam_x * math.sin(yaw)
        delta_E = delta_cam_z * math.sin(yaw) + delta_cam_x * math.cos(yaw)

        new_N = float(self.fp._position.x + delta_N)
        new_E = float(self.fp._position.y + delta_E)
        new_D = float(self.fp._position.z) # Keep same altitude
        
        new_yaw = float(yaw + delta_yaw)
        # Normalize yaw to [-pi, pi]
        new_yaw = (new_yaw + math.pi) % (2.0 * math.pi) - math.pi

        goto_msg = GotoSetpoint()
        goto_msg.position = [new_N, new_E, new_D]
        goto_msg.heading = new_yaw
        goto_msg.flag_control_heading = True

        self.fp._goto_publisher.publish(goto_msg)
        self.has_requested = True
        print(f"AdjustFromDetection: Moving to N={new_N:.2f}, E={new_E:.2f}, D={new_D:.2f}, Yaw={new_yaw:.2f}")

        return STATUS.SUCCESS


class MoveToTarget(BTNode):
    def __init__(self, name, fp:FlightPlanner, target_dist=1.5):
        super().__init__(name)
        self.fp = fp
        self.target_dist = target_dist
        self.has_requested = False

    def setup(self, blackboard:dict):
        super().setup(blackboard)

    def initialize(self):
        super().initialize()
        self.has_requested = False

    def reset(self):
        super().reset()
        self.has_requested = False

    def tick(self):
        if self.blackboard is None or "target_pos" not in self.blackboard:
            return STATUS.RUNNING
        
        polygon = self.blackboard.get("target_pos")
        if len(polygon.points) == 0:
            return STATUS.RUNNING

        p_c = polygon.points[0]

        # Ignore if the center point is nan
        if math.isnan(p_c.x) or math.isnan(p_c.z):
            return STATUS.RUNNING

        if self.has_requested:
            return STATUS.SUCCESS

        xc = p_c.x
        zc = p_c.z

        # Current horizontal distance to target
        L = math.hypot(xc, zc)
        if L < 1e-3:
            return STATUS.SUCCESS

        # How much distance we need to cover to be exactly target_dist away
        delta_L = L - self.target_dist

        # Direction to the target in the camera frame
        dir_x = xc / L
        dir_z = zc / L

        delta_cam_x = delta_L * dir_x
        delta_cam_z = delta_L * dir_z

        # Transform to NED
        yaw = self.fp._position.heading
        
        delta_N = delta_cam_z * math.cos(yaw) - delta_cam_x * math.sin(yaw)
        delta_E = delta_cam_z * math.sin(yaw) + delta_cam_x * math.cos(yaw)

        new_N = float(self.fp._position.x + delta_N)
        new_E = float(self.fp._position.y + delta_E)
        new_D = float(self.fp._position.z) # Keep same altitude

        goto_msg = GotoSetpoint()
        goto_msg.position = [new_N, new_E, new_D]
        goto_msg.heading = yaw
        goto_msg.flag_control_heading = True

        self.fp._goto_publisher.publish(goto_msg)
        self.has_requested = True
        print(f"MoveToTarget: Moving to N={new_N:.2f}, E={new_E:.2f}, D={new_D:.2f}, Yaw={yaw:.2f} (Delta={delta_L:.2f}m)")

        return STATUS.SUCCESS