#!/usr/bin/env python3
import flight_stack
print("Using flight_stack from:", flight_stack.__file__)
import math
import numpy as np
import time
import datetime

import rclpy

from flight_stack.flight_stack import FlightPlanner
from flight_stack.pather import trajectory
from flight_stack.btree import manager, controls, decorators, actions, utils

from flight_stack_msgs.srv import CoreCommand
from std_msgs.msg import Float32
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition, BatteryStatus, TrajectorySetpoint
from rclpy.qos import QoSPresetProfiles

data = [["pathtime", "mj_vx", "mj_vy", "cc_tgt", "px", "py", "pz", "vx", "vy", "vz", "ax", "ay", "az"]]
logfile = "logs/log-" + datetime.datetime.now().isoformat() + ".csv"
with open(logfile, 'w') as f:
    f.writelines(','.join(row)+'\n' for row in data)
data = []

class MinJerkTraj(utils.BTNode):
    def __init__(self, name, fp:FlightPlanner, traj:trajectory.Trajectory, target_vel:float, forecast_time:float, resolution:float=0.1, project_ahead:float=0.1):
        super().__init__(name)
        self.fp = fp
        self.traj = traj
        self.pathtime = 0
        self.duration = 0
        self.traj_sp = TrajectorySetpoint()
        self.traj_sp.yaw = math.nan
        self.traj_sp.yawspeed = math.nan

        # restrict path jumping
        self.leg = 0
        self.subdurations = []

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
        self.subdurations = [0, self.traj.duration]
        traj_iter = self.traj
        while traj_iter.next != None:
            traj_iter = traj_iter.next
            self.duration += traj_iter.duration
            self.subdurations.append(self.subdurations[-1] + traj_iter.duration)
        self.subdurations.append(self.subdurations[-1] + 1)

        self.constants = [
            3 * np.array([20/self.T, -8, -12]) / (2*self.T**2),
            4 * np.array([-30/self.T, 14, 16]) / (2*self.T**3),
            5 * np.array([12/self.T, -6, -6]) / (2*self.T**4),
        ]
        self.powers = [[(-x)**i for i in range(2, 5)] for x in np.arange(-self.T, 0, self.resolution/self.target_vel)]

    def reset(self):
        super().reset()
        self.pathtime = 0
        self.leg = 0

    def projection(self) -> bool:
        resolution = 0.1
        if self.pathtime >= self.subdurations[self.leg + 1] - resolution + 0.001: # accumulated floating point error
            self.leg += 1

        closest = self.pathtime
        closest_dist = float('inf')
        cur_pos = np.array([self.fp._position.x, self.fp._position.y, self.fp._position.z])
        # binary search instead of linear interpol?
        for t in np.arange(max(self.pathtime - 5, self.subdurations[self.leg]), min(self.pathtime + 5, self.subdurations[self.leg + 1] + resolution / 2, self.duration), resolution):
            pos = self.traj.path(t)
            dist = np.linalg.norm(cur_pos - pos)
            if dist <= closest_dist:
                closest_dist = dist
                closest = t
        return closest

    def position(self) -> np.ndarray:
        return list(self.traj.path(self.pathtime + self.project_ahead))

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
            self.status = utils.STATUS.SUCCESS
            return self.status

        self.pathtime = self.projection()
        current_spd = (self.fp._position.vx**2 + self.fp._position.vy**2) ** 0.5
        self.project_ahead = 0.025 + 0.475 * current_spd / self.target_vel

        self.traj_sp.position = self.position()
        #self.traj_sp.velocity = list(self.traj.velocity(self.pathtime) * self.target_spd)
        vx, vy = self.velocity()
        print(f"{self.name} - Pathtime: {self.pathtime}, Velocity: {vx}, {vy}")
        self.traj_sp.velocity = [vx, vy, 0.0]
        self.fp._traj_publisher.publish(self.traj_sp)

        return self.status

class MinJerkTester(FlightPlanner):
    def __init__(self):
        super().__init__()

        # trajectory
        self.traj = trajectory.Wrapper(trajectory.Custom(None, 0, np.array([]), 0))
        self.duration = 0
        # state vars
        self.pathtime = 0

        # pubs
        self.goto = TrajectorySetpoint()
        self.goto.yaw = math.nan
        self.goto.yawspeed = math.nan

        # behavior tree assembly
        self.mj = MinJerkTraj(
                        name="min_jerk",
                        fp=self,
                        traj=self.traj,
                        target_vel=5,
                        forecast_time=4,
                        resolution=0.1,
                        project_ahead=0.1
                    )
        self.btree = manager.BehaviorTree("min_jerk_trajectory_test")
        self.btree.setroot(
            controls.Sequence(
                name="root",
                children=[
                    decorators.RemapStatus(
                        name="always_succeed_offboard",
                        child=decorators.Timeout(
                            name="offboard_timeout",
                            timeout=5,
                            child=actions.SetOffboard(
                                name="set_offboard",
                                fp=self
                            )
                        ),
                        remap={utils.STATUS.FAILURE: utils.STATUS.SUCCESS}
                    ),
                    actions.SetTrajMode(
                        name="set_traj",
                        fp=self
                    ),
                    self.mj
                ]
            )
        )
        self.btree.setup()
        time.sleep(1)  # wait for setup to complete
        self.btree.initialize()

        self._local_pos_subscriber = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self._local_pos_cb,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

    def _local_pos_cb(self, msg: VehicleLocalPosition) -> None:
        # calculate velocity magnitude from xy vectors
        self.drone_vel_mag = math.sqrt(msg.vx**2 + msg.vy**2)
        if self.mj.status != utils.STATUS.RUNNING: return
        data.append([self.mj.pathtime, self.mj.traj_sp.velocity[0], self.mj.traj_sp.velocity[1], 0, msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz, msg.ax, msg.ay, msg.az])

    def main_loop(self):
        self.time = time.time()

        if self.duration == 0:
            x, y, z = self._position.x, self._position.y, self._position.z
            if (x==0 or y==0 or z==0):
                print(f"{x}, {y}, {z}")
                return

            paths = [
                trajectory.Line(np.array([x, y, z]), np.array([x, y + 10, z]), duration=10),
                trajectory.Circle(np.array([x, y + 10, z]), np.array([x + 5, y + 10, z]), cycles=0.5, axis=np.array([0, 0, -1])),
                #trajectory.Circle(np.array([x + 10, y + 10, z]), np.array([x + 12, y + 10, z]), cycles=1),
                trajectory.Line(np.array([x + 10, y + 10, z]), np.array([x + 10, y - 10, z]), duration=20),
                trajectory.Line(np.array([x + 10, y - 10, z]), np.array([x + 15, y - 15, z]), duration=50**0.5),
                trajectory.Line(np.array([x + 15, y - 15, z]), np.array([x + 30, y, z]), duration=450**0.5),
                trajectory.Line(np.array([x + 30, y, z]), np.array([x, y, z]), duration=30),
            ]
            for i, traj in enumerate(paths[:-1]):
                traj.next = paths[i + 1]
                self.duration += traj.duration
            self.duration += paths[-1].duration
            self.traj.replace_child(paths[0])

        self.btree.tick()
        self.flush_data()
        if self.btree.status == manager.STATUS.SUCCESS:
            print("mission complete")
            exit()
        elif self.btree.status == manager.STATUS.FAILURE:
            print("mission failed")
            exit()

    def flush_data(self):
        with open(logfile, 'a') as f:
            f.writelines(','.join([str(x) for x in row])+'\n' for row in data)
        data.clear()

def main(args=None):
    rclpy.init(args=args)

    btree_fp = MinJerkTester()

    rclpy.spin(btree_fp)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    btree_fp.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()