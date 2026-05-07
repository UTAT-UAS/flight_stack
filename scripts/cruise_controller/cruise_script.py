import math
import time
import numpy as np
from std_msgs.msg import Float32
import rclpy
from flight_stack.flight_stack import FlightPlanner
from flight_stack.pather import trajectory
from flight_stack_msgs.srv import CoreCommand
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition, BatteryStatus, TrajectorySetpoint, OffboardControlMode
from rclpy.qos import QoSPresetProfiles
from cruise_controller import CurrentController
from min_jerk import MinJerkTraj

class CruiseNode(FlightPlanner):

    def __init__(self):
        super().__init__()

        # subscribers

        # this is for sim only
        self._current_subscriber = self.create_subscription(
            Float32,
            "sim/current_draw",
            self._current_cb,
            10,
        )
        self._capacity_subscriber = self.create_subscription(
            Float32,
            "sim/discharged_mah",
            self._capacity_cb,
            10,
        )

        # real DDS battery stuff
        # self._battery_subscriber = self.create_subscription(
        #     BatteryStatus,
        #     "/fmu/out/battery_status",
        #     self._battery_cb,
        #     QoSPresetProfiles.SENSOR_DATA.value,
        # )

        # local position for velocity feedback
        self._local_pos_subscriber = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self._local_pos_cb,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

        # pubs -> this should be changed to flight stack publisher
        self._px4_traj_publisher = self.create_publisher(
            TrajectorySetpoint,
            "/fmu/in/trajectory_setpoint",
            QoSPresetProfiles.SYSTEM_DEFAULT.value,
        )
        self.traj_sp = TrajectorySetpoint()
        self.target_pos = [0.0, 0.0, 0.0]
        self.target_vel = [0.0, 0.0, 0.0]

        self._offboard_ctrl_publisher = self.create_publisher(
            OffboardControlMode,
            "/fmu/in/offboard_control_mode",
            QoSPresetProfiles.SYSTEM_DEFAULT.value,
        )

        ### Current controller setup ###

        # outer loop vars
        self.capacity_consumed = 0.0
        self.initial_capacity_consumed = None

        # inner loop vars
        self.current_draw = 0.0
        self.drone_vel_mag = 0.0
        self.target_cruise_spd = 0.0

        self.cc = None
        self.inner_timer = None
        self.outer_timer = None

        # initialize controller
        self._startup_timer = self.create_timer(0.1, self._initialize)

        ### Trajectory generation setup ###
        self.mj = None
        self.traj = None

        # min jerk results
        self.target_min_jerk_spd = 0
        self.target_spd = 0

        # timer to publish setpoints
        self._traj_timer = self.create_timer(0.01, self._pub_traj_setpoint)

    def _initialize(self):
        self._initialize_current_controller()
        self._initialize_minjerk_controller()

        self.target_pos = [self._position.x, self._position.y, self._position.z]
        self.target_vel = [0.0, 0.0, 0.0]
        self.traj_sp.yaw = math.nan
        self.traj_sp.yawspeed = math.nan
        self._pub_traj_setpoint()

        self._request_offboard()

        self.inner_timer = self.create_timer(self.cc.inner_dt, self._inner_cb)
        self.outer_timer = self.create_timer(self.cc.outer_dt, self._outer_cb)

        # stop the startup timer
        self._startup_timer.cancel()
        self._startup_timer = None

    def _initialize_current_controller(self):
        '''get initial conditions for current controller'''
        if self.initial_capacity_consumed is None:
            return  # not ready yet, check again on next timer tick

        # ready: create core and control timers once
        start_time = self.get_clock().now().nanoseconds / 1e9
        self.cc = CurrentController(start_time, self.initial_capacity_consumed)

    def _initialize_minjerk_controller(self):
        ''' set up min jerk trajectory '''
        x, y, z = self._position.x, self._position.y, -2.5#self._position.z
        paths = [
            trajectory.Line(np.array([x, y, z]), np.array([x - 110, y -545, z]), duration=556),
            trajectory.Line(np.array([x - 110, y - 545, z]), np.array([x, y, z]), duration=556),
        ]
        for i, traj in enumerate(paths[:-1]):
            traj.next = paths[i + 1]
        self.traj = paths[0]
    
        self.mj = MinJerkTraj("min_jerk", self, self.traj, self.cc.base_ff_speed, 4)
        self.mj.initialize()

    def _manual_ema(self, current_val, previous_ema, alpha):
        return (alpha * current_val) + ((1.0 - alpha) * previous_ema)

    def _current_cb(self, msg: Float32) -> None:
        self.current_draw = self._manual_ema(msg.data, self.current_draw, 0.1)

    def _capacity_cb(self, msg: Float32) -> None:
        self.capacity_consumed = msg.data / 1000.0  # convert mAh to Ah
        if self.initial_capacity_consumed is None:  # initialize starting point for coulomb counting
            self.initial_capacity_consumed = self.capacity_consumed

    def _battery_cb(self, msg: BatteryStatus) -> None:
        self.current_draw = self._manual_ema(msg.current_a, self.current_draw, 0.1)
        self.capacity_consumed = msg.discharged_mah / 1000.0  # Convert to Ah
        if self.initial_capacity_consumed is None:
            self.initial_capacity_consumed = self.capacity_consumed

    def _local_pos_cb(self, msg: VehicleLocalPosition) -> None:
        # calculate velocity magnitude from xy vectors
        self.drone_vel_mag = math.sqrt(msg.vx**2 + msg.vy**2)

    def _request_offboard(self):
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        offboard_msg.position = False
        offboard_msg.velocity = True 
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        self._offboard_ctrl_publisher.publish(offboard_msg)

    def _pub_traj_setpoint(self):
        self.traj_sp.position = self.target_pos#[math.nan, math.nan, math.nan]
        self.traj_sp.velocity = self.target_vel
        #print(self.target_vel)
        self._px4_traj_publisher.publish(self.traj_sp)

    # current controller loops
    def _outer_cb(self):
        if self.cc is None:
            return  # controller not initialized yet

        self.cc.outer_capacity_ctrl(self.capacity_consumed, self.get_clock().now().nanoseconds / 1e9)

        print(f"Current Draw: {self.current_draw:.2f} A, Commanded Current: {self.cc.current_setpoint:.2f} A, FF Vel: {self.cc.base_ff_speed:.2f} m/s\nTarget Ah: {self.cc.target_ah:.3f} Ah, Consumed Ah: {self.cc.discharged_ah_corrected:.3f} Ah, Ah Error: {self.cc.target_ah - self.cc.discharged_ah_corrected:.3f} Ah, Cruise Speed target: {self.target_cruise_spd:.3f}, MinJerk Speed target: {self.target_min_jerk_spd:.3f}")

    def _inner_cb(self):
        if self.cc is None:
            return  # controller not initialized yet

        self.target_cruise_spd = self.cc.inner_current_ctrl(self.current_draw, self.drone_vel_mag)

        self.mj.pathtime = self.mj.projection()
        self.target_min_jerk_spd = self.mj.velocity_scale() * 1.1 * self.cc.typical_cruise_spd / self.mj.target_vel # 10% wiggle room for cruise controller

        # allocator function
        self.target_spd = min(self.target_cruise_spd, self.target_min_jerk_spd)

        self.target_pos = list(self.traj.path(self.mj.pathtime))
        self.target_vel = list(self.traj.velocity(self.mj.pathtime) * self.target_spd)

        # check for completion
        if self.mj.pathtime > self.mj.duration - self.mj.project_ahead:
            print("Path completed")
            self.inner_timer.cancel()
            self.inner_timer = None
            self.outer_timer.cancel()
            self.outer_timer = None


def main(args=None):
    rclpy.init(args=args)
    node = CruiseNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
