import math
import numpy as np
import datetime
from std_msgs.msg import Float32
import rclpy
from flight_stack.flight_stack import FlightPlanner
from flight_stack.pather import trajectory
from flight_stack.btree.actions import MinJerkTraj
from px4_msgs.msg import VehicleLocalPosition, BatteryStatus, TrajectorySetpoint, OffboardControlMode
from rclpy.qos import QoSPresetProfiles
from cruise_controller import CurrentController
from flight_stack_msgs.srv import CoreCommand

data = [["pathtime", "mj_vx", "mj_vy", "cc_tgt", "px", "py", "pz", "vx", "vy", "vz", "ax", "ay", "az"]]
logfile = "logs/log-" + datetime.datetime.now().isoformat() + ".csv"
with open(logfile, 'w') as f:
    f.writelines(','.join(row)+'\n' for row in data)
data = []

class CruiseNode(FlightPlanner):

    def __init__(self):
        super().__init__()

        # subscribers
        #this is for sim only
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
        #self._battery_subscriber = self.create_subscription(
        #    BatteryStatus,
        #    "/fmu/out/battery_status",
        #    self._battery_cb,
        #    QoSPresetProfiles.SENSOR_DATA.value,
        #)

        # local position for velocity feedback
        self._local_pos_subscriber = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self._local_pos_cb,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

        self.traj_sp = TrajectorySetpoint()
        self.target_pos = [0.0, 0.0, 0.0]
        self.target_vel = [0.0, 0.0, 0.0]

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

        # timer to trigger offboard
        self.offboard_timer = None

    def _initialize(self):
        if self.initial_capacity_consumed is None:
            self.get_logger().info("Waiting for initial capacity reading...")
            return  # not ready yet, check again on next timer tick

        x, y, z = self._position.x, self._position.y, self._position.z
        if x == 0 or y == 0 or z == 0:
            self.get_logger().info(f"Invalid position: {x}, {y}, {z}")
            return

        # only fire once
        self._startup_timer.cancel()

        self._initialize_current_controller()
        self._initialize_minjerk_controller()

        self.target_pos = [self._position.x, self._position.y, self._position.z]
        self.target_vel = [0.0, 0.0, 0.0]
        self.traj_sp.yaw = math.nan
        self.traj_sp.yawspeed = math.nan
        self._pub_traj_setpoint()

        # request TRAJ mode
        command = CoreCommand.Request()
        command.request.command = 7 # CORE_TRAJ request command
        self._core_command_client.call_async(command)

        self.offboard_timer = self.create_timer(3, self._request_offboard)
        self.inner_timer = self.create_timer(self.cc.inner_dt, self._inner_cb)
        self.outer_timer = self.create_timer(self.cc.outer_dt, self._outer_cb)

        self.destroy_timer(self._startup_timer)

    def _initialize_current_controller(self):
        '''get initial conditions for current controller'''

        # ready: create core and control timers once
        start_time = self.get_clock().now().nanoseconds / 1e9
        self.cc = CurrentController(start_time, self.initial_capacity_consumed)

    def _initialize_minjerk_controller(self):
        ''' set up min jerk trajectory '''
        x, y, z = self._position.x, self._position.y, self._position.z
        paths = [
            trajectory.Line(np.array([x, y + 80, z]), np.array([x, y - 80, z]), duration=160),
            trajectory.Line(np.array([x, y - 80, z]), np.array([x, y + 80, z]), duration=160),
        ]
        for i, traj in enumerate(paths[:-1]):
            traj.next = paths[i + 1]
        self.traj = paths[0]

        self.mj = MinJerkTraj(
            name="min_jerk",
            fp=self,
            traj=self.traj,
            target_vel=self.cc.base_ff_speed,
            forecast_time=10,
            resolution=1,
            project_ahead=0.5
        )
        self.mj.initialize()
        self.mj.pathtime += 80

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
        if self.mj is None: return
        data.append([self.mj.pathtime, self.traj_sp.velocity[0], self.traj_sp.velocity[1], self.target_cruise_spd, msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz, msg.ax, msg.ay, msg.az])

    def _request_offboard(self):
        self.offboard_timer.cancel()
        command = CoreCommand.Request()
        command.request.command = 2 # Offboard mode request command
        self._core_command_client.call_async(command)
        self.destroy_timer(self.offboard_timer)

    def _pub_traj_setpoint(self):
        self.traj_sp.position = self.target_pos#[math.nan, math.nan, math.nan]
        self.traj_sp.velocity = self.target_vel
        #print(self.target_vel)
        self._traj_publisher.publish(self.traj_sp)

    # current controller loops
    def _outer_cb(self):
        if self.cc is None:
            return  # controller not initialized yet

        self.cc.outer_capacity_ctrl(self.capacity_consumed, self.get_clock().now().nanoseconds / 1e9)

        self.get_logger().info(f"Pathtime: {self.mj.pathtime}, Current Draw: {self.current_draw:.2f} A, Commanded Current: {self.cc.current_setpoint:.2f} A, FF Vel: {self.cc.base_ff_speed:.2f} m/s\nTarget Ah: {self.cc.target_ah:.3f} Ah, Consumed Ah: {self.cc.discharged_ah_corrected:.3f} Ah, Ah Error: {self.cc.target_ah - self.cc.discharged_ah_corrected:.3f} Ah, Cruise Speed target: {self.target_cruise_spd:.3f}, MinJerk Speed target: {self.target_min_jerk_spd:.3f}")
        self.flush_data()

    def _inner_cb(self):
        if self.cc is None:
            return  # controller not initialized yet

        # check for completion
        if self.mj.pathtime > self.mj.duration - self.mj.project_ahead - 80:
            self.get_logger().info("Path completed")
            self.inner_timer.cancel()
            self.outer_timer.cancel()
            self.target_pos = list(self.traj.path(self.mj.duration))
            self.target_vel = [0.0, 0.0, 0.0]
            self._pub_traj_setpoint()
            # end 
            self.destroy_timer(self.outer_timer)
            self.destroy_timer(self.inner_timer)
            return

        self.target_cruise_spd = self.cc.inner_current_ctrl(self.current_draw, self.drone_vel_mag)

        self.mj.pathtime = self.mj.projection()
        vx, vy = self.mj.velocity()
        norm = (vx**2 + vy**2)**0.5

        self.target_min_jerk_spd = norm * 1.1 * self.cc.typical_cruise_spd / self.mj.target_vel # 10% wiggle room for cruise controller

        # allocator function
        self.target_spd = min(self.target_cruise_spd, self.target_min_jerk_spd)

        self.mj.project_ahead = 0.025 + 0.475 * self.drone_vel_mag / self.cc.typical_cruise_spd
        self.target_pos = list(self.mj.position())
        self.target_vel = [vx * self.target_spd / norm, vy * self.target_spd / norm, 0.0]

    def flush_data(self):
        with open(logfile, 'a') as f:
            f.writelines(','.join([str(x) for x in row])+'\n' for row in data)
        data.clear()

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
