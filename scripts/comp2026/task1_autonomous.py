import math
import numpy as np
from std_msgs.msg import Float32, Bool
import rclpy
from flight_stack.flight_stack import FlightPlanner
from flight_stack.pather import trajectory
from flight_stack.btree.actions import MinJerkTraj, SetOffboard
from flight_stack.btree.manager import BehaviorTree
from flight_stack.btree import utils
from px4_msgs.msg import VehicleLocalPosition, BatteryStatus, TrajectorySetpoint, OffboardControlMode
from rclpy.qos import QoSPresetProfiles
from flight_stack_msgs.srv import CoreCommand
import lap as lap


class CurrentController():

    def __init__(self, start_time=None, initial_capacity_consumed=None):

        # outer loop vars
        self.start_time = start_time
        self.initial_capacity_consumed = initial_capacity_consumed
        self.target_current_draw = 50.0 # input of outer loop
        self.target_ah = 0.0
        self.discharged_ah_corrected = 0.0
        self.current_setpoint = self.target_current_draw   # Output of outer loop, defaults to 60A 
        self.max_current = 65.0
        self.min_current = 25.0
        self.base_ff_speed = self.get_feedforward_velocity(self.current_setpoint)
        self.typical_cruise_spd = self.base_ff_speed

        self.kp_outer = 30.0  # Amps to adjust per Ah error
        self.outer_current_override = None

        # inner loop vars
        self.target_spd = 0.0  # Output of inner loop
        self.integral_limit = 10.0  # m/s, max contribution of integral term to velocity setpoint
        self.stored_integral = 0.0

        self.kp_inner = 0.25
        self.ki_inner = 0.0


        self.last_target_speed = self.base_ff_speed
        self.max_acceleration = 1.0      # m/s^2 

        # timers
        self.inner_dt = 0.01  # 100 Hz 
        self.outer_dt = 0.1  # 10 Hz

    def get_feedforward_velocity(self, target_current: float) -> float:
        """
        Solves the cubic thrust curve: e.g. 0.003v^3 - 0.1v + 25 - I_target = 0
        This has been updated from experimental flight test data as:
        I(v) = 0.00621v^3 - 0.1010v^2 + 0.4244v + 24.7248
        """
        coeffs = [0.00621, -0.1010, 0.4244, 24.7248 - target_current]
        roots = np.roots(coeffs)
        real_roots = roots[np.isreal(roots)].real

        # Pick the first valid positive velocity root
        for r in real_roots:
            if r >= 0:
                return float(r)
        return 0.0

    def inner_current_ctrl(self, current_draw, drone_speed):
        """Update target velocity based on current tracking error."""
        error = self.current_setpoint - current_draw
        p_term = self.kp_inner * error

        # anti-windup by scaling integral based on how close we are to target vel
        safe_target_vel = max(self.target_spd, 0.1) # Prevent div by 0
        vel_ratio = drone_speed / safe_target_vel
        vel_ratio = max(0.0, min(vel_ratio, 1.0)) # Clamp between 0 and 1

        self.stored_integral += (error * self.ki_inner * self.inner_dt) * vel_ratio

        # clamp integral action
        self.stored_integral = max(-self.integral_limit, min(self.stored_integral, self.integral_limit))

        # raw target vel
        self.target_spd = self.base_ff_speed + p_term + self.stored_integral
        # track typical commanded speed to scale min jerk speed
        a = 0.0001 + 0.0003*vel_ratio*vel_ratio
        self.typical_cruise_spd = a * self.target_spd + (1-a) * self.typical_cruise_spd

        # slew rate limiter to velocity output
        max_delta = self.max_acceleration * self.inner_dt
        # calculate requested rate of change
        requested_delta = self.target_spd - self.last_target_speed
        # clamp request
        clamped_delta = max(-max_delta, min(requested_delta, max_delta))

        # publish slewed velocity setpoint
        self.target_spd = self.last_target_speed + clamped_delta
        self.last_target_speed = self.target_spd

        return self.target_spd

    def outer_capacity_ctrl(self, discharged_ah, current_time):
        """Update the current setpoint from Ah tracking error."""
        if self.initial_capacity_consumed is None:
            return

        elapsed_time_hours = (current_time - self.start_time) / 3600.0

        # Determine how many Ah we SHOULD have burned by this exact millisecond
        self.target_ah = self.target_current_draw * elapsed_time_hours

        self.discharged_ah_corrected = discharged_ah - self.initial_capacity_consumed
        ah_error = self.target_ah - self.discharged_ah_corrected

        #  calculate new current to command based on Ah error
        base_current = self.target_current_draw
        commanded_current = base_current + (self.kp_outer * ah_error)

        # clamp current setpoint to safe limits
        self.current_setpoint = max(self.min_current, min(commanded_current, self.max_current))
        if self.outer_current_override is not None:
            self.current_setpoint = self.outer_current_override

        # update feedforward velocity based on new current setpoint
        self.base_ff_speed = self.get_feedforward_velocity(self.current_setpoint)


class MinJerkLaps(utils.BTNode):
    def __init__(self, name, fp:FlightPlanner, lap:trajectory.Trajectory, ingress:trajectory.Trajectory, target_vel:float, forecast_time:float, resolution:float=0.1, project_ahead:float=0.1):
        super().__init__(name)
        self.fp = fp
        self.lap = lap
        self.pathtime = 0
        self.duration = 0
        self.ingress = ingress
        self.ingress_len = 0
        self.ingress_duration = 0
        self.ingress_removed = False
        self.traj = None
        self.traj_sp = TrajectorySetpoint()
        self.traj_sp.yaw = math.nan
        self.traj_sp.yawspeed = math.nan
        self.again = True

        # restrict path jumping
        self.leg = 0
        self.subdurations = []
        self.lap_completion = []
        self.laps_completed = 0

        # parameters
        self.T = forecast_time
        self.target_vel = target_vel
        self.horizon = self.T * self.target_vel
        self.resolution = resolution
        self.project_ahead = project_ahead
        self.constants = []
        self.powers = []

    def remove_ingress(self):
        self.pathtime -= self.ingress_duration
        self.leg -= self.ingress_len
        self.duration -= self.ingress_duration
        self.subdurations = [x - self.ingress_duration for x in self.subdurations[self.ingress_len:]]
        self.traj = self.lap
        self.ingress_removed = True

    def initialize(self):
        super().initialize()
        self.ingress_len = 1
        self.duration = self.ingress.duration
        self.subdurations = [0, self.ingress.duration]

        traj_iter = self.ingress
        while traj_iter.next != None:
            traj_iter = traj_iter.next
            self.ingress_len += 1
            self.duration += traj_iter.duration
            self.subdurations.append(self.subdurations[-1] + traj_iter.duration)
        self.ingress_duration = self.subdurations[-1]

        traj_iter.next = self.lap
        while traj_iter.next != None:
            traj_iter = traj_iter.next
            self.duration += traj_iter.duration
            self.subdurations.append(self.subdurations[-1] + traj_iter.duration)

        self.traj = self.ingress
        self.lap_completion = [False for i in range(len(self.subdurations) - 1)]
        self.lap_completion[0] = True

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
            self.lap_completion[self.leg] = self.lap_completion[self.leg - 1]
            if self.lap_completion[-1]:
                self.leg = 0
                self.laps_completed += 1
                self.lap_completion = [False for i in range(len(self.subdurations) - 1)]
                self.lap_completion[0] = True

        closest = self.pathtime
        closest_dist = float('inf')
        cur_pos = np.array([self.fp._position.x, self.fp._position.y, self.fp._position.z])
        # binary search instead of linear interpol?
        for t in np.arange(max(self.pathtime - 5, self.subdurations[self.leg]), min(self.pathtime + 5, self.subdurations[self.leg + 1] + resolution / 2), resolution):
            pos = self.traj.path(t % self.duration)
            dist = np.linalg.norm(cur_pos - pos)
            if dist <= closest_dist:
                closest_dist = dist
                closest = t
        
        if not self.ingress_removed and self.leg > self.ingress_len + 2:
            self.remove_ingress()

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
            if t0 < 0 and not self.ingress_removed:
                pos0 = self.traj.path(-t0)
                vel0 = -self.traj.velocity(-t0)
            else:
                pos0 = self.traj.path(t0 % self.duration)
                vel0 = self.traj.velocity(t0 % self.duration)

            if t1 > self.duration and not self.again:
                t1 = self.duration + self.duration - t1
                pos1 = self.traj.path(t1)
                vel1 = -self.traj.velocity(t1)
            else:
                pos1 = self.traj.path(t1 % self.duration)
                vel1 = self.traj.velocity(t1 % self.duration)

            dpx = pos1[0] - pos0[0]
            vx0 = vel0[0]
            vx1 = vel1[0]

            dpy = pos1[1] - pos0[1]
            vy0 = vel0[1]
            vy1 = vel1[1]

            sum_v_x += vx0 + np.dot(np.matmul(self.constants, [dpx, vx1, vx0]), self.powers[i])
            sum_v_y += vy0 + np.dot(np.matmul(self.constants, [dpy, vy1, vy0]), self.powers[i])
        return sum_v_x/len(self.powers), sum_v_y/len(self.powers)

    def tick(self):
        if not self.again and (self.pathtime > self.duration - self.project_ahead or self.pathtime < self.project_ahead):
            self.traj_sp.position = list(self.traj.path(self.duration))
            self.traj_sp.velocity = [0.0, 0.0, 0.0]
            self.fp._traj_publisher.publish(self.traj_sp)
            self.status = utils.STATUS.SUCCESS
            return self.status

        self.pathtime = self.projection()
        current_spd = (self.fp._position.vx**2 + self.fp._position.vy**2) ** 0.5
        self.project_ahead = 0.025 + 0.475 * current_spd / self.target_vel

        self.traj_sp.position = self.position()
        vx, vy = self.velocity()
        print(f"{self.name} - Pathtime: {self.pathtime}, Velocity: {vx}, {vy}")
        self.traj_sp.velocity = [vx, vy, 0.0]
        self.fp._traj_publisher.publish(self.traj_sp)

        return self.status


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

        # override lap decision
        self._stop_laps_subscriber = self.create_subscription(
            Bool,
            "task1/stop_laps",
            self._stop_laps_cb,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

        # outer loop current setpoint override
        self._outer_override_subscriber = self.create_subscription(
            Float32,
            "/task1/current_override",
            self._outer_override_cb,
            QoSPresetProfiles.SENSOR_DATA.value,
        )


        # pubs -> this should be changed to flight stack publisher
        # self._px4_traj_publisher = self.create_publisher(
        #     TrajectorySetpoint,
        #     "/fmu/in/trajectory_setpoint",
        #     QoSPresetProfiles.SYSTEM_DEFAULT.value,
        # )
        self.traj_sp = TrajectorySetpoint()
        self.target_pos = [0.0, 0.0, 0.0]
        self.target_vel = [0.0, 0.0, 0.0]

        # self._offboard_ctrl_publisher = self.create_publisher(
        #     OffboardControlMode,
        #     "/fmu/in/offboard_control_mode",
        #     QoSPresetProfiles.SYSTEM_DEFAULT.value,
        # )

        self.ah_limit = 7.0

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
        if self.initial_capacity_consumed is None:
            self.get_logger().info("Waiting for initial capacity reading...")
            return  # not ready yet, check again on next timer tick
        
        x, y, z = self._position.x, self._position.y, self._position.z
        if x==0 or y==0 or z==0:
            self.get_logger().info(f"Invalid position: {x}, {y}, {z}")
            return

        self._initialize_current_controller()
        print("here")
        self._initialize_minjerk_controller()
        print("hello")

        self.target_pos = [self._position.x, self._position.y, self._position.z]
        self.target_vel = [0.0, 0.0, 0.0]
        self.traj_sp.yaw = math.nan
        self.traj_sp.yawspeed = math.nan
        self._pub_traj_setpoint()

        # self._request_offboard()
        # request offboard
        command = CoreCommand.Request()
        command.request.command = 7 # CORE_TRAJ request command
        self._core_command_client.call_async(command)

        self.inner_timer = self.create_timer(self.cc.inner_dt, self._inner_cb)
        self.outer_timer = self.create_timer(self.cc.outer_dt, self._outer_cb)

        req_offboard = SetOffboard(name="request_offboard", fp=self)
        req_offboard.tick()  # request offboard mode immediately

        # stop the startup timer
        self._startup_timer.cancel()
        self._startup_timer = None

    def _initialize_current_controller(self):
        '''get initial conditions for current controller'''

        # ready: create core and control timers once
        start_time = self.get_clock().now().nanoseconds / 1e9
        self.cc = CurrentController(start_time, self.initial_capacity_consumed)

    def get_lap_paths(self):
        # 1. Get the drone's current local PX4 position
        curr_x = self._position.x
        curr_y = self._position.y
        curr_z = self._position.z

        # 2. Get the GPS coordinates (Ensure you have these variables available in your node)
        # Replace these with your actual variable names for the drone's current global position
        drone_lat = self._position.ref_lat#current_lat
        drone_lon = self._position.ref_lon
        wp0_lat = lap.first_wp.latitude
        wp0_lon = lap.first_wp.longitude
        
        R_EARTH = 6378137.0
        delta_lat = math.radians(wp0_lat - drone_lat)
        delta_lon = math.radians(wp0_lon - drone_lon)
        dist_n = delta_lat * R_EARTH
        dist_e = delta_lon * R_EARTH * math.cos(math.radians(drone_lat))
        
        # wp0's local coordinates
        wp0_off_x = dist_n
        wp0_off_y = dist_e
        wp0_off_z = curr_z  # maintain current altitude
        
        lap_paths = lap.generate_lap(wp0_off_x, wp0_off_y, wp0_off_z)
        
        cur_pos = np.array([curr_x, curr_y, curr_z])
        ingress = trajectory.Line(start=cur_pos, end=lap.overshoot_points[1], duration=np.linalg.norm(lap.overshoot_points[1] - cur_pos))
        lap_paths.insert(0, ingress)

        return lap_paths

    def _initialize_minjerk_controller(self):
        ''' set up min jerk trajectory '''
        paths = self.get_lap_paths()
        print("hi")
        self.ingress = paths[0]
        for i, traj in enumerate(paths[1:-1]):
            traj.next = paths[i + 2]
        self.laptraj = paths[1]

        self.mj = MinJerkLaps(
            fp=self,
            name="min_jerk",
            lap=self.laptraj,
            ingress=self.ingress,
            target_vel=self.cc.base_ff_speed,
            forecast_time=10,
            resolution=1,
            project_ahead=0.5
        )

        btree = BehaviorTree("cruise_control_and_curvature_test")
        btree.setroot(self.mj)
        print("set root")
        btree.setup()
        print("setup")
        btree.initialize()
        print("intialized")

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

    def _stop_laps_cb(self, msg: Bool) -> None:
        self.mj.again = not bool(msg.data)

    def _outer_override_cb(self, msg: Float32) -> None:
        self.cc.outer_current_override = msg.data if msg.data > 0.0 else None

    # def _request_offboard(self):
    #     offboard_msg = OffboardControlMode()
    #     offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
    #     offboard_msg.position = False
    #     offboard_msg.velocity = True 
    #     offboard_msg.acceleration = False
    #     offboard_msg.attitude = False
    #     offboard_msg.body_rate = False
    #     self._offboard_ctrl_publisher.publish(offboard_msg)

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

    def _inner_cb(self):
        if self.cc is None:
            return  # controller not initialized yet

        # check for completion
        distance_flown = self.mj.laps_completed * self.mj.duration + self.mj.pathtime + (self.mj.ingress_duration if self.mj.ingress_removed else 0)
        if distance_flown > 200:
            projected_cap_used = self.cc.discharged_ah_corrected + (self.cc.discharged_ah_corrected / distance_flown) * (2*self.mj.duration - self.mj.pathtime)
            self.get_logger().info(f"Predicted laps: {self.ah_limit / (self.cc.discharged_ah_corrected / distance_flown) / self.mj.duration:.2f}")
            if projected_cap_used > self.ah_limit:
                self.get_logger().warn(f"Projected Ah after next lap: {projected_cap_used:.3f} Ah, which is < {self.ah_limit}Ah limit. Consider concluding.")
        if not self.mj.again and (self.mj.pathtime > self.mj.duration - self.mj.project_ahead):
            self.get_logger().info("Path completed")
            self.inner_timer.cancel()
            self.inner_timer = None
            self.outer_timer.cancel()
            self.outer_timer = None

            self.target_pos = list(self.traj.path(self.mj.duration))
            self.target_vel = [0.0, 0.0, 0.0]
            self._pub_traj_setpoint()

        self.target_cruise_spd = self.cc.inner_current_ctrl(self.current_draw, self.drone_vel_mag)

        self.mj.pathtime = self.mj.projection()
        vx, vy = self.mj.velocity()
        norm = (vx**2 + vy**2)**0.5
        self.target_min_jerk_spd = norm * 1.1 * self.cc.typical_cruise_spd / self.mj.target_vel # 10% wiggle room for cruise controller

        # allocator function
        self.target_spd = min(self.target_cruise_spd, self.target_min_jerk_spd)

        self.target_pos = list(self.mj.position())
        self.target_vel = [vx * self.target_spd / norm, vy * self.target_spd / norm, 0.0]


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
