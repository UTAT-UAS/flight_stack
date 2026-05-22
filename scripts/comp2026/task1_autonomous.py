import math
import numpy as np
from std_msgs.msg import Float32, Bool
import rclpy
from flight_stack.flight_stack import FlightPlanner
from flight_stack.pather import trajectory
from flight_stack.btree.actions import SetOffboard, Land, SetTrajMode
from flight_stack.btree.manager import BehaviorTree
from flight_stack.btree import utils
from px4_msgs.msg import VehicleLocalPosition, BatteryStatus, TrajectorySetpoint, OffboardControlMode
from rclpy.qos import QoSPresetProfiles
from flight_stack_msgs.srv import CoreCommand

import lap_generator as lap_generator
import lap as lap
from jerk_limited import JerkLimitedTrajectoryManager


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
        for r in real_roots:
            if r >= 0:
                return float(r)
        return 0.0

    def inner_current_ctrl(self, current_draw, drone_speed):
        error = self.current_setpoint - current_draw
        p_term = self.kp_inner * error

        safe_target_vel = max(self.target_spd, 0.1) 
        vel_ratio = drone_speed / safe_target_vel
        vel_ratio = max(0.0, min(vel_ratio, 1.0)) 

        self.stored_integral += (error * self.ki_inner * self.inner_dt) * vel_ratio
        self.stored_integral = max(-self.integral_limit, min(self.stored_integral, self.integral_limit))

        self.target_spd = self.base_ff_speed + p_term + self.stored_integral
        a = 0.0001 + 0.0003*vel_ratio*vel_ratio
        self.typical_cruise_spd = a * self.target_spd + (1-a) * self.typical_cruise_spd

        max_delta = self.max_acceleration * self.inner_dt
        requested_delta = self.target_spd - self.last_target_speed
        clamped_delta = max(-max_delta, min(requested_delta, max_delta))

        self.target_spd = self.last_target_speed + clamped_delta
        self.last_target_speed = self.target_spd

        return self.target_spd

    def outer_capacity_ctrl(self, discharged_ah, current_time):
        if self.initial_capacity_consumed is None:
            return

        elapsed_time_hours = (current_time - self.start_time) / 3600.0
        self.target_ah = self.target_current_draw * elapsed_time_hours
        self.discharged_ah_corrected = discharged_ah - self.initial_capacity_consumed
        ah_error = self.target_ah - self.discharged_ah_corrected

        base_current = self.target_current_draw
        commanded_current = base_current + (self.kp_outer * ah_error)

        self.current_setpoint = max(self.min_current, min(commanded_current, self.max_current))
        if self.outer_current_override is not None:
            self.current_setpoint = self.outer_current_override

        self.base_ff_speed = self.get_feedforward_velocity(self.current_setpoint)


class JerkLimitedLaps(utils.BTNode):
    """Drop-in BTNode replacement that hooks into the infinite 1D parametric wrapper."""
    def __init__(self, name, fp:FlightPlanner, waypoints: list, ingress_start: np.ndarray, ingress_vel: np.ndarray, target_vel:float, max_accel:float, max_jerk:float, nav_acc_rad:float):
        super().__init__(name)
        self.fp = fp
        self.waypoints = waypoints
        self.ingress_start = ingress_start
        self.ingress_vel = ingress_vel
        self.target_vel = target_vel

        self.laps_completed = 0
        self.pathtime = 0 
        
        self.manager = JerkLimitedTrajectoryManager(
            cruise_speed=target_vel,
            max_accel=max_accel,
            max_jerk=max_jerk,
            nav_acc_rad=nav_acc_rad
        )

    def initialize(self):
        super().initialize()
        # Initialize in Infinite Looping Mode
        self.manager.load_mission(self.waypoints, self.ingress_start, self.ingress_vel, is_looping=True)

    def reset(self):
        super().reset()
        self.pathtime = 0

    def request_stop(self):
        # Gracefully break the loop and brake perfectly at WP0
        self.manager.request_stop()

    def is_done(self):
        return self.manager.is_done

    def update(self, dt, external_speed_limit=None):
        self.laps_completed = self.manager.laps_completed
        self.pathtime += dt 
        return self.manager.update(dt, external_speed_limit)

    def tick(self):
        self.status = utils.STATUS.SUCCESS
        return self.status


class CruiseNode(FlightPlanner):

    def __init__(self):
        super().__init__()
        
        self.MAX_ACC = 2
        self.TURN_RADIUS = lap_generator.OVERSHOOT_RADIUS  

        # subscribers

        # this is for sim only
        self._current_subscriber = self.create_subscription(
            Float32, "sim/current_draw", self._current_cb, 10)
        self._capacity_subscriber = self.create_subscription(
            Float32, "sim/discharged_mah", self._capacity_cb, 10)
        self._local_pos_subscriber = self.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position", self._local_pos_cb, QoSPresetProfiles.SENSOR_DATA.value)
        self._stop_laps_subscriber = self.create_subscription(
            Bool, "task1/stop_laps", self._stop_laps_cb, QoSPresetProfiles.SENSOR_DATA.value)
        self._outer_override_subscriber = self.create_subscription(
            Float32, "/task1/current_override", self._outer_override_cb, QoSPresetProfiles.SENSOR_DATA.value)

        self.traj_sp = TrajectorySetpoint()
        self.target_pos = [0.0, 0.0, 0.0]
        self.target_vel = [0.0, 0.0, 0.0]
        self.target_yaw = math.nan

        self.ah_limit = 7.0

        ### Current controller setup ###
        self.capacity_consumed = 0.0
        self.initial_capacity_consumed = None
        self.current_draw = 0.0
        self.drone_vel_mag = 0.0
        self.target_cruise_spd = 0.0

        self.cc = None
        self.inner_timer = None
        self.outer_timer = None

        self._startup_timer = self.create_timer(0.1, self._initialize)

        ### Trajectory generation setup ###
        self.mj = None
        self.traj = None
        self.target_min_jerk_spd = 0
        self.target_spd = 0

        self._traj_timer = self.create_timer(0.01, self._pub_traj_setpoint)

    def _initialize(self):
        if self.initial_capacity_consumed is None:
            self.get_logger().info("Waiting for initial capacity reading...")
            return  
        
        x, y, z = self._position.x, self._position.y, self._position.z
        if x==0 or y==0 or z==0:
            self.get_logger().info(f"Invalid position: {x}, {y}, {z}")
            return

        self._initialize_current_controller()
        self._initialize_minjerk_controller()

        self.target_pos = [self._position.x, self._position.y, self._position.z]
        self.target_vel = [0.0, 0.0, 0.0]
        self.target_yaw = math.nan
        self.traj_sp.yawspeed = math.nan
        self._pub_traj_setpoint()

        command = CoreCommand.Request()
        command.request.command = 7 
        self._core_command_client.call_async(command)

        self.inner_timer = self.create_timer(self.cc.inner_dt, self._inner_cb)
        self.outer_timer = self.create_timer(self.cc.outer_dt, self._outer_cb)

        req_offboard = SetOffboard(name="request_offboard", fp=self)
        req_offboard.tick()
        set_traj_mode = SetTrajMode(name="set_traj_mode", fp=self)
        set_traj_mode.tick()

        self._startup_timer.cancel()
        self._startup_timer = None

    def _initialize_current_controller(self):
        start_time = self.get_clock().now().nanoseconds / 1e9
        self.cc = CurrentController(start_time, self.initial_capacity_consumed)

    def _initialize_minjerk_controller(self):
        curr_x = self._position.x
        curr_y = self._position.y
        curr_z = self._position.z

        drone_lat = self._position.ref_lat
        drone_lon = self._position.ref_lon
        wp0_lat = lap.first_wp.latitude
        wp0_lon = lap.first_wp.longitude
        
        R_EARTH = 6378137.0
        delta_lat = math.radians(wp0_lat - drone_lat)
        delta_lon = math.radians(wp0_lon - drone_lon)
        dist_n = delta_lat * R_EARTH
        dist_e = delta_lon * R_EARTH * math.cos(math.radians(drone_lat))
        
        offset = np.array([dist_n, dist_e, curr_z])
        
        # Only pass the base waypoints! No unrolling necessary.
        transformed_wps = [wp + offset for wp in lap.waypoints]

        self.mj = JerkLimitedLaps(
            name="min_jerk",
            fp=self,
            waypoints=transformed_wps,
            ingress_start=np.array([curr_x, curr_y, curr_z]),
            ingress_vel=np.array([self._position.vx, self._position.vy, self._position.vz]),
            target_vel=self.cc.base_ff_speed,
            max_accel=self.MAX_ACC,
            max_jerk=1.0,
            nav_acc_rad=30.0
        )

        btree = BehaviorTree("cruise_control_and_curvature_test")
        btree.setroot(self.mj)
        btree.setup()
        btree.initialize()

    def _manual_ema(self, current_val, previous_ema, alpha):
        return (alpha * current_val) + ((1.0 - alpha) * previous_ema)

    def _current_cb(self, msg: Float32) -> None:
        self.current_draw = self._manual_ema(msg.data, self.current_draw, 0.1)

    def _capacity_cb(self, msg: Float32) -> None:
        self.capacity_consumed = msg.data / 1000.0  
        if self.initial_capacity_consumed is None:  
            self.initial_capacity_consumed = self.capacity_consumed

    def _battery_cb(self, msg: BatteryStatus) -> None:
        self.current_draw = self._manual_ema(msg.current_a, self.current_draw, 0.1)
        self.capacity_consumed = msg.discharged_mah / 1000.0  
        if self.initial_capacity_consumed is None:
            self.initial_capacity_consumed = self.capacity_consumed

    def _local_pos_cb(self, msg: VehicleLocalPosition) -> None:
        self.drone_vel_mag = math.sqrt(msg.vx**2 + msg.vy**2)

    def _stop_laps_cb(self, msg: Bool) -> None:
        # Trigger the brake sequence via the new API
        if msg.data and hasattr(self, 'mj'):
            self.mj.request_stop()

    def _outer_override_cb(self, msg: Float32) -> None:
        self.cc.outer_current_override = msg.data if msg.data > 0.0 else None

    def _pub_traj_setpoint(self):
        self.traj_sp.position = self.target_pos
        self.traj_sp.velocity = self.target_vel
        
        if hasattr(self, 'target_acc'):
            self.traj_sp.acceleration = self.target_acc
            
        self.traj_sp.yaw = self.target_yaw
        self._traj_publisher.publish(self.traj_sp)

    def _outer_cb(self):
        if self.cc is None:
            return  

        self.cc.outer_capacity_ctrl(self.capacity_consumed, self.get_clock().now().nanoseconds / 1e9)

        distance_flown = self.mj.manager.total_distance_flown
        if distance_flown > 200:
            avg_lap_dist = distance_flown / max(1, self.mj.laps_completed)
            projected_cap_used = self.cc.discharged_ah_corrected + (self.cc.discharged_ah_corrected / distance_flown) * avg_lap_dist
            self.get_logger().info(f"Current laps: {self.mj.laps_completed}, Predicted laps: {self.ah_limit / (self.cc.discharged_ah_corrected / distance_flown) / avg_lap_dist:.2f}")
            if projected_cap_used > self.ah_limit:
                self.get_logger().warn(f"Projected Ah after next lap: {projected_cap_used:.3f} Ah, which is < {self.ah_limit}Ah limit. Consider concluding.")

        self.get_logger().info(f"Pathtime: {self.mj.pathtime:.2f}, Current Draw: {self.current_draw:.2f} A, Commanded Current: {self.cc.current_setpoint:.2f} A, FF Vel: {self.cc.base_ff_speed:.2f} m/s\nTarget Ah: {self.cc.target_ah:.3f} Ah, Consumed Ah: {self.cc.discharged_ah_corrected:.3f} Ah, Ah Error: {self.cc.target_ah - self.cc.discharged_ah_corrected:.3f} Ah, Cruise Speed target: {self.target_cruise_spd:.3f}, MinJerk Speed target: {self.target_min_jerk_spd:.3f}")

    def _inner_cb(self):
        if self.cc is None:
            return  

        # Replace your arbitrary lap limit with a stop request here if desired
        if self.mj.laps_completed >= 2: 
            self.mj.request_stop()
            
        if self.mj.is_done():
            self.get_logger().info("Path completed")
            self.inner_timer.cancel()
            self.inner_timer = None
            self.outer_timer.cancel()
            self.outer_timer = None

            self.target_pos = list(self.mj.manager.current_pos)
            self.target_vel = [0.0, 0.0, 0.0]
            self.target_acc = [0.0, 0.0, 0.0]
            self._pub_traj_setpoint()

            land = Land(name="land", fp=self)
            land.tick()
            return

        self.target_cruise_spd = self.cc.inner_current_ctrl(self.current_draw, self.drone_vel_mag)

        sp_px, sp_py, sp_vx, sp_vy, sp_ax, sp_ay = self.mj.update(self.cc.inner_dt, external_speed_limit=self.target_cruise_spd)
        
        norm = (sp_vx**2 + sp_vy**2)**0.5
        self.target_min_jerk_spd = norm 
        self.target_spd = norm 

        self.target_pos = [sp_px, sp_py, self.mj.manager.mission_z]
        self.target_vel = [sp_vx, sp_vy, 0.0]
        FF_GAIN = 0.7
        self.target_acc = [sp_ax * FF_GAIN, sp_ay * FF_GAIN, 0.0]
        
        if self.target_spd > 0.001:
            target_yaw = math.atan2(sp_vy, sp_vx)
            if getattr(self, 'sp_yaw', None) is None:
                self.sp_yaw = getattr(self._position, 'yaw', getattr(self._position, 'heading', target_yaw))
                
            MAX_YAW_RATE = math.radians(45.0)
            yaw_err = (target_yaw - self.sp_yaw + math.pi) % (2 * math.pi) - math.pi
            yaw_step = np.clip(yaw_err, -MAX_YAW_RATE * self.cc.inner_dt, MAX_YAW_RATE * self.cc.inner_dt)
            
            self.sp_yaw += yaw_step
            self.sp_yaw = (self.sp_yaw + math.pi) % (2 * math.pi) - math.pi
            self.target_yaw = self.sp_yaw
        else:
            self.target_yaw = math.nan
            self.sp_yaw = None

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
