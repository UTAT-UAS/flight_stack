#!/usr/bin/env python3

import math
import time
import sys
import numpy as np
from std_msgs.msg import Float32
import rclpy
from flight_stack.flight_stack import FlightPlanner
from flight_stack.pather import trajectory
from flight_stack_msgs.srv import CoreCommand
from geometry_msgs.msg import Point
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition, BatteryStatus, TrajectorySetpoint
from rclpy.qos import QoSPresetProfiles

class CurrentController(FlightPlanner):

    def __init__(self):
        super().__init__()
        self.point = Point()

        # trajectory
        self.traj = trajectory.Custom(None, 0, np.array([]), 0)
        self.duration = 0
        # state vars
        self.unitialized_traj = True
        self.current_draw = 0.0
        self.capacity_consumed = 0.0
        self.drone_vel_mag = 0.0
        self.pathtime = 0
        # inner loop vars
        self.intermediate_target_velocity = 0.0  # current controller, feedforward + P + I
        self.target_velocity = 0.0  # Output of inner loop (curvature slowdown factored)
        self.integral_limit = 10.0  # m/s, max contribution of integral term to velocity setpoint
        # outer loop vars
        self.target_current_draw = 30.0 # input of outer loop
        self.current_setpoint = self.target_current_draw   # Output of outer loop, defaults to 60A
        self.max_current = 45.0
        self.min_current = 15.0
        self.base_ff_velocity = self.get_feedforward_velocity(self.current_setpoint)
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.initial_capacity_consumed = None

        # controller gains
        self.kp_inner = 0.5
        self.ki_inner = 0.1
        self.kp_outer = 20.0  # Amps to adjust per Ah error

        self.stored_integral = 0.0

        # slew rate limiter variables
        self.last_target_velocity = self.base_ff_velocity
        self.max_acceleration = 5.0      # m/s^2 

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
        self._battery_subscriber = self.create_subscription(
            BatteryStatus,
            "/fmu/out/battery_status",
            self._battery_cb,
            10,
        )

        # local position for velocity feedback
        self._local_pos_subscriber = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self._local_pos_cb,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

        # pubs
        self.goto = TrajectorySetpoint()

        # timers
        self.inner_dt = 0.01  # 100 Hz
        self.outer_dt = 0.1   # 10 Hz
        self.inner_timer = self.create_timer(self.inner_dt, self.inner_current_loop)
        self.outer_timer = self.create_timer(self.outer_dt, self.outer_capacity_loop)

    def _manual_ema(self, current_val, previous_ema, alpha):
        return (alpha * current_val) + ((1.0 - alpha) * previous_ema)
    
    def _current_cb(self, msg: Float32) -> None: # sim
        self.current_draw = msg.data

    def _capacity_cb(self, msg: Float32) -> None: # sim
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

    def _pub_traj_setpoint(self):
        #self.goto.position = [math.nan, math.nan, math.nan]  # ignore position setpoint
        self.goto.position = list(self.traj.path(self.pathtime) * 10)
        #self.goto.velocity = [self.target_velocity, 0.0, 0.0]
        self.goto.velocity = list(self.traj.velocity(self.pathtime) * self.target_velocity)
        self._traj_publisher.publish(self.goto)

    def get_feedforward_velocity(self, target_current: float) -> float:
        """
        Solves the cubic thrust curve for now: 0.003v^3 - 0.1v + 25 - I_target = 0
        This will be updated to the model we derive from experimental data.
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

    def inner_current_loop(self):
        """Update target velocity based on current tracking error and curvature."""
        if self.unitialized_traj:
            self.unitialized_traj = False
            x, y, z = self._position.x / 10, self._position.y / 10, self._position.z / 10
            paths = [
                trajectory.Line(np.array([x, y, z]), np.array([x - 110, y -545, z]), duration=556),
                trajectory.Line(np.array([x - 110, y - 545, z]), np.array([x, y, z]), duration=55.6),
            ]
            for i, traj in enumerate(paths[:-1]):
                traj.next = paths[i + 1]
                self.duration += traj.duration
            self.duration += paths[-1].duration
            self.traj = paths[0]
            self.goto.position = [self._position.x, self._position.y, self._position.z]
            self.goto.velocity = [0.0, 0.0, 0.0]
            self._pub_traj_setpoint()

            command = CoreCommand.Request()
            command.request.command = 2
            self._core_command_client.call_async(command)
            command = CoreCommand.Request()
            command.request.command = 7 # CORE_TRAJ request command
            self._core_command_client.call_async(command)

            return
        if self.pathtime > self.duration - 1:
            sys.exit()

        if self.pathtime < 556:
            error = self.current_setpoint - self.current_draw
            # error = self.current_setpoint - self.current_draw + estimate_correction(self.target_velocity, self.intermediate_target_velocity) # back calculation for anti-windup
            p_term = self.kp_inner * error

            # anti-windup by scaling integral based on how close we are to target vel
            safe_target_vel = max(self.target_velocity, 0.1) # Prevent div by 0
            vel_ratio = self.drone_vel_mag / safe_target_vel
            vel_ratio = max(0.0, min(vel_ratio, 1.0)) # Clamp between 0 and 1

            self.stored_integral += (error * self.ki_inner * self.inner_dt) * vel_ratio

            # clamp integral action
            self.stored_integral = max(-self.integral_limit, min(self.stored_integral, self.integral_limit))

            # raw target vel
            self.intermediate_target_velocity = self.base_ff_velocity + p_term + self.stored_integral
        else:
            self.intermediate_target_velocity = 1

        # projection and curvature
        self.pathtime = self.projection()
        curvature_scale = self.velocity_scale()
        self.target_velocity = self.intermediate_target_velocity * curvature_scale

        # slew rate limiter to velocity output
        max_delta = self.max_acceleration * self.inner_dt
        # calculate requested rate of change
        requested_delta = self.target_velocity - self.last_target_velocity
        # clamp request
        clamped_delta = max(-max_delta, min(requested_delta, max_delta))

        # publish slewed velocity setpoint
        self.target_velocity = self.last_target_velocity + clamped_delta
        self.last_target_velocity = self.target_velocity
        self._pub_traj_setpoint()



    def outer_capacity_loop(self):
        """Update the current setpoint from Ah tracking error."""
        if self.initial_capacity_consumed is None:
            return
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed_time_hours = (current_time - self.start_time) / 3600.0
        
        # Determine how many Ah we SHOULD have burned by this exact millisecond
        target_ah = self.target_current_draw * elapsed_time_hours
        
        net_capacity = self.capacity_consumed - self.initial_capacity_consumed
        ah_error = target_ah - net_capacity
        
        #  calculate new current to command based on Ah error
        base_current = self.target_current_draw
        commanded_current = base_current + (self.kp_outer * ah_error)
        
        # clamp current setpoint to safe limits
        self.current_setpoint = max(self.min_current, min(commanded_current, self.max_current))

        # update feedforward velocity based on new current setpoint
        self.base_ff_velocity = self.get_feedforward_velocity(self.current_setpoint)

        # print diagnostics
        self.get_logger().info(
            f"Current Draw: {self.current_draw:.2f} A, Commanded Current: {self.current_setpoint:.2f} A, Target Vel: {self.base_ff_velocity:.2f} m/s, Inner Target Vel: {self.target_velocity:.2f} m/s\nTarget Ah: {target_ah:.3f} Ah, Consumed Ah: {net_capacity:.3f} Ah, Ah Error: {ah_error:.3f} Ah"
        )

    def projection(self) -> bool:
        closest = self.pathtime
        closest_dist = 1e6
        cur_pos = np.array([self._position.x, self._position.y, self._position.z]) / 10
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
            v = self.traj.velocity(self.pathtime + (1 + dt)*7)
            slowdown += 0.4 * np.linalg.norm(v - last_v) * (2 - abs(dt)/5.05)
            last_v = v
        return max(1 / slowdown, 0.2)


def main(args=None):
    rclpy.init(args=args)
    node = CurrentController()
    
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()