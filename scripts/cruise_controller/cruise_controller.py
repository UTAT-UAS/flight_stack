#!/usr/bin/env python3

import math
import numpy as np

class CurrentController():

    def __init__(self, start_time=None, initial_capacity_consumed=None):
        
        # outer loop vars
        self.start_time = start_time
        self.initial_capacity_consumed = initial_capacity_consumed
        self.target_current_draw = 30.0 # input of outer loop
        self.target_ah = 0.0
        self.discharged_ah_corrected = 0.0
        self.current_setpoint = self.target_current_draw   # Output of outer loop, defaults to 60A 
        self.max_current = 45.0
        self.min_current = 15.0
        self.base_ff_speed = self.get_feedforward_velocity(self.current_setpoint)
        self.typical_cruise_spd = self.base_ff_speed
        
        self.kp_outer = 20.0  # Amps to adjust per Ah error

        # inner loop vars
        self.target_spd = 0.0  # Output of inner loop
        self.integral_limit = 5.0  # m/s, max contribution of integral term to velocity setpoint
        self.stored_integral = 0.0

        self.kp_inner = 0.5
        self.ki_inner = 0.1


        self.last_target_speed = self.base_ff_speed
        self.max_acceleration = 5.0      # m/s^2 

        # timers
        self.inner_dt = 0.01  # 100 Hz
        self.outer_dt = 0.1   # 10 Hz


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
        a = 0.001 + 0.003*vel_ratio*vel_ratio
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

        # update feedforward velocity based on new current setpoint
        self.base_ff_speed = self.get_feedforward_velocity(self.current_setpoint)