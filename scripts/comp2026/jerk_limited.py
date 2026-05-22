import math
import numpy as np

class PX4PositionSmoother:
    """Mimics PX4's geometric waypoint blending."""
    def __init__(self, mpc_xy_cruise=15.0, mpc_acc_hor=3.0, nav_acc_rad=10.0):
        self.cruise_speed = mpc_xy_cruise
        self.max_accel = mpc_acc_hor
        self.acc_rad = nav_acc_rad

    def calculate_turn_speed(self, turn_radius: float) -> float:
        safe_v = math.sqrt(self.max_accel * turn_radius)
        return min(self.cruise_speed, safe_v)

    def generate_mission(self, waypoints: list[np.ndarray]):
        if len(waypoints) < 2: return []
        trajectories = []
        current_pos = waypoints[0]

        for i in range(1, len(waypoints) - 1):
            wp_prev = waypoints[i-1]
            wp_curr = waypoints[i]
            wp_next = waypoints[i+1]

            v_in = wp_curr - wp_prev
            v_out = wp_next - wp_curr
            dir_in = v_in / np.linalg.norm(v_in)
            dir_out = v_out / np.linalg.norm(v_out)

            dot_prod = np.clip(np.dot(-dir_in, dir_out), -1.0, 1.0)
            corner_angle = math.acos(dot_prod)

            if corner_angle > math.radians(175): continue

            cross_prod = dir_in[0]*dir_out[1] - dir_in[1]*dir_out[0]
            turn_dir = 1.0 if cross_prod >= 0 else -1.0

            dist_to_corner = min(np.linalg.norm(v_in)/2.0, self.acc_rad)
            corner_start = wp_curr - (dir_in * dist_to_corner)
            corner_end = wp_curr + (dir_out * dist_to_corner)

            turn_radius = dist_to_corner * math.tan(corner_angle / 2.0)
            corner_speed = self.calculate_turn_speed(turn_radius)

            if np.linalg.norm(corner_start - current_pos) > 0.1:
                length = np.linalg.norm(corner_start - current_pos)
                trajectories.append({
                    'type': 'line', 'start': current_pos.copy(), 'end': corner_start.copy(),
                    'dir': (corner_start - current_pos)/length, 'length': length,
                    'target_v': self.cruise_speed
                })

            norm_in = np.array([-dir_in[1] * turn_dir, dir_in[0] * turn_dir, 0.0])
            center = corner_start + norm_in * turn_radius
            start_angle = math.atan2(corner_start[1] - center[1], corner_start[0] - center[0])
            arc_length = turn_radius * (math.pi - corner_angle)

            trajectories.append({
                'type': 'arc', 'control_point': wp_curr.copy(), 'center': center, 
                'radius': turn_radius, 'start_angle': start_angle, 'turn_dir': turn_dir,
                'length': arc_length, 'target_v': corner_speed
            })
            current_pos = corner_end

        length = np.linalg.norm(waypoints[-1] - current_pos)
        trajectories.append({
            'type': 'line', 'start': current_pos.copy(), 'end': waypoints[-1].copy(),
            'dir': (waypoints[-1] - current_pos)/length if length > 0 else np.array([1.0, 0.0, 0.0]), 
            'length': length, 'target_v': self.cruise_speed 
        })
        return trajectories


class PX4VelocitySmoother:
    """Robust 1D Jerk-Limited S-Curve Generator."""
    def __init__(self, max_jerk=2.0, max_accel=3.0):
        self.j_max = max_jerk
        self.a_max = max_accel
        self.a_curr = 0.0
        self.v_curr = 0.0

    def update(self, dt: float, v_target: float) -> tuple[float, float]:
        v_err = v_target - self.v_curr
        
        if v_target == 0.0 and self.v_curr < 0.05 and abs(self.a_curr) < 0.1:
            self.v_curr = 0.0
            self.a_curr = 0.0
            return self.v_curr, self.a_curr
            
        v_brake = (self.a_curr ** 2) / (2.0 * self.j_max)
        
        if v_err > 0.01:
            if v_err <= v_brake and self.a_curr > 0: jerk = -self.j_max
            else: jerk = self.j_max if self.a_curr < self.a_max else 0.0
        elif v_err < -0.01:
            if abs(v_err) <= v_brake and self.a_curr < 0: jerk = self.j_max
            else: jerk = -self.j_max if self.a_curr > -self.a_max else 0.0
        else:
            jerk = -math.copysign(self.j_max, self.a_curr) if abs(self.a_curr) > 0.1 else 0.0

        self.a_curr += jerk * dt
        self.a_curr = max(-self.a_max, min(self.a_curr, self.a_max))
        self.v_curr += self.a_curr * dt
        self.v_curr = max(0.0, self.v_curr) 
        
        return self.v_curr, self.a_curr


class JerkLimitedTrajectoryManager:
    """State manager featuring a Circular Index Buffer for infinite lapping."""
    def __init__(self, cruise_speed, max_accel, max_jerk, nav_acc_rad):
        self.pos_smoother = PX4PositionSmoother(cruise_speed, max_accel, nav_acc_rad)
        self.vel_smoother = PX4VelocitySmoother(max_jerk, max_accel)
        self.max_accel = max_accel
        self.max_jerk = max_jerk
        
        self.mission_queue = []
        self.current_segment_index = 0
        self.s_current = 0.0
        self.mission_z = 0.0
        self.current_pos = np.zeros(3)
        self.core_waypoints = []
        
        self.is_looping = False
        self.loop_start_idx = 0
        self.loop_end_idx = 0
        
        self.is_done = False
        self.stop_requested = False
        self.laps_completed = 0
        self.total_distance_flown = 0.0

    def load_mission(self, raw_waypoints, initial_pos, initial_vel, is_looping=True):
        W = list(raw_waypoints)
        # Strip duplicate end waypoint if user manually closed the loop
        if np.linalg.norm(W[-1] - W[0]) < 0.1:
            W = W[:-1]
            
        self.core_waypoints = W
        self.is_looping = is_looping
        self.vel_smoother.v_curr = np.linalg.norm(initial_vel)
        self.vel_smoother.a_curr = 0.0
        self.mission_z = initial_pos[2]
        self.current_pos = initial_pos.copy()
        
        if is_looping:
            # Build an overlapping template to generate all necessary corner arcs perfectly
            template_wps = [W[-1]] + W + [W[0], W[1]]
            queue = self.pos_smoother.generate_mission(template_wps)
            
            # The first segment generated is a line. Its end is the exact start of the WP0 arc.
            # We replace this first segment with an Ingress Line straight from the drone.
            arc_w0_start = queue[0]['end'] 
            length = np.linalg.norm(arc_w0_start - initial_pos)
            
            ingress_line = {
                'type': 'line', 'start': initial_pos.copy(), 'end': arc_w0_start.copy(),
                'dir': (arc_w0_start - initial_pos) / length if length > 0 else np.array([1.0, 0.0, 0.0]),
                'length': length, 'target_v': self.pos_smoother.cruise_speed
            }
            queue[0] = ingress_line
            self.mission_queue = queue
            
            # Define the circular buffer boundaries
            self.current_segment_index = 0
            self.s_current = 0.0
            self.loop_start_idx = 2
            self.loop_end_idx = self.loop_start_idx + 2 * len(W) - 1
            
        else:
            self.mission_queue = self.pos_smoother.generate_mission(W)
            self.current_segment_index = 0
            self.s_current = 0.0
            
            if len(self.mission_queue) > 0:
                first_target = self.mission_queue[0]['start']
                length = np.linalg.norm(first_target - initial_pos)
                if length > 0.1:
                    ingress_line = {
                        'type': 'line', 'start': initial_pos.copy(), 'end': first_target.copy(),
                        'dir': (first_target - initial_pos) / length, 'length': length,
                        'target_v': self.pos_smoother.cruise_speed
                    }
                    self.mission_queue.insert(0, ingress_line)

        self.is_done = False
        self.stop_requested = False
        self.laps_completed = 0

    def request_stop(self):
        """Disables wrapping and drops a braking line exactly at WP0."""
        if self.stop_requested or self.is_done: return
        self.stop_requested = True
        
        if self.is_looping:
            self.is_looping = False
            target_wp = self.core_waypoints[0]
            
            # The segment that approaches WP0 is always right before the WP0 Arc
            target_idx = self.loop_end_idx
            
            if self.current_segment_index > target_idx:
                # We missed the stop line this lap! Brake immediately to be safe.
                self.mission_queue[self.current_segment_index]['target_v'] = 0.0
                self.mission_queue = self.mission_queue[:self.current_segment_index + 1]
            else:
                # Truncate the queue and end it at WP0 with 0 velocity
                start_pt = self.mission_queue[target_idx - 1]['end']
                length = np.linalg.norm(target_wp - start_pt)
                self.mission_queue[target_idx] = {
                    'type': 'line', 'start': start_pt.copy(), 'end': target_wp.copy(),
                    'dir': (target_wp - start_pt) / length if length > 0 else np.array([1.0, 0.0, 0.0]),
                    'length': length, 'target_v': 0.0
                }
                self.mission_queue = self.mission_queue[:target_idx + 1]

    def evaluate_segment(self, segment, s, v_mag, a_mag):
        if segment['type'] == 'line':
            pos = segment['start'] + segment['dir'] * s
            vel = segment['dir'] * v_mag
            acc = segment['dir'] * a_mag
            return pos[0], pos[1], vel[0], vel[1], acc[0], acc[1]

        elif segment['type'] == 'arc':
            r = segment['radius']
            theta = segment['start_angle'] + segment['turn_dir'] * (s / r)
            pos_x = segment['center'][0] + r * math.cos(theta)
            pos_y = segment['center'][1] + r * math.sin(theta)

            tx, ty = -math.sin(theta) * segment['turn_dir'], math.cos(theta) * segment['turn_dir']
            nx, ny = -math.cos(theta), -math.sin(theta)

            return pos_x, pos_y, tx * v_mag, ty * v_mag, tx * a_mag + nx * (v_mag**2)/r, ty * a_mag + ny * (v_mag**2)/r

    def update(self, dt, external_speed_limit=None):
        if self.is_done:
            return self.current_pos[0], self.current_pos[1], 0.0, 0.0, 0.0, 0.0
        
        current_segment = self.mission_queue[self.current_segment_index]
        
        # 1. Advance queue paramaterically with Circular Wrapping
        while self.s_current >= current_segment['length']:
            self.s_current -= current_segment['length']
            self.current_segment_index += 1

            if self.is_looping:
                # Wrap the index back to the start of the template lap
                if self.current_segment_index > self.loop_end_idx:
                    self.current_segment_index = self.loop_start_idx
                    self.laps_completed += 1
            else:
                if self.current_segment_index >= len(self.mission_queue):
                    self.s_current = current_segment['length']
                    self.is_done = True
                    return self.evaluate_segment(current_segment, self.s_current, 0.0, 0.0)
                    
            current_segment = self.mission_queue[self.current_segment_index]

        # 2. Look-ahead braking
        dist_to_end = current_segment['length'] - self.s_current
        next_v_limit = 0.0
        
        if self.is_looping and self.current_segment_index == self.loop_end_idx:
            # If looping, the segment "after" the end is the loop start
            next_v_limit = self.mission_queue[self.loop_start_idx]['target_v']
        elif self.current_segment_index + 1 < len(self.mission_queue):
            next_v_limit = self.mission_queue[self.current_segment_index + 1]['target_v']
            
        current_speed = self.vel_smoother.v_curr
        braking_dist = abs(current_speed**2 - next_v_limit**2) / (2.0 * self.max_accel) + current_speed * (self.max_accel / self.max_jerk) 
        
        if dist_to_end <= braking_dist:
            commanded_v_mag = next_v_limit
        else:
            commanded_v_mag = current_segment['target_v']

        # 3. Apply external Controller Constraints
        if external_speed_limit is not None and dist_to_end > braking_dist:
            commanded_v_mag = min(commanded_v_mag, external_speed_limit)

        # 4. Temporal Execution
        sp_v_mag, sp_a_mag = self.vel_smoother.update(dt, commanded_v_mag)
        
        step_dist = sp_v_mag * dt
        self.s_current += step_dist
        self.total_distance_flown += step_dist

        sp_px, sp_py, sp_vx, sp_vy, sp_ax, sp_ay = self.evaluate_segment(current_segment, self.s_current, sp_v_mag, sp_a_mag)
        self.current_pos = np.array([sp_px, sp_py, self.mission_z])
        
        return sp_px, sp_py, sp_vx, sp_vy, sp_ax, sp_ay
