import math

from bisect import bisect_right
from typing import TypeVar, Union, List
import numpy as np

def norm(v: np.ndarray): return v / np.linalg.norm(v)

_T = TypeVar("_T", bound="Trajectory")
class Trajectory():
    def __init__(self):
        self.next = None  # type: Trajectory | None
        self.endpoint = np.array([0, 0, 0])
        self.duration = 0.0

        def path(t: Union[int, float, np.number]):
            return self.endpoint
        self.path = path

        def velocity(t: Union[int, float, np.number]):
            return self.endpoint
        self.velocity = velocity
    
    def append(self, x: _T):
        nxt = self.next
        while nxt is not None:
            nxt = nxt.next
        nxt.next = x
    
    def prepend(self, x: _T):
        x.next = self
        return x

class Custom(Trajectory):
    def __init__(self, path, velocity, endpoint: np.ndarray, duration: Union[int, float, np.number]):
        super().__init__()
        self.path = path
        self.velocity = velocity
        self.endpoint = endpoint
        self.duration = duration

class Wrapper(Trajectory):
    def __init__(self, child: Trajectory):
        self.__dict__['child'] = child

    def __getattr__(self, name):
        return getattr(self.child, name)

    def __setattr__(self, name, val):
        setattr(self.child, name, val)

    def replace_child(self, child: Trajectory):
        self.__dict__['child'] = child

class Pause(Trajectory):
    def __init__(self, loc: np.ndarray, duration: Union[int, float, np.number]):
        super().__init__()
        def path(t: Union[int, float, np.number]):
            return loc

        zero_v = np.array([0 for _ in range(len(loc))])
        def velocity(t: Union[int, float, np.number]):
            return zero_v

        self.path = path
        self.velocity = velocity
        self.endpoint = loc
        self.duration = duration

class Line(Trajectory):
    def __init__(
            self,
            start: np.ndarray,
            end: np.ndarray,
            duration: Union[int, float, np.number]):
        super().__init__()

        delta = end - start
        def path(t: Union[int, float, np.number]):
            if t < 0: return start
            if t < duration:
                return start + delta * t / duration
            if self.next: return self.next.path(t - duration)
            return end

        const_vel = delta / duration
        zero_v = np.array([0 for _ in range(len(delta))])
        def velocity(t: Union[int, float, np.number]):
            if t < 0: return zero_v
            if t < duration:
                return const_vel
            if self.next: return self.next.velocity(t - duration)
            return zero_v

        self.path = path
        self.velocity = velocity
        self.endpoint = end
        self.duration = duration

class Circle(Trajectory):
    def __init__(
            self,
            start: np.ndarray,
            center: np.ndarray,  # position vector to axis
            cycles: Union[int, float, np.number],
            axis: np.ndarray = np.array([0,0,1]),
            speed: Union[int, float, np.number] = 1):
        super().__init__()

        axis = axis / np.linalg.norm(axis)
        center = center + np.vdot(start - center, axis) * axis
        diff_orth = start - center
        w = np.array([  # Cross product only works in 3D
            axis[1]*diff_orth[2] - axis[2]*diff_orth[1],
            axis[2]*diff_orth[0] - axis[0]*diff_orth[2],
            axis[0]*diff_orth[1] - axis[1]*diff_orth[0],
        ])
        dilation = speed / np.linalg.norm(diff_orth)

        endpoint = center + math.cos(2 * math.pi * cycles) * diff_orth + math.sin(2 * math.pi * cycles) * w
        duration = 2 * math.pi * cycles / dilation
        def path(t: Union[int, float, np.number]):
            if t < 0: return start
            if t < duration:
                return center + math.cos(dilation * t) * diff_orth + math.sin(dilation * t) * w
            if self.next: return self.next.path(t - duration)
            return endpoint

        zero_v = np.array([0 for _ in range(len(start))])
        def velocity(t: Union[int, float, np.number]):
            if t < 0: return zero_v
            if t < duration:
                return dilation * (-math.sin(dilation * t) * diff_orth + math.cos(dilation * t) * w)
            if self.next: return self.next.velocity(t - duration)
            return zero_v

        self.path = path
        self.velocity = velocity
        self.endpoint = endpoint
        self.duration = duration

class Ellipse(Trajectory):
    def __init__(
            self,
            start: np.ndarray,
            center: np.ndarray,  # position vector to axis
            cross_axis: np.ndarray,  # cross-axis vector relative to axis
            duration: Union[int, float, np.number],
            axis: np.ndarray = np.array([0,0,1]),  # does not affect cw/ccw motion, will always go towards cross_axis
            max_speed: Union[int, float, np.number] = 1):
        super().__init__()

        axis = axis / np.linalg.norm(axis)
        center = center + np.vdot(start - center, axis) * axis
        diff_orth = start - center
        _matrix = np.array([diff_orth, cross_axis])
        major_axis_magnitude = max(np.linalg.svd(_matrix, compute_uv=False))
        dilation = max_speed / major_axis_magnitude

        endpoint = center + math.cos(dilation * duration) * diff_orth + math.sin(dilation * duration) * cross_axis
        def path(t: Union[int, float, np.number]):
            if t < 0: return start
            if t < duration:
                return center + math.cos(dilation * t) * diff_orth + math.sin(dilation * t) * cross_axis
            if self.next: return self.next.path(t - duration)
            return endpoint

        zero_v = np.array([0 for _ in range(len(start))])
        def velocity(t: Union[int, float, np.number]):
            if t < 0: return zero_v
            if t < duration:
                return dilation * (-math.sin(dilation * t) * diff_orth + math.cos(dilation * t) * cross_axis)
            if self.next: return self.next.velocity(t - duration)
            return zero_v

        self.path = path
        self.velocity = velocity
        self.endpoint = endpoint
        self.duration = duration

class Spiral(Trajectory):
    def __init__(
            self,
            start: np.ndarray,  # cannot be same as center, otherwise starting direction is undetermined
            center: np.ndarray,  # position vector to axis
            end_radius: Union[int, float, np.number] = 0,
            axis: np.ndarray = np.array([0,0,1]),
            spiral_gap: Union[int, float, np.number] = 6,
            speed: Union[int, float, np.number] = 1):
        super().__init__()

        axis = axis / np.linalg.norm(axis)
        center = center + np.vdot(start - center, axis) * axis
        diff_orth = (start - center) / np.linalg.norm(start - center)  # undetermined when start same as center
        w = np.array([  # Cross product only works in 3D
            axis[1]*diff_orth[2] - axis[2]*diff_orth[1],
            axis[2]*diff_orth[0] - axis[0]*diff_orth[2],
            axis[0]*diff_orth[1] - axis[1]*diff_orth[0],
        ])

        spiral_gap /= math.pi
        radius = np.linalg.norm(start - center)
        theta_start = radius / spiral_gap
        theta_end = end_radius / spiral_gap
        arclength = spiral_gap / 2 * abs((theta_start * (1 + theta_start ** 2) ** 0.5 +
                                          math.log(theta_start + (1 + theta_start ** 2) ** 0.5)) -
                                     (theta_end * (1 + theta_end ** 2) ** 0.5 +
                                          math.log(theta_end + (1 + theta_end ** 2) ** 0.5)))
        theta_start_sq = theta_start ** 2
        speed_norm = speed * abs(theta_end ** 2 - theta_start_sq) / arclength

        duration = arclength / speed
        endpoint = spiral_gap * theta_end * (math.cos(theta_end - theta_start) * diff_orth + math.sin(theta_end - theta_start) * w) + center

        zero_v = np.array([0 for _ in range(len(start))])
        if theta_start > theta_end:
            def path(t: Union[int, float, np.number]):
                if t < 0: return start
                if t < duration:
                    theta = (theta_start_sq - t * speed_norm) ** 0.5
                    return spiral_gap * theta * (math.cos(theta - theta_start) * diff_orth + math.sin(theta - theta_start) * w) + center
                if self.next: return self.next.path(t - duration)
                return endpoint

            def velocity(t: Union[int, float, np.number]):
                if t < 0: return zero_v
                if t < duration:
                    theta = (theta_start_sq - t * speed_norm) ** 0.5
                    return spiral_gap * 0.5 * -speed_norm * ((math.cos(theta - theta_start) * diff_orth + math.sin(theta - theta_start) * w) / theta +
                                                             (-math.sin(theta - theta_start) * diff_orth + math.cos(theta - theta_start) * w))
                if self.next: return self.next.velocity(t - duration)
                return zero_v

        else:
            def path(t: Union[int, float, np.number]):
                if t < 0: return start
                if t < duration:
                    theta = (theta_start_sq + t * speed_norm) ** 0.5
                    return spiral_gap * theta * (math.cos(theta - theta_start) * diff_orth + math.sin(theta - theta_start) * w) + center
                if self.next: return self.next.path(t - duration)
                return endpoint

            def velocity(t: Union[int, float, np.number]):
                if t < 0: return zero_v
                if t < duration:
                    theta = (theta_start_sq + t * speed_norm) ** 0.5
                    return spiral_gap * 0.5 * speed_norm * ((math.cos(theta - theta_start) * diff_orth + math.sin(theta - theta_start) * w) / theta +
                                                            (-math.sin(theta - theta_start) * diff_orth + math.cos(theta - theta_start) * w))
                if self.next: return self.next.velocity(t - duration)
                return zero_v

        self.path = path
        self.velocity = velocity
        self.endpoint = endpoint
        self.duration = duration

class Rectangle(Trajectory):
    def __init__(
            self,
            start: np.ndarray,
            opposite: np.ndarray,  #opposite corner
            duration: Union[int, float, np.number],
            clockwise: bool=True):
        super().__init__()

        # 4 possible situations: {x = +, y = +}, {x = -, y = -}, {x = -, y = +}, {x = +, y = -}
        zero_v = np.array([0 for _ in range(len(start))])
        delta_x, delta_y = zero_v.copy(), zero_v.copy()
        delta_x[0] = opposite[0] - start[0]
        delta_y[1] = opposite[1] - start[1]

        x_dur = float(abs(delta_x[0])/(abs(delta_x[0]) + abs(delta_y[1]))) * (duration/2)  # type: float
        y_dur = (duration/2) - x_dur  # type: float

        if clockwise ^ (delta_x[0] < 0) ^ (delta_y[1] < 0): # XOR statement which covers all 8 cases
            lines = [
                Line(start=start, end=start + delta_y, duration=y_dur),
                Line(start=start + delta_y, end=opposite, duration=x_dur),
                Line(start=opposite, end=opposite - delta_y, duration=y_dur),
                Line(start=opposite - delta_y, end=start, duration=x_dur),
            ]
            t_cumul = [0, y_dur, y_dur+x_dur, duration-x_dur, duration]
        else:
            lines = [
                Line(start=start, end=start + delta_x, duration=x_dur),
                Line(start=start + delta_x, end=opposite, duration=y_dur),
                Line(start=opposite, end=opposite - delta_x, duration=x_dur),
                Line(start=opposite - delta_x, end=start, duration=y_dur),
            ]
            t_cumul = [0, x_dur, x_dur+y_dur, duration-y_dur, duration]

        def path(t: Union[int, float, np.number]):
            if t < duration:
                for i in range(4):
                    if t < t_cumul[i + 1]:
                        return lines[i].path(t - t_cumul[i])
            if self.next: return self.next.path(t - duration)
            return start

        def velocity(t: Union[int, float, np.number]):
            if t < duration:
                for i in range(4):
                    if t < t_cumul[i + 1]:
                        return lines[i].velocity(t - t_cumul[i])
            if self.next: return self.next.velocity(t - duration)
            return zero_v

        self.path = path
        self.velocity = velocity
        self.endpoint = start
        self.duration = duration

class Square(Trajectory): ##start node is bottom left
    def __init__(self,
        start: np.ndarray,
        length: Union[int, float, np.number],
        duration: Union[int, float, np.number],
        clockwise: bool=True):

        super().__init__()

        opposite = start.copy()
        opposite[0] += length
        opposite[1] += length

        square = Rectangle(start=start, opposite=opposite, duration=duration, clockwise=clockwise)

        self.path = square.path
        self.velocity = square.velocity
        self.endpoint = start
        self.duration = duration

class Amongus(Trajectory):
    def __init__(self,
                 start: np.ndarray,
                 v_dir: np.ndarray=np.array([0, -1, 0]),  # vector from top to bottom, normal to baseline
                 h_dir: np.ndarray=np.array([-1, 0, 0]),  # vector from right to left, normal to height
                 scale: Union[int, float, np.number]=100):  # height
        super().__init__()
        v_dir = v_dir / np.linalg.norm(v_dir)
        h_dir = h_dir / np.linalg.norm(h_dir)

        r = scale / 3
        x = scale / 10
        y = scale * 5 / 18

        ellipse_dur = self.ellipse_duration(start=start, center=start - x*v_dir, cross_axis=3*x*h_dir, axis=np.array([0,0,1]), max_velocity=1)
        trajs = [
            Ellipse(start=start, center=start + x*v_dir, cross_axis=3*x*h_dir, duration=ellipse_dur),
            Circle(start=start, center=start + r*h_dir, cycles=0.5)
        ]
        trajs.append(Circle(start=trajs[-1].endpoint, center=trajs[-1].endpoint + r/3*v_dir, cycles=0.25))
        trajs.append(Line(start=trajs[-1].endpoint, end=trajs[-1].endpoint + y*v_dir, duration=y))
        trajs.append(Circle(start=trajs[-1].endpoint, center=trajs[-1].endpoint - r/3*h_dir, cycles=0.25))
        trajs.append(Circle(start=trajs[-1].endpoint, center=trajs[-1].endpoint - r/2*h_dir, cycles=0.5))
        trajs.append(Circle(start=trajs[-1].endpoint, center=trajs[-1].endpoint - r/2*h_dir, cycles=0.5))
        trajs.append(Line(start=trajs[-1].endpoint, end=trajs[-1].endpoint - 3*x*v_dir, duration=3*x))
        trajs.append(Ellipse(start=trajs[-1].endpoint, center=trajs[-1].endpoint - x*v_dir, cross_axis=3*x*h_dir, duration=ellipse_dur/2))

        for i, traj in enumerate(trajs[:-1]):
            traj.next = trajs[i + 1]
        self.path = trajs[0].path
        self.velocity = trajs[0].velocity
        self.duration = sum(t.duration for t in trajs)
        self.endpoint = trajs[-1].endpoint

    def ellipse_duration(self, start: np.ndarray, center: np.ndarray, cross_axis: np.ndarray, axis: np.ndarray, max_velocity: Union[int, float, np.number]):
        axis = axis / np.linalg.norm(axis)
        center = center + np.vdot(start - center, axis) * axis
        diff_orth = start - center
        _matrix = np.array([diff_orth, cross_axis])
        major_axis_magnitude = max(np.linalg.svd(_matrix, compute_uv=False))
        dilation = max_velocity / major_axis_magnitude
        return 2 * math.pi / dilation

class AmongusHigherRes(Trajectory):
    def __init__(self,
                 start: np.ndarray,
                 v_dir: np.ndarray=np.array([0, -1, 0]),  # vector from top to bottom, normal to baseline
                 h_dir: np.ndarray=np.array([-1, 0, 0]),  # vector from right to left, normal to height
                 scale: Union[int, float, np.number]=200):  # height
        super().__init__()
        v_dir = v_dir / np.linalg.norm(v_dir)
        h_dir = h_dir / np.linalg.norm(h_dir)
        normal = -np.array([  # Cross product only works in 3D
            v_dir[1] * h_dir[2] - v_dir[2] * h_dir[1],
            v_dir[2] * h_dir[0] - v_dir[0] * h_dir[2],
            v_dir[0] * h_dir[1] - v_dir[1] * h_dir[0],
        ])

        r = scale / 5
        x = scale / 10
        y = scale * 3 / 30
        z = scale * 4 / 30

        ellipse_dur_goggles = self.ellipse_duration(start=start, center=start + x*v_dir + 1.5*x*h_dir, cross_axis=x*v_dir - 1.5*x*h_dir, axis=np.array([0,0,1]), max_velocity=1)
        ellipse_dur_scalp = self.ellipse_duration(start=start, center=start + 1.2*r*h_dir, cross_axis=-r*v_dir, axis=np.array([0,0,1]), max_velocity=1)
        # Head
        trajs = [
            #Ellipse(start=start, center=start + x*v_dir, cross_axis=3*x*h_dir, duration=ellipse_dur_goggles),
            Ellipse(start=start, center=start + x*v_dir + 1.5*x*h_dir, cross_axis=x*v_dir - 1.5*x*h_dir, duration=ellipse_dur_goggles),
            Ellipse(start=start, center=start + 1.2*r*h_dir, cross_axis=-r*v_dir, duration=ellipse_dur_scalp/2)
        ]  # type: List[Trajectory]
        # Back
        trajs.append(Circle(start=trajs[-1].endpoint, center=trajs[-1].endpoint + 2*r/3*v_dir, cycles=0.25, axis=normal))
        trajs.append(Line(start=trajs[-1].endpoint, end=trajs[-1].endpoint + y*v_dir, duration=y))
        trajs.append(Circle(start=trajs[-1].endpoint, center=trajs[-1].endpoint - 2*r/3*h_dir, cycles=0.25, axis=normal))
        trajs.append(Line(start=trajs[-1].endpoint, end=trajs[-1].endpoint + z*v_dir, duration=z))
        # Legs
        trajs.append(Circle(start=trajs[-1].endpoint, center=trajs[-1].endpoint - 0.9*r/2*h_dir, cycles=0.5, axis=normal))
        trajs.append(Line(start=trajs[-1].endpoint, end=trajs[-1].endpoint - z*v_dir, duration=z))
        trajs.append(Line(start=trajs[-1].endpoint, end=trajs[-1].endpoint - 0.6*r*h_dir, duration=0.6*r))
        trajs.append(Line(start=trajs[-1].endpoint, end=trajs[-1].endpoint + z*v_dir, duration=z))
        trajs.append(Circle(start=trajs[-1].endpoint, center=trajs[-1].endpoint - 0.9*r/2*h_dir, cycles=0.5, axis=normal))
        # Return
        trajs.append(Line(start=trajs[-1].endpoint, end=trajs[-1].endpoint - 3*x*v_dir, duration=3*x))
        trajs.append(Ellipse(start=trajs[-1].endpoint, center=trajs[-1].endpoint - x*v_dir + 1.5*x*h_dir, cross_axis=-x*v_dir - 1.5*x*h_dir, duration=ellipse_dur_goggles/4))

        for i, traj in enumerate(trajs[:-1]):
            traj.next = trajs[i + 1]
        self.path = trajs[0].path
        self.velocity = trajs[0].velocity
        self.duration = sum(t.duration for t in trajs)
        self.endpoint = trajs[-1].endpoint

    def ellipse_duration(self, start: np.ndarray, center: np.ndarray, cross_axis: np.ndarray, axis: np.ndarray, max_velocity: Union[int, float, np.number]):
        axis = axis / np.linalg.norm(axis)
        center = center + np.vdot(start - center, axis) * axis
        diff_orth = start - center
        _matrix = np.array([diff_orth, cross_axis])
        major_axis_magnitude = max(np.linalg.svd(_matrix, compute_uv=False))
        dilation = max_velocity / major_axis_magnitude
        return 2 * math.pi / dilation

class RouteHandler(Trajectory):
    def __init__(self, start: np.ndarray = np.array([0,0,0])):
        super().__init__()
        self.trajectories = []  # type: List[Trajectory]
        self.ids = []  # type: List[str]
        self.time_start = []  # type: List[Union[int, float, np.number]]
        self.time_end = []  # type: List[float]
        self.last_pos = [start]  # type: List[np.ndarray]
        self.zero_v = np.array([0 for _ in range(len(start))])

        self.path = self._path
        self.velocity = self._velocity
        self.endpoint = start

    def _path(self, t: Union[int, float, np.number]):
        m = bisect_right(self.time_end, t)
        pos = self.last_pos[m].copy()
        for i in range(m, len(self.trajectories)):
            if self.time_start[i] < t:
                pos = pos + self.trajectories[i].path(t - self.time_start[i])
        return pos

    def _velocity(self, t: Union[int, float, np.number]):
        m = bisect_right(self.time_end, t)
        vel = self.zero_v.copy()
        for i in range(m, len(self.trajectories)):
            if self.time_start[i] < t:
                vel = vel + self.trajectories[i].velocity(t - self.time_start[i])
        return vel

    def add_traj(self, traj: Trajectory, start_time: Union[int, float, np.number], id: str=''):
        i = bisect_right(self.time_end, start_time + traj.duration)
        self.trajectories.insert(i, traj)
        self.ids.insert(i, id)
        self.time_start.insert(i, start_time)
        self.time_end.insert(i, start_time + traj.duration)
        self.last_pos.insert(i + 1, traj.endpoint + self.last_pos[i])
        for lp in self.last_pos[i + 2:]:  # update psa here instead of in the path function
            lp += traj.endpoint

        self.endpoint = self.last_pos[-1]
        self.duration = self.time_end[-1]

        # RouteHandler depends on each trajectory starting at origin, would have to mutate object to normalize
        if traj.next:
            print(f"Object detected in {type(traj)}.next but will not be used by RouteHandler")

    def pop_traj(self, traj: Trajectory):
        i = 0
        while i < len(self.trajectories):
            if traj == self.trajectories[i]:
                self.pop_i(i)
            else:
                i += 1

    def pop_id(self, id: str):
        i = 0
        while i < len(self.trajectories):
            if id == self.ids[i]:
                self.pop_i(i)
            else:
                i += 1

    def pop_i(self, i):
        self.time_start.pop(i)
        self.time_end.pop(i)
        self.ids.pop(i)
        self.last_pos.pop(i + 1)
        for lp in self.last_pos[i + 1:]:
            lp -= self.trajectories[i].endpoint
        self.trajectories.pop(i)

        self.endpoint = self.last_pos[-1]
        self.duration = self.time_end[-1] if self.time_end else 0
