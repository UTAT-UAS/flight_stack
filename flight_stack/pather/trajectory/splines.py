import math
from bisect import bisect_right
from typing import Union, List, Optional
import numpy as np
from .trajectories import Trajectory, norm

class SplineNode:
    def __init__(self, p: np.ndarray, d0: np.ndarray, d1: np.ndarray, duration: Union[int, float, np.number]):
        self.p = p
        self.d0 = d0  # incoming
        self.d1 = d1  # outgoing
        self.duration = duration

def bezier(a: np.ndarray, av: np.ndarray, b: np.ndarray, bv: np.ndarray, t: Union[int, float, np.number], duration: Union[int, float, np.number]):
    frac = t / duration
    i1 = a + av * frac
    i2 = b + bv * (1 - frac)
    return i1 + (i2 - i1) * frac

def cubic_bezier(a: SplineNode, b: SplineNode, t: Union[int, float, np.number]):
    c = a.p + a.d1
    frac = t / a.duration
    i1 = a.p + a.d1 * frac
    i2 = c + (b.p + b.d0 - c) * frac
    i3 = b.p + b.d0 * (1 - frac)

    i4 = i1 + (i2 - i1) * frac
    i5 = i2 + (i3 - i2) * frac

    return i4 + (i5 - i4) * frac

def n_bezier(nodes: List[np.ndarray], t: Union[int, float, np.number], duration: Union[int, float, np.number]):
    frac = t / duration
    lerp = [nodes[i] + frac * (nodes[i + 1] - nodes[i]) for i in range(len(nodes) - 1)]
    while len(lerp) > 1:
        lerp = [nodes[i] + frac * (nodes[i + 1] - nodes[i]) for i in range(len(lerp) - 1)]
    return lerp[0]

class Spline(Trajectory):
    '''
    After initializing, use class methods to append shapes
    To obtain position at a time, use path() method
    '''
    def __init__(self, start: np.ndarray):
        super().__init__()
        self.nodes = [SplineNode(start, np.ndarray([]), np.ndarray([]), 0)]  # type: List[SplineNode]
        self.path = self._path
        self.endpoint = np.array([])
        self.duration = 0  # type: Union[int, float, np.number]

    def _path(self, t: Union[int, float, np.number]):
        i = 0
        # TODO: replace with binary search
        while i + 1 < len(self.nodes) and t > self.nodes[i].duration:
            t -= self.nodes[i].duration
            i += 1
        if i + 1 == len(self.nodes):
            return self.nodes[-1].p
        return cubic_bezier(self.nodes[i], self.nodes[i + 1], t)

    def append(self, a: List[SplineNode], prev_d1: Optional[np.ndarray] = None, prev_duration: Optional[Union[int, float, np.number]] = None):
        if prev_d1 and prev_duration:
            self.nodes[-1].d1 = prev_d1
            self.nodes[-1].duration = prev_duration
        # TODO: option to append copies of a's SplineNodes instead
        self.nodes.extend(a)

    def append_line(self, end: np.ndarray, duration: Union[int, float, np.number]):
        delta = end - self.nodes[-1].p
        self.nodes[-1].d1 = delta / 3
        self.nodes[-1].duration = duration
        self.nodes.append(SplineNode(end, -delta / 3, np.ndarray([]), 0))
        self.duration += duration
        self.endpoint = self.nodes[-1].p

    # approximation
    def append_circle(self,
                      center: np.ndarray,
                      duration: Union[int, float, np.number],
                      axis: np.ndarray=np.array([0,0,1]),
                      rot: float = 1):
        if rot < 0:
            rot = -rot
            axis = -axis

        start = self.nodes[-1].p
        axis = norm(axis)
        center = center + np.vdot(start - center, axis) * axis
        diff_orth = start - center
        opp = center - diff_orth
        w = np.array([  # Cross product only works in 3D
            axis[1]*diff_orth[2] - axis[2]*diff_orth[1],
            axis[2]*diff_orth[0] - axis[0]*diff_orth[2],
            axis[0]*diff_orth[1] - axis[1]*diff_orth[0],
        ])
        control = w * 4 / 3

        self.nodes[-1].d1 = control
        self.nodes[-1].duration = duration / rot / 2
        # Assemble in half circles
        if rot >= 0.5:
            self.nodes.extend(SplineNode(opp, control.copy(), -control.copy(), duration / rot / 2) if i % 2 == 0 else
                              SplineNode(start, -control.copy(), control.copy(), duration / rot / 2)
                              for i in range(int(rot // 0.5)))
        if rot % 0.5:
            phi = (rot % 0.5) * math.pi
            # TODO: simplify?
            # mag = (1 - math.cos(phi)) / math.sin(phi)
            # self.nodes[-1].d1 = mag * self.nodes[-1].d1
            mag = 4 / 3 * np.linalg.norm(diff_orth) * (1 - math.cos(phi)) / math.sin(phi)
            self.nodes[-1].d1 = mag * norm(self.nodes[-1].d1)
            self.nodes[-1].duration = duration / rot * (rot % 0.5)
            end_diff = math.cos(rot * 2 * math.pi) * diff_orth + math.sin(rot * 2 * math.pi) * w
            end_control = np.array([  # Cross product only works in 3D
                axis[1]*end_diff[2] - axis[2]*end_diff[1],
                axis[2]*end_diff[0] - axis[0]*end_diff[2],
                axis[0]*end_diff[1] - axis[1]*end_diff[0],
            ]) # * mag
            end_control = norm(end_control) * mag
            self.nodes.append(SplineNode(center + end_diff, -end_control, np.ndarray([]), 0))
        self.duration += duration
        self.endpoint = self.nodes[-1].p

    def append_ellipse(self,
                       center: np.ndarray,  # position vector to axis
                       cross_axis: np.ndarray,  # cross-axis vector relative to axis
                       duration: Union[int, float, np.number],
                       axis: np.ndarray=np.array([0,0,1]),
                       rot: float = 1):
        if rot < 0:
            rot = -rot
            axis = -axis

        start = self.nodes[-1].p
        axis = norm(axis)
        center = center + np.vdot(start - center, axis) * axis
        diff_orth = start - center
        opp = center - diff_orth
        _matrix = np.array([diff_orth, cross_axis])

        # dilation * (-math.sin(dilation * t) * diff_orth + math.cos(dilation * t) * cross_axis)
        control = 4 / 3 * cross_axis
        self.nodes[-1].d1 = control
        self.nodes[-1].duration = duration / rot / 2  # 0.5 / dilation
        # Assemble in half ellipses
        if rot >= 0.5:
            self.nodes.extend(SplineNode(opp, control.copy(), -control.copy(), duration / rot / 2) if i % 2 == 0 else
                              SplineNode(start, -control.copy(), control.copy(), duration / rot / 2)
                              for i in range(int(rot // 0.5)))
        if rot % 0.5:
            phi = (rot % 0.5) * math.pi
            mag = 7 / 6 * (1 - math.cos(phi)) / math.sin(phi)  # compounded 7/6 seems to approximate well. yes this is scuffed
            self.nodes[-1].d1 = mag * self.nodes[-1].d1  # not being normalized
            self.nodes[-1].duration = duration / rot * (rot % 0.5)  # not true for an ellipse of varying speed

            end_diff = math.cos(rot * 2 * math.pi) * diff_orth + math.sin(rot * 2 * math.pi) * cross_axis
            end_control = -math.sin(rot * 2 * math.pi) * diff_orth + math.cos(rot * 2 * math.pi) * cross_axis
            end_control = mag * end_control
            self.nodes.append(SplineNode(center + end_diff, -end_control, np.ndarray([]), 0))
        self.duration += duration
        self.endpoint = self.nodes[-1].p

    def append_spiral(self,
                      center: np.ndarray,  # position vector to axis, cannot be same as start otherwise starting direction is undetermined
                      duration: Union[int, float, np.number],
                      end_radius: Union[int, float, np.number] = 0,
                      axis: np.ndarray = np.array([0, 0, 1]),
                      spiral_gap: Union[int, float, np.number] = 6,
                      ):
        start = self.nodes[-1].p
        axis = norm(axis)
        center = center + np.vdot(start - center, axis) * axis
        diff_orth = norm(start - center)
        w = np.array([  # Cross product only works in 3D
            axis[1] * diff_orth[2] - axis[2] * diff_orth[1],
            axis[2] * diff_orth[0] - axis[0] * diff_orth[2],
            axis[0] * diff_orth[1] - axis[1] * diff_orth[0],
        ])

        spiral_gap /= math.pi
        radius = np.linalg.norm(start - center)
        theta_start = radius / spiral_gap
        theta_end = end_radius / spiral_gap
        total_arclength = spiral_gap / 2 * abs((theta_start * (1 + theta_start ** 2) ** 0.5 +
                                             math.log(theta_start + (1 + theta_start ** 2) ** 0.5)) -
                                           (theta_end * (1 + theta_end ** 2) ** 0.5 +
                                             math.log(theta_end + (1 + theta_end ** 2) ** 0.5)))

        rot = abs(theta_start - theta_end) / 2 / math.pi
        dir = 1 if theta_start < theta_end else -1
        control = -dir * 4 / 3 * ((math.cos(0) * diff_orth + math.sin(0) * w) / theta_start + (-math.sin(0) * diff_orth + math.cos(0) * w))

        self.nodes[-1].d1 = -control * spiral_gap * theta_start
        # Assemble in "half" spirals
        if rot >= 0.5:
            for i in range(1, int(rot // 0.5) + 1):
                theta1 = theta_start + dir*(i-1)*math.pi
                theta2 = theta_start + dir*i*math.pi
                subarclength = spiral_gap / 2 * abs((theta1 * (1 + theta1 ** 2) ** 0.5 +
                                                  math.log(theta1 + (1 + theta1 ** 2) ** 0.5)) -
                                                (theta2 * (1 + theta2 ** 2) ** 0.5 +
                                                  math.log(theta2 + (1 + theta2 ** 2) ** 0.5)))
                self.nodes[-1].duration = duration * subarclength / total_arclength

                scale = spiral_gap * (theta_start + i*math.pi*dir)
                angle = dir*i*math.pi
                control = -dir * 4 / 3 * scale * ((math.cos(angle) * diff_orth + math.sin(angle) * w) / (theta_start + angle) + (-math.sin(angle) * diff_orth + math.cos(angle) * w))
                self.nodes.append(SplineNode(
                    center + diff_orth * scale * (-1)**i,
                    control.copy(),
                    -control.copy(),
                    0
                ))
        if rot % 0.5:
            phi = (rot % 0.5) * math.pi
            mag = (1 - math.cos(phi)) / math.sin(phi)
            self.nodes[-1].d1 = mag * self.nodes[-1].d1
            self.nodes[-1].duration = duration - sum(x.duration for x in self.nodes[-int(rot // 0.5) - 1:])

            end_diff = spiral_gap * theta_end * (math.cos(theta_end - theta_start) * diff_orth + math.sin(theta_end - theta_start) * w)
            end_control = ((math.cos(theta_end - theta_start) * diff_orth + math.sin(theta_end - theta_start) * w) / theta_end +
                           (-math.sin(theta_end - theta_start) * diff_orth + math.cos(theta_end - theta_start) * w))
            end_control = dir * 4 / 3 * mag * spiral_gap * theta_end * end_control
            self.nodes.append(SplineNode(center + end_diff, -end_control, np.ndarray([]), 0))
        self.duration += duration
        self.endpoint = self.nodes[-1].p


class CardinalSpline():
    def __init__(self, nodes: List[np.ndarray], speed: Union[int, float, np.number] = 1, scale: Union[int, float, np.number]=0.5):
        self.speed = speed
        self.scale = scale
        self.control = [(nodes[1] - nodes[0]) / 3 * scale] + [(nodes[i + 1] - nodes[i - 1]) / 3 * scale for i in range(1, len(nodes) - 1)] + [(nodes[-1] - nodes[-2]) / 3 * scale]
        self.duration = [0.0 for _ in range(len(nodes))]  # type: List[Union[int, float, np.number]]
        for i in range(1, len(nodes)):
            self.duration[i] = np.linalg.norm(nodes[i] - nodes[i - 1]) / speed + self.duration[i - 1]
        self.nodes = [SplineNode(nodes[i], -self.control[i], self.control[i], self.duration[i + 1] - self.duration[i]) for i in range(len(nodes) - 1)] + [SplineNode(nodes[-1], -self.control[-1], self.control[-1], 0)]

    def path(self, t: Union[int, float, np.number]):
        i = bisect_right(self.duration, t)
        if i == len(self.duration):
            return self.nodes[-1].p
        return cubic_bezier(self.nodes[i - 1], self.nodes[i], t - self.duration[i - 1])

    # TODO: add methods for inserting/removing nodes


class ClosedCubicBSpline(Trajectory):
    """
    C2 continuous closed B-Spline
    """
    def __init__(self, nodes: List[np.ndarray], speed: Union[int, float, np.number] = 1):
        super().__init__()
        self.speed = speed
        n = len(nodes)

        # Calculate B-Spline to Bezier control point conversions
        p_list = []
        d1_list = []
        for i in range(n):
            p_m1 = nodes[(i - 1) % n]
            p_0  = nodes[i]
            p_1  = nodes[(i + 1) % n]

            # Bezier junction points (B0)
            p = (p_m1 + 4 * p_0 + p_1) / 6.0
            # Bezier control vectors (B1 - B0)
            d1 = (p_1 - p_m1) / 6.0

            p_list.append(p)
            d1_list.append(d1)

        # Duplicate the first point at the end to close the mathematical loop
        p_list.append(p_list[0])
        d1_list.append(d1_list[0])

        self.t_nodes = [0.0] * (n + 1)
        for i in range(1, n + 1):
            self.t_nodes[i] = self.t_nodes[i - 1] + np.linalg.norm(p_list[i] - p_list[i - 1]) / speed

        self.nodes = [SplineNode(p_list[i], -d1_list[i], d1_list[i], self.t_nodes[i+1] - self.t_nodes[i]) for i in range(n)]
        self.nodes.append(SplineNode(p_list[-1], -d1_list[-1], d1_list[-1], 0))

        self.duration = self.t_nodes[-1]
        self.next = None

        self.path = self._path
        self.velocity = self._velocity

    def _path(self, t: Union[int, float, np.number]):
        t = t % self.duration
        i = bisect_right(self.t_nodes, t)
        if i == 0: 
            i = 1
        if i >= len(self.t_nodes):
            return self.nodes[-1].p
        return cubic_bezier(self.nodes[i - 1], self.nodes[i], t - self.t_nodes[i - 1])

    def _velocity(self, t: Union[int, float, np.number]):
        """Exact analytical derivative for the MinJerk solver."""
        t = t % self.duration
        i = bisect_right(self.t_nodes, t)
        if i == 0: 
            i = 1
        if i >= len(self.t_nodes): 
            i = len(self.t_nodes) - 1

        node_a = self.nodes[i - 1]
        node_b = self.nodes[i]

        c = node_a.p + node_a.d1
        b2 = node_b.p + node_b.d0

        seg_dur = node_a.duration
        if seg_dur == 0:
            return np.zeros_like(node_a.p)

        u = (t - self.t_nodes[i - 1]) / seg_dur

        # dP/du Bezier derivative
        dp_du = 3 * (1 - u)**2 * node_a.d1 + \
                6 * (1 - u) * u * (b2 - c) + \
                3 * u**2 * (-node_b.d0)

        return dp_du / seg_dur
