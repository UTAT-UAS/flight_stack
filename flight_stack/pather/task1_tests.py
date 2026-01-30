"""from bisect import bisect_right
import math
import numpy as np
from typing import Union, List

import trajectory

def lin_interpol(
        start: np.ndarray,  # x, y, z
        delta: np.ndarray,  # dx, dy, dz
        duration: Union[int, float, np.number],
        t: float):
    return start + delta * t / duration

def scout_exhaustive(
        target: np.ndarray,
        radius: float=100,
        origin: np.ndarray=np.array([0,0,0]),
        alg: int=0,
        detection_radius = 3,
        **kwargs):
    '''Returns a function that returns position along path given time in seconds
        :param target: xyz coordinates of the center of the search area (z is the height the drone should be when searching)
        :param radius: radius of circular search area in the xy-plane
        :param origin: xyz coordinates of drone at beginning
        :param alg: the algorithm to use (0: tangent spiral, 1: direct spiral, 2: griddle)
        :param detection_radius: the distance between spiral arms
        :param kwargs: passed to specific algorithms
    '''

    # Tangent spiral
    if alg == 0:
        radius += detection_radius

        xy_dist = np.linalg.norm(origin[:2] - target[:2])
        phi = -math.acos(radius / xy_dist)  # Doesn't work if origin is already within radius of target
        r11 = math.cos(phi)
        r21 = math.sin(phi)
        rot_mat = np.array([[r11, r21, 0], [-r21, r11, 0], [0, 0, 0]])
        entry = target - origin + radius * np.matmul((origin - target) / xy_dist, rot_mat)
        beeline_duration = np.linalg.norm(entry - origin)

        spiral_gap = detection_radius / math.pi
        theta_max = radius / spiral_gap
        theta_diff = math.atan2(entry[1] - target[1], entry[0] - target[0]) - (theta_max % (2 * math.pi))
        arclength = spiral_gap / 2 * ((theta_max * (1 + theta_max ** 2) ** 0.5 +
                                       math.log(theta_max * (1 + theta_max ** 2) ** 0.5)) -
                                      (1 + math.log(1)))
        spiral_domain = theta_max ** 2
        speed_norm = spiral_domain / arclength
        total_duration = spiral_domain + beeline_duration

        def path(t: float):
            if t < beeline_duration:  # Beeline
                return lin_interpol(origin, entry, beeline_duration, t)
            elif t < total_duration:
                theta = (spiral_domain - (t - beeline_duration) * speed_norm) ** 0.5
                return np.array([
                    spiral_gap * theta * math.cos(theta + theta_diff),
                    spiral_gap * theta * math.sin(theta + theta_diff),
                    0
                ]) + target
            return target

        return total_duration, path

    # Direct archimedean spiral
    elif alg == 1:
        radius += detection_radius
        xy_dist = np.linalg.norm(origin[:2] - target[:2])
        xy_angle = math.atan2(origin[1] - target[1], origin[0] - target[0])  # Directional angle of origin from target

        entry_radius = kwargs['entry_radius'] if 'entry_radius' in kwargs.keys() else 6  # Radius for the entry-smoothening circle
        entry_theta = math.acos(entry_radius / (entry_radius + radius))
        entry_curve_start_dist = ((entry_radius + radius)**2 - entry_radius**2) ** 0.5  # Distance from the node that the drone will begin to veer
        entry_curve_start_angle = xy_angle + math.pi/2
        entry_curve_duration = entry_theta * entry_radius
        entry_curve_origin = target - origin + [
            (entry_radius + radius) * math.cos(xy_angle - (math.pi/2 - entry_theta)),
            (entry_radius + radius) * math.sin(xy_angle - (math.pi/2 - entry_theta)),
            0]
        spiral_entry = [
            target[0] + radius * math.cos(xy_angle - (math.pi/2 - entry_theta)),
            target[1] + radius * math.sin(xy_angle - (math.pi/2 - entry_theta))]

        beeline_entry = (target - origin) * (1 - entry_curve_start_dist/xy_dist)
        beeline_entry[2] = target[2] - origin[2]  # Reach desired height before beginning search
        beeline_duration = abs(xy_dist - entry_curve_start_dist)

        # sauce: https://en.wikipedia.org/wiki/Archimedean_spiral
        spiral_gap = detection_radius / math.pi
        theta_max = radius / spiral_gap
        theta_diff = math.atan2(spiral_entry[1] - target[1], spiral_entry[0] - target[0]) - (theta_max % (2*math.pi))
        arclength = spiral_gap/2 * ((theta_max*(1+theta_max**2)**0.5 + math.log(theta_max*(1+theta_max**2)**0.5)) - (1 + math.log(1)))
        # TODO: use arclength formula to prove that spacing is even
        spiral_domain = theta_max ** 2
        speed_norm = spiral_domain / arclength
        total_duration = arclength + beeline_duration + entry_curve_duration

        def path(t: float):
            if t < beeline_duration:  # Beeline
                return lin_interpol(origin, beeline_entry, beeline_duration, t)
            elif t < beeline_duration + entry_curve_duration:
                theta = (t - beeline_duration) / entry_radius
                return [
                    entry_radius * math.cos(theta + entry_curve_start_angle),
                    entry_radius * math.sin(theta + entry_curve_start_angle),
                    0
                ] + entry_curve_origin
            elif t < total_duration:
                theta = (spiral_domain - (t - beeline_duration - entry_curve_duration) * speed_norm) ** 0.5
                return np.array([
                    spiral_gap * theta * math.cos(theta + theta_diff),
                    spiral_gap * theta * math.sin(theta + theta_diff),
                    0
                ]) + target
            return target

        return total_duration, path

    # Griddle pattern
    elif alg == 2:
        xy_dist = np.linalg.norm(origin[:2] - target[:2])
        entry = (target - origin) * (1 - radius/xy_dist)
        entry[2] = target[2] - origin[2]  # Reach desired height before beginning search

        # Griddle pattern
        r_2 = radius**2
        x_step = [detection_radius,
                  detection_radius + min(radius % (2*detection_radius), (radius-detection_radius) % (2*detection_radius)),
                  2*detection_radius]  # Last element treated as continuous
        circle_discrete = []  # y coords in the 3rd quadrant of a circle of radius radius at each cumulative x_step from the left
        i = 0
        x_cumul = x_step[0]
        while x_cumul <= radius:
            circle_discrete.append((r_2 - (radius - x_cumul)**2) ** 0.5)
            i += 1
            x_cumul += x_step[min(i, len(x_step) - 1)]
        navpoints = np.array([[0, circle_discrete[0] * -2, 0]])
        for i in range(1, len(circle_discrete)):
            navpoints = np.append(
                navpoints,
                [
                    [x_step[min(i, len(x_step) - 1)], (circle_discrete[i] - circle_discrete[i-1]) * (-1 if i % 2 == 1 else 1), 0],
                    [0, circle_discrete[i] * 2 * (-1 if i % 2 == 0 else 1), 0]
                ],
                axis=0)

        if x_cumul - x_step[min(len(circle_discrete), len(x_step) - 1)] == radius:
            # edge case where x_step perfectly cumulates to radius (double traversal of diameter would be inefficient)
            navpoints = np.append(
                navpoints,
                np.flip(navpoints[:-1], axis=0),
                axis=0
            )
        else:
            navpoints = np.append(
                navpoints,
                np.append([[(radius - x_cumul) * 2, 0, 0]],
                    np.flip(navpoints, axis=0) * [1, -1, 1],
                    axis=0),
                axis=0
            )

        # Rotate griddle pattern
        angle = math.atan2(entry[1] - target[1], entry[0] - target[0]) - math.atan2(circle_discrete[0], x_step[0] - radius)
        r11 = math.cos(angle)
        r21 = math.sin(angle)
        rot_mat = np.array([[r11, r21, 0], [-r21, r11, 0], [0, 0, 1]])
        navpoints = np.matmul(navpoints, rot_mat)
        #navpoints = np.array([[r11 * p[0] - r21 * p[1], r21 * p[0] + r11 * p[1], p[2]] for p in navpoints])

        pos_cumul = np.append([entry], navpoints, axis=0)
        time_cumul = [0.0 for _ in range(len(navpoints) + 1)]  # Directly proportional to distance
        time_cumul[0] = float(xy_dist) - radius
        for i in range(1, len(navpoints) + 1):
            time_cumul[i] = time_cumul[i - 1] + float(np.linalg.norm(pos_cumul[i]))
            pos_cumul[i] += pos_cumul[i - 1]

        def path(t: float):
            if t < xy_dist - radius:  # Beeline
                return lin_interpol(origin, entry, xy_dist - radius, t)
            i = bisect_right(time_cumul, t)
            if i < len(time_cumul):
                return lin_interpol(pos_cumul[i - 1], navpoints[i - 1], time_cumul[i] - time_cumul[i - 1], t - time_cumul[i - 1])
            return pos_cumul[-1]

        return time_cumul[-1], path

def task1_griddle_path(cur_pos: List[float], target: List[float], radius: float, gap: float, auto_return: bool = True):
    origin = np.array(cur_pos)
    np_target = np.array(target)

    xy_dist = np.linalg.norm(origin[:2] - np_target[:2])
    entry = (np_target - origin) * (1 - radius / xy_dist)
    entry[2] = np_target[2] - origin[2]  # Reach desired height before beginning search

    # Griddle pattern
    r_2 = radius ** 2
    x_step = [gap,
              gap + min(radius % (2 * gap), (radius - gap) % (2 * gap)),
              2 * gap]  # Last element treated as continuous
    circle_discrete = []  # y coords in the 3rd quadrant of a circle of radius radius at each cumulative x_step from the left
    i = 0
    x_cumul = x_step[0]
    while x_cumul <= radius:
        circle_discrete.append((r_2 - (radius - x_cumul) ** 2) ** 0.5)
        i += 1
        x_cumul += x_step[min(i, len(x_step) - 1)]
    navpoints = np.array([[0, circle_discrete[0] * -2, 0]])
    for i in range(1, len(circle_discrete)):
        navpoints = np.append(
            navpoints,
            [
                [x_step[min(i, len(x_step) - 1)],
                 (circle_discrete[i] - circle_discrete[i - 1]) * (-1 if i % 2 == 1 else 1), 0],
                [0, circle_discrete[i] * 2 * (-1 if i % 2 == 0 else 1), 0]
            ],
            axis=0)

    if abs(x_cumul - x_step[min(len(circle_discrete), len(x_step) - 1)] - radius) < (0.000000001):  # arbitrary tolerance for floating point precision
        # edge case where x_step perfectly cumulates to radius (double traversal of diameter would be inefficient)
        navpoints = np.append(
            navpoints,
            np.flip(navpoints[:-1], axis=0),
            axis=0
        )
    else:
        navpoints = np.append(
            navpoints,
            np.append([[(x_cumul - radius) * 2, 0, 0]],
                      np.flip(navpoints, axis=0) * [1, -1, 1],
                      axis=0),
            axis=0
        )

    # Rotate griddle pattern
    angle = math.atan2(entry[1] - np_target[1], entry[0] - np_target[0]) - math.atan2(circle_discrete[0], x_step[0] - radius)
    r11 = math.cos(angle)
    r21 = math.sin(angle)
    rot_mat = np.array([[r11, r21, 0], [-r21, r11, 0], [0, 0, 1]])
    navpoints = np.matmul(navpoints, rot_mat)

    pos_cumul = np.append([entry], navpoints, axis=0)
    for i in range(1, len(navpoints) + 1):
        pos_cumul[i] += pos_cumul[i - 1]
    # Start from and return to origin
    pos_cumul = np.append([origin], pos_cumul, axis=0)
    if auto_return:
        pos_cumul = np.append(pos_cumul, [origin], axis=0)
    
    lines = [trajectory.Line(start=pos_cumul[i], end=pos_cumul[i + 1], duration=np.linalg.norm(pos_cumul[i] - pos_cumul[i + 1])) for i in range(len(pos_cumul) - 1)]
    for i, l in enumerate(lines[:-1]):
        l.next = lines[i + 1]
    duration = sum(l.duration for l in lines)
    return duration, lines[0].path

def task1_spiral_path(cur_pos: List[float], target: List[float], radius: float, gap: float):
    np_cur_pos = np.array(cur_pos)
    np_target = np.array(target)
    entry = np_target + (np_cur_pos - np_target) / np.linalg.norm(np_cur_pos - np_target) * radius
    beeline = trajectory.Line(start=np_cur_pos, end=entry, duration=np.linalg.norm(np_cur_pos - entry))
    spiral = trajectory.Spiral(start=entry, center=np_target, speed=1, spiral_gap=gap)
    return_to_start = trajectory.Line(start=np_target, end=np_cur_pos, duration=np.linalg.norm(np_target - np_cur_pos))
    print(entry)

    beeline.next = spiral
    spiral.next = return_to_start
    duration = beeline.duration + spiral.duration + return_to_start.duration
    return duration, beeline.path
"""