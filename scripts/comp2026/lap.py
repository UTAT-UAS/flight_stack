import numpy as np
from geopy.point import Point

from flight_stack.pather import trajectory

first_wp = Point("44 1m 39.9462s N, 79 32m 14.8303s W")

waypoints = np.array([
[0., 0., 0.],
[   6.98198367, -327.27524796,    0.        ],
[ -65.2393483 , -154.69632788,    0.        ],
])  # original waypoints in local NED

overshoot_points = np.array([
[9.99772515, 0.21328822, 0.        ],
[  16.97970881, -327.06195975,    0.        ],
[  -2.24282612, -331.13567338,    0.        ],
[ -74.46415809, -158.5567533 ,    0.        ],
[ -74.45348398, -150.81049417,    0.        ],
[-9.21413568,  3.88583372,  0.        ],
])  # target waypoints in local NED

def generate_lap(x, y, z, turn_v_ratio):
    for i, wp in enumerate(waypoints):
        waypoints[i] = (x + wp[0], y + wp[1], z + wp[2])
    for i, wp in enumerate(overshoot_points):
        overshoot_points[i] = (x + wp[0], y + wp[1], z + wp[2])

    trajectories = [
        trajectory.Line(start=overshoot_points[0], end=overshoot_points[1], duration=327.34971517429216),
        trajectory.Circle(start=overshoot_points[1], center=waypoints[1], cycles=0.4403157398141634, axis=np.array([0,0,-1]), speed=turn_v_ratio),
        trajectory.Line(start=overshoot_points[2], end=overshoot_points[3], duration=187.08127764858688),
        trajectory.Circle(start=overshoot_points[3], center=waypoints[2], cycles=0.12659683093150756, axis=np.array([0,0,-1]), speed=turn_v_ratio),
        trajectory.Line(start=overshoot_points[4], end=overshoot_points[5], duration=167.89022135925956),
        trajectory.Circle(start=overshoot_points[5], center=waypoints[0], cycles=0.4330874292543291, axis=np.array([0,0,-1]), speed=turn_v_ratio),
    ]  # path objects
    return trajectories
