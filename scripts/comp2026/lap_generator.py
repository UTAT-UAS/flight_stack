from typing import List
import math
import numpy as np

from geopy.point import Point
from geopy.distance import distance
from flight_stack.flight_stack import FlightPlanner
from flight_stack.pather import trajectory
from flight_stack.btree import manager, controls, decorators, actions

WAYPOINT_SRC = "/home/uas/workspace/uas_ws/src/flight_stack/scripts/comp2026/koffler-waypoints-5-17.txt"
WAYPOINT_DST = "/home/uas/workspace/uas_ws/src/flight_stack/scripts/comp2026/lap.py"
OVERSHOOT_RADIUS = 10  # meters

def parse_waypoints(waypoint_lines: List[str]) -> List[Point]:
    """
    Parse a list of waypoint lines in mixed formats and return as geopy.Point objects.
    
    Point automatically detects and parses:
    - Decimal Degrees: 37.7, -122.2
    - Degrees, Minutes, Seconds: 37°25'19.07"N, 122°05'06.24"W
    - Degrees, Decimal Minutes: 32° 18.385' N 122° 36.875' W
    
    Returns:
        List of geopy.Point objects
    """
    waypoints = []
    for line in waypoint_lines:
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        try:
            p = Point(line)
            waypoints.append(p)
        except Exception as e:
            print(f"Warning: Could not parse '{line}': {e}")
    return waypoints


def global_to_local(waypoints_global: List[Point]) -> List[np.ndarray]:
    """
    Convert global waypoints to local coordinates (x, y) with first waypoint as origin.
    
    Uses bearing and distance from origin to compute local East-North coordinates.
    
    Args:
        waypoints_global: List of geopy.Point objects
        
    Returns:
        List of (x, y) tuples in meters, where x=East, y=North, origin at first waypoint
    """
    if not waypoints_global:
        return []

    origin = waypoints_global[0]
    waypoints_local = []

    for point in waypoints_global:
        # Calculate distance in meters
        dist_m = distance(origin, point).meters

        # Calculate bearing (azimuth) from origin to point in degrees
        # bearing calculation using haversine formula
        lat1_rad = math.radians(origin.latitude)
        lon1_rad = math.radians(origin.longitude)
        lat2_rad = math.radians(point.latitude)
        lon2_rad = math.radians(point.longitude)

        dlon = lon2_rad - lon1_rad
        y = math.sin(dlon) * math.cos(lat2_rad)
        x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)
        bearing_rad = math.atan2(y, x)
        bearing_deg = math.degrees(bearing_rad)

        # Convert bearing + distance to (x, y) coordinates
        # bearing 0° = North, 90° = East
        bearing_rad = math.radians(bearing_deg)
        x_local = dist_m * math.cos(bearing_rad)
        y_local = dist_m * math.sin(bearing_rad)

        waypoints_local.append(np.array([x_local, y_local, 0]))

    return waypoints_local

def get_winding(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> np.ndarray:
    """
    Determine the winding direction of three points (a, b, c).
    
    Returns:
        1 if counter-clockwise (left turn)
        -1 if clockwise (right turn)
        0 if collinear
    """
    ax, ay = a[0], a[1]
    bx, by = b[0], b[1]
    cx, cy = c[0], c[1]
    cross_product = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
    if cross_product > 0:
        return 1
    elif cross_product < 0:
        return -1
    else:
        return 0


def get_orthogonal(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """
    Get a unit vector orthogonal to the vector from a to b.
    
    Returns:
        A tuple representing the orthogonal unit vector (x, y)
    """
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    length = math.sqrt(dx**2 + dy**2)
    # rotate 90 deg cw
    orth_x = dy / length
    orth_y = -dx / length
    return np.array([orth_x, orth_y])


def get_angle(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float:
    """
    Get the signed angle (in degrees) formed by three points (a, b, c) with b as the vertex.
    
    Returns:
        Angle in degrees, positive for counter-clockwise turns, negative for clockwise turns
    """
    v0 = (b[0] - a[0], b[1] - a[1])
    v1 = (c[0] - b[0], c[1] - b[1])
    
    dot_prod = v0[0] * v1[0] + v0[1] * v1[1]
    cross_prod = v0[0] * v1[1] - v0[1] * v1[0]
    # CCW turns will be positive, CW turns will be negative
    angle = math.atan2(cross_prod, dot_prod)
    return angle


def generate_lap(waypoints_local: List[np.ndarray]) -> List[str]:
    """
    Generate a lap trajectory from local waypoints.
    
    For simplicity, this function just formats the local waypoints into a string list.
    In a real implementation, this would create a trajectory object for the flight planner.
    
    Args:
        waypoints_local: List of (x, y) tuples in meters
    Returns:
        List of strings representing the lap trajectory (placeholder)
    """
    overshoot_points = []
    windings = []
    turn_angles = []
    for i, (x, y, z) in enumerate(waypoints_local):
        winding = get_winding(waypoints_local[i-1], waypoints_local[i], waypoints_local[(i+1) % len(waypoints_local)])
        if winding == 0:  # hope that next waypoint is a turn
            winding = get_winding(waypoints_local[i-2], waypoints_local[i-1], waypoints_local[i])
        windings.append(winding)

        orth0 = winding * OVERSHOOT_RADIUS * get_orthogonal(waypoints_local[i-1], waypoints_local[i])
        overshoot_x = x + orth0[0]
        overshoot_y = y + orth0[1]
        overshoot_points.append(np.array([overshoot_x, overshoot_y, 0]))

        orth1 = winding * OVERSHOOT_RADIUS * get_orthogonal(waypoints_local[i], waypoints_local[(i+1) % len(waypoints_local)])
        overshoot_x = x + orth1[0]
        overshoot_y = y + orth1[1]
        overshoot_points.append(np.array([overshoot_x, overshoot_y, 0]))

        angle = get_angle(waypoints_local[i-1], waypoints_local[i], waypoints_local[(i+1) % len(waypoints_local)])
        turn_angles.append(angle)


    overshoot_points = overshoot_points[1:] + overshoot_points[:1]
    trajectories = []
    for i in range(len(overshoot_points)):
        wpi = ((i + 1) // 2) % len(waypoints_local)
        if i % 2 == 1: # arc
            #trajectory.Circle(start=overshoot_points[i], center=waypoints_local[wpi], cycles=turn_angles[wpi] / 2 / math.pi, axis=np.array([0,0,windings[wpi]]), speed=1)
            trajectories.append(f"trajectory.Circle(start=overshoot_points[{i}], center=waypoints[{wpi}], cycles={abs(turn_angles[wpi]) / 2 / math.pi}, axis=np.array([0,0,{windings[wpi]}]), speed=turn_v_ratio),\n")
        else:
            #trajectory.Line(start=overshoot_points[i], end=overshoot_points[i+1], duration={np.norm(overshoot_points[i+1] - overshoot_points[i])})
            trajectories.append(f"trajectory.Line(start=overshoot_points[{i}], end=overshoot_points[{(i+1) % len(overshoot_points)}], duration={np.linalg.norm(overshoot_points[(i+1) % len(overshoot_points)] - overshoot_points[i])}),\n")
    #trajectories = [
    #    "trajectory.ClosedCubicBSpline(nodes=overshoot_points, speed=1),\n"
    #]
    return overshoot_points, trajectories

def main():
    with open(WAYPOINT_SRC, "r") as f:
        data = f.readlines()
        waypoints_global = parse_waypoints(data)
        waypoints_local = global_to_local(waypoints_global)
        overshoot_points, trajectories = generate_lap(waypoints_local)
    
    with open(WAYPOINT_DST, "r") as f:
        contents = f.readlines()

    ri = -1
    for i, line in enumerate(contents):
        if line.startswith("first_wp ="):
            ri = i
            break
    if ri != -1:
        contents[ri] = f"first_wp = Point(\"{waypoints_global[0]}\")\n"

    try:
        rr = contents.index("waypoints = np.array([\n")
        rl = contents.index("])  # original waypoints in local NED\n", rr)
        contents = contents[:rr + 1] + [np.array2string(wp, separator=', ') + ',\n' for wp in waypoints_local] + contents[rl:]
    except ValueError:
        print("Error: Could not find waypoints section in lap.py. Please ensure section of format:\nwaypoints = np.array([\n...\n])  # original waypoints in local NED")

    try:
        rr = contents.index("overshoot_points = np.array([\n")
        rl = contents.index("])  # target waypoints in local NED\n", rr)
        contents = contents[:rr + 1] + [np.array2string(wp, separator=', ') + ',\n' for wp in overshoot_points] + contents[rl:]
    except ValueError:
        print("Error: Could not find waypoints section in lap.py. Please ensure section of format:\novershoot_points = np.array([\n...\n])  # target waypoints in local NED")

    try:
        rr = contents.index("    trajectories = [\n")
        rl = contents.index("    ]  # path objects\n", rr)
        trajectories = ["        " + t for t in trajectories]
        contents = contents[:rr + 1] + trajectories + contents[rl:]
    except ValueError:
        print("Error: Could not find trajectories section in lap.py. Please ensure section of format:\n    trajectories = [\n    ...\n    ]  # path objects")

    with open(WAYPOINT_DST, "w") as f:
        f.writelines(contents)

if __name__ == "__main__":
    main()
