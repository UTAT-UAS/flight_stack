"""from typing import List, Optional, Union, Tuple, Dict
import numpy as np  # You could not live with your own failure. Where did that bring you? Back to C.
from itertools import permutations
import heapq

import trajectory

def fastest_sequence(
        weights: List[List[float]],
        alg: int = 0):
    '''Returns a reordered list of indices representing the shortest distance path reaching all destinations
    Assumes that index 0 is the start node, returned list only contains index 1 to len(weights)
    '''

    # Exhaustive brute force (should be fine with only 8 nodes max)
    if alg == 0:
        cost = sum(weights[i][i + 1] for i in range(len(weights)-1))
        path0 = tuple(range(1, len(weights)))
        for p in permutations(range(1, len(weights))):
            cost_temp = weights[0][p[0]] + sum(weights[p[i]][p[i + 1]] for i in range(len(p) - 1))
            if cost_temp < cost:
                cost = cost_temp
                path0 = p

        return list(path0)

    # Insertion sort (doesn't always match exhaustive search)
    elif alg == 1:
        path1 = [0, 1]
        for n in range(2, len(weights)):
            opti = [len(path1), weights[path1[-1]][n]]
            for i in range(1, len(path1)):
                added_dist = weights[path1[i]][n] + weights[path1[i - 1]][n] - weights[path1[i]][path1[i - 1]]
                if added_dist < opti[1]:
                    opti = [i, added_dist]
            path1.insert(int(opti[0]), n)

        return path1[1:]

    # DFS
    elif alg == 2:
        # doubt memoization would help much, so not sure what advantage DFS has rn
        # nvm lmao it's actually faster than permutations
        def dfs(cost, node, visited, depth):
            if depth == len(visited) - 2:
                visited[node] = True
                last = visited.index(False)
                return cost + weights[node][last], [node, last]

            visited[node] = True
            opti_cost = -1
            opti_path = []
            for i in range(1, len(visited)):  # 0 is guaranteed to be visited
                if visited[i]: continue
                res_cost, res_path = dfs(cost + weights[node][i], i, visited[:], depth + 1)
                if opti_cost == -1:
                    opti_cost = res_cost
                    opti_path = res_path
                elif res_cost < opti_cost:
                    opti_cost = res_cost
                    opti_path = res_path
            return opti_cost, [node] + opti_path

        cost, path2 = dfs(0, 0, [True] + [False for _ in range(len(weights)-1)], 0)
        #for start in range(1, len(nodes)):
        #    res_cost, res_path = dfs(0, start, [1] + [0 for _ in range(len(nodes))], 0)
        path2 = path2[1:]
        return path2

    elif alg == 3:
        memo_cost = [[-1.0 for _ in range((1<<(len(weights))) - 1)] for _ in range(len(weights))]
        memo_path = [[[] for _ in range((1<<(len(weights))) - 1)] for _ in range(len(weights))]  # type: List[List[List[int]]]

        # Fill in base case
        for a in range(len(weights)):
            visited = (((1<<(len(weights))) - 1)) ^ (1<<a)
            for b in range(len(weights)):
                if a == b: continue
                memo_cost[b][visited] = weights[b][a]
                memo_path[b][visited] = [b, a]

        def dfs(cost, node, visited, depth):
            visited += 1 << node

            if memo_cost[node][visited] != -1:
                return cost + memo_cost[node][visited], memo_path[node][visited]

            opti_cost = -1
            opti_path = []
            for i in range(1, len(weights)):  # 0 is guaranteed to be visited
                if visited & (1 << i): continue
                res_cost, res_path = dfs(weights[node][i], i, visited, depth + 1)
                if opti_cost == -1:
                    opti_cost = res_cost
                    opti_path = res_path
                elif res_cost < opti_cost:
                    opti_cost = res_cost
                    opti_path = res_path
            memo_cost[node][visited] = opti_cost
            memo_path[node][visited] = [node] + opti_path
            return cost + opti_cost, memo_path[node][visited]

        cost, path2 = dfs(0, 0, 0, 0)
        path2 = path2[1:]
        return path2
    
    #A*
    elif alg == 4:
        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1]) #returning the combined x and y distances between two nodes which can be used for the heuristic cost to find the distance from the current node to the end node

        #takes in an nxn 2d array as well as the start and end coords
        #right now the algoirthm assumes the 
        #heapq is used for pushing the nodes into the openList because it handles sorting the list based on the f score by itself
        def a_star(matrix, start, end):

            n = len(matrix)
            openList = []  #initializing the open list 
            heapq.heappush(openList, (0, start))

            came_from = {}  # defining a dictionary to keep track of the node that each node comes from so that the [ath can later be recontrcuted through this]
            #keeping track of all the g_scores and the f_scores in dictionaries so that we can identify which scores belong to which nodes
            g_score = {start: 0}  # Cost from start to current node.
            f_score = {start: heuristic(start, end)}  # Estimated cost from start to end by combining the g score with the heuristic score

            while openList: #while the open list isn't empty
                _, current = heapq.heappop(openList)

                # If the current node is the end, end the loop and then reconstruct and return the path
                if current == end:
                    path = []
                    while current in came_from: #iterating through the came from list based on the end node so that each time the node is appended to the path so is the node it came from all the way till the start node
                        path.append(current)
                        current = came_from[current]
                    path.append(start)
                    return path[::-1] #reversing the order in which the nodes were added to define the path

                # Explore neighbors
                neighbors = [(current[0] + dx, current[1] + dy) for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]] #defining all the neighbours (top, down, left, right) of the node being examined by essentially taking the current coordinates and iterating through a list of the delta between itself and the nieghbours

                for neighbor in neighbors:
                    row, col = neighbor #deconstructing the neighbour to identify its coordinates
                    if 0 <= row < n and 0 <= col < n and matrix[row][col] == 0: #making sure the node is not an obstacle and that it is within the actual matrix map
                        temp_g = g_score[current] + 1 #giving the neighbours a temporary g_score as later on it is used as a condition. The score is just equal to the parent node plus 1 as its assuming the node is moving forward

                        #the following chunk of code updates the scores for the neighbours
                        if neighbor not in g_score or temp_g < g_score[neighbor]: #since the code below is updating the scores of the neighbour the code checks if actually even has to be updated by ensuring it has either not been assigned a g_score yet or ensuring that the new g_score is higher than the ones its already been assigned
                            # Update path and scores
                            came_from[neighbor] = current #defining the came_from node of the neighbour node to the current node being examined
                            g_score[neighbor] = temp_g #setting the scores for the neighbour nodes
                            f_score[neighbor] = temp_g + heuristic(neighbor, end)
                            heapq.heappush(openList, (f_score[neighbor], neighbor)) #adding it into the open list and automatically sorting it within the list based on its f_score

            # If no path is found, it return an empty list
            return []


    # hth would I modify a minimum spanning tree alg

class Obstacle:
    def path_collision(self, *args, **kwargs):
        return False

    def point_collision(self, *args, **kwargs):
        return False

class ObstacleCircular(Obstacle):
    def __init__(self,
            center: np.ndarray,
            radius: Union[int, float, np.number],
            tolerance: Union[int, float, np.number] = 0):
        self.center = center
        self.radius = radius + tolerance


    def path_collision(self,
            start: np.ndarray,
            end: np.ndarray,
            tolerance: Union[int, float, np.number] = 0):
        super().__init__()
        radius = self.radius + tolerance
        if np.linalg.norm(end - self.center) < radius or np.linalg.norm(start - self.center) < radius:
            return True
        u_path = (end - start) / np.linalg.norm(end - start)
        proj = np.vdot(self.center - start, u_path)
        return 0 < proj < np.linalg.norm(end - start) and np.linalg.norm(self.center - start - proj*u_path) < radius

    def point_collision(self,
            point: np.ndarray,
            tolerance: Union[int, float, np.number] = 0):
        return np.linalg.norm(point - self.center) < self.radius + tolerance

def euclidean_weights(targets: np.ndarray):
    weights = [[0.0 for _ in range(len(targets))] for _ in range(len(targets))]
    for a in range(len(targets)):
        for b in range(a + 1, len(targets)):
            weights[a][b] = float(np.linalg.norm(targets[a] - targets[b]))
            weights[b][a] = weights[a][b]
    return weights

class PRM_Weights:
    def __init__(self,
             targets: np.ndarray,
             bounds: List[List[Union[int, float, np.number]]],
             step_size: Union[int, float, np.number],
             obstacles: List[Obstacle],
             goal_tolerance: Union[int, float, np.number] = 0,
             collision_tolerance: Union[int, float, np.number] = 0,
             iter_limit_per: int = 10000000,
             alg: str = 'rrt',
             repetitions: int = 1,
             rewire_radius: Union[int, float, np.number] = 5):
        self.weights = [[-1.0 for _ in range(len(targets))] for _ in range(len(targets))]
        self.paths = [[[] for _ in range(len(targets))] for _ in range(len(targets))]  # type: List[List]
        for _ in range(repetitions):
            for a in range(len(targets) - 1):
                if alg == 'rrt*':
                    assert rewire_radius >= step_size
                    prm_obj = RRTStar(start=targets[a],
                                   targets=targets[a+1:],
                                   bounds=bounds,
                                   step_size=step_size,
                                   obstacles=obstacles,
                                   rewire_radius=rewire_radius,
                                   goal_tolerance=goal_tolerance,
                                   collision_tolerance=collision_tolerance,
                                   iter_limit=iter_limit_per)  # type: PRM
                else:
                    prm_obj = RRT(start=targets[a],
                                   targets=targets[a+1:],
                                   bounds=bounds,
                                   step_size=step_size,
                                   obstacles=obstacles,
                                   goal_tolerance=goal_tolerance,
                                   collision_tolerance=collision_tolerance,
                                   iter_limit=iter_limit_per)

                prm_obj.postprocess()
                for b in range(a + 1, len(targets)):
                    if prm_obj.dists[b - (a + 1)] != -1 and (self.weights[a][b] == -1 or prm_obj.dists[b - (a + 1)] < self.weights[a][b]):
                        self.weights[a][b] = prm_obj.dists[b - (a + 1)]
                        self.weights[b][a] = self.weights[a][b]
                        self.paths[b][a] = [prm_obj.nodes[x].coords for x in prm_obj.paths[b - (a + 1)]]
                        self.paths[a][b] = reversed(self.paths[a][b])  # hopefully this is more memory efficient

class PRM:
    class Node:
        def __init__(self, coords: np.ndarray, parent: int, dist: float):
            self.coords = coords.copy()
            self.parent = parent
            self.dist = dist

    def __init__(self):
        self.nodes = []  # type: List[PRM.Node]
        self.obstacles = []  # type: List[Obstacle]
        self.dists = []  # type: List[float]
        self.paths = []  # type: List[List[int]]

    def postprocess(self, collision_tolerance: Union[int, float, np.number]=0):
        for i, path in enumerate(self.paths):
            if self.dists[i] == float('inf'): continue  # no path
            dist = 0.0
            p = [path[0]]  # type: List[int]
            for pi in range(len(path) - 1):
                if any(o.path_collision(self.nodes[path[pi]].coords, self.nodes[p[-1]].coords,
                                        collision_tolerance) for o in self.obstacles):
                    dist += float(np.linalg.norm(self.nodes[path[pi - 1]].coords - self.nodes[p[-1]].coords))
                    p.append(path[pi - 1])
            dist += float(np.linalg.norm(self.nodes[path[-1]].coords - self.nodes[p[-1]].coords))
            p.append(path[-1])
            self.dists[i] = dist
            self.paths[i] = p[:]


class RRT(PRM):
    '''
    Only a class for the sake of not cluttering higher namespace with helper functions
    '''
    def __init__(self,
            start: np.ndarray,
            targets: np.ndarray,
            bounds: List[List[Union[int, float, np.number]]],
            step_size: Union[int, float, np.number],
            obstacles: List[Obstacle],
            goal_tolerance: Union[int, float, np.number]=0,
            collision_tolerance: Union[int, float, np.number]=0,
            iter_limit: int=10000000):
        super().__init__()
        self.nodes = [super().Node(coords=start, parent=0, dist=0)]  # type: List[PRM.Node]
        self.obstacles = obstacles

        visited = [[] for _ in range(len(targets))]  # type: List[List[int]]
        _i = 0
        while not all(visited) and _i < iter_limit:
            _i += 1
            rand = self.rand_node(bounds)

            closest_id = 0
            closest_dist = np.linalg.norm(self.nodes[0].coords - rand)
            for i, n in enumerate(self.nodes[1:]):
                _dist = np.linalg.norm(n.coords - rand)
                if _dist < closest_dist:
                    closest_id = i + 1
                    closest_dist = _dist

            delta = (rand - self.nodes[closest_id].coords) / np.linalg.norm(
                rand - self.nodes[closest_id].coords) * step_size
            if any(o.path_collision(self.nodes[closest_id].coords, self.nodes[closest_id].coords + delta,
                                    collision_tolerance) for o in obstacles):
                continue
            self.nodes.append(self.Node(self.nodes[closest_id].coords + delta, closest_id, self.nodes[closest_id].dist + 1))
            for t in range(len(targets)):
                if not visited[t] and np.linalg.norm(self.nodes[-1].coords - targets[t]) < goal_tolerance:
                    visited[t].append(len(self.nodes) - 1)

        if not all(visited):
            raise Exception(f"Could not reach targets:\n{chr(10).join(f'{i}: {targets[i]}' for i in range(len(targets)) if not visited[i])}")

        self.dists = [-1.0 for _ in range(len(targets))]  # type: List[float]
        self.paths = [[] for _ in range(len(targets))]  # type: List[List[int]]
        # 🤢
        for i in range(len(targets)):
            if len(visited[i]) == 0:
                self.dists[i] = float('inf')
                continue
            for leaf in visited[i]:  # yes this isn't technically always gonna be a leaf node
                _path = self.backtrack_to_origin(self.nodes, leaf)
                if not self.paths[i] or len(_path) < len(self.paths[i]):
                    self.dists[i] = (len(_path) - 1) * step_size + float(np.linalg.norm(targets[i] - self.nodes[_path[0]].coords))
                    self.paths[i] = _path[:]

    def rand_node(self, bounds):
        return np.array([np.random.uniform(b[0], b[1]) for b in bounds])

    def backtrack_to_origin(self, nodes, leaf):
        path = [leaf]
        while nodes[path[-1]].parent != path[-1]:
            path.append(nodes[path[-1]].parent)
        return path

    # obsolete
    def tree_dist(self, nodes, a, b):
        # TODO: replace with more efficient lowest common ancestor code alg
        a_path = [a]
        while nodes[a_path[-1]].parent != a_path[-1]:
            a_path.append(nodes[a_path[-1]].parent)
        b_path = [b]
        while nodes[b_path[-1]].parent != b_path[-1]:
            b_path.append(nodes[b_path[-1]].parent)
        i = 1
        while i <= min(len(a), len(b)):
            if a_path[-i] != b_path[-i]:
                return a[:-i + 2] + reversed(b[:-i + 1])
            i += 1
        if len(a) > len(b):
            return a[:-len(b) + 1]
        else:
            return b[:-len(a) + 1]

class RRTStar(PRM):
    def __init__(self,
            start: np.ndarray,
            targets: np.ndarray,
            bounds: List[List[Union[int, float, np.number]]],
            step_size: Union[int, float, np.number],
            obstacles: List[Obstacle],
            rewire_radius: Union[int, float, np.number],
            goal_tolerance: Union[int, float, np.number]=5,
            collision_tolerance: Union[int, float, np.number]=0,
            iter_limit: int=10000000):
        super().__init__()
        self.nodes = [super().Node(coords=start, parent=0, dist=0)]  # type: List[PRM.Node]
        self.obstacles = obstacles

        visited = [[] for _ in range(len(targets))]  # type: List[List[int]]
        _i = 0
        while not all(visited) and _i < iter_limit:
            _i += 1
            rand = self.rand_node(bounds)

            closest_id = 0
            closest_dist = np.linalg.norm(self.nodes[0].coords - rand)
            for i, n in enumerate(self.nodes[1:]):
                _dist = np.linalg.norm(n.coords - rand)
                if _dist < closest_dist:
                    closest_id = i + 1
                    closest_dist = _dist

            new_c = self.nodes[closest_id].coords + (rand - self.nodes[closest_id].coords) / np.linalg.norm(rand - self.nodes[closest_id].coords) * step_size

            star_id = -1
            star_dist = float('inf')
            for i, n in enumerate(self.nodes):
                if np.linalg.norm(n.coords - new_c) < rewire_radius:
                    _dist = n.dist + np.linalg.norm(new_c - n.coords)
                    if _dist >= star_dist or any(o.path_collision(n.coords, new_c, collision_tolerance) for o in obstacles):
                        continue
                    star_id = i
                    star_dist = float(_dist)
            if star_id == -1: continue

            self.nodes.append(self.Node(new_c, star_id, star_dist))
            for t in range(len(targets)):
                if not visited[t] and np.linalg.norm(new_c - targets[t]) < goal_tolerance:
                    visited[t].append(len(self.nodes) - 1)

        if not all(visited):
            raise Exception(f"Could not reach targets:\n{chr(10).join(f'{i}: {targets[i]}' for i in range(len(targets)) if not visited[i])}")

        self.dists = [-1.0 for _ in range(len(targets))]
        self.paths = [[] for _ in range(len(targets))]  # type: List[List[int]]
        # 🤢
        for i in range(len(targets)):
            if len(visited[i]) == 0:
                self.dists[i] = float('inf')
                continue
            for leaf in visited[i]:  # yes this isn't technically always gonna be a leaf node
                _path = self.backtrack_to_origin(self.nodes, leaf)
                if not self.paths[i] or len(_path) < len(self.paths[i]):
                    self.dists[i] = (len(_path) - 1) * step_size + float(np.linalg.norm(targets[i] - self.nodes[_path[0]].coords))
                    self.paths[i] = _path[:]

    def rand_node(self, bounds):
        return np.array([np.random.uniform(b[0], b[1]) for b in bounds])

    def backtrack_to_origin(self, nodes, leaf):
        path = [leaf]
        while nodes[path[-1]].parent != path[-1]:
            path.append(nodes[path[-1]].parent)
        return path

class AStar_Weights():
    def __init__(self,
                 matrix: List[List[int,]],
                 targets: List[Tuple[int, int]]):
        self.weights = [[0 for _ in range(len(targets))] for _ in range(len(targets))]  # type: List[List[float]]
        self.paths = [[[] for _ in range(len(targets))] for _ in range(len(targets))]  # type: List[List]

        for i, cell in enumerate(targets):
            discrete_map_obj = AStar(matrix, cell, targets[i + 1:])
            for j in range(len(targets) - i - 1):
                self.weights[i][i + j + 1] = discrete_map_obj.dists[j]
                self.weights[i + j + 1][i] = self.weights[i][i + j + 1]
                self.paths[i][i + j + 1] = discrete_map_obj.paths[j]
                self.paths[i + j + 1][i] = reversed(self.paths[i][i + j + 1])

class AStar():
    def __init__(self,
                 matrix: List[List[int]],
                 start: Tuple[int, int],
                 targets: List[Tuple[int, int]]):
        self.dists = [-1.0 for _ in range(len(targets))]  # type: List[float]
        self.paths = [[] for _ in range(len(targets))]  # type: List[List[Tuple[int]]]

        for i, target in enumerate(targets):
            _path = self.a_star(matrix, start, target)
            if not _path:
                self.dists[i] = float('inf')
                continue
            self.dists[i] = self.get_distance(_path)
            self.paths[i] = _path[:]

    def heuristic(self, a, b):
        # Diagonal distance heuristic suitable when diagonal moves cost sqrt(2)
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return (dx + dy) -0.5857864376269049 * min(dx, dy)

    def a_star(self,
               matrix: List[List[int]],
               start: Tuple[int, int],
               end: Tuple[int, int]):
        n = len(matrix)
        openList = []  # type: List[tuple] # Priority queue for the open set
        heapq.heappush(openList, (0, start))

        came_from = {}  # type: Dict[tuple, float] # To reconstruct the path later
        g_score = {start: 0}  # type: Dict[tuple, float] # Cost from start to the current node
        f_score = {start: self.heuristic(start, end)}  # Estimated total cost

        # Moves: 4 cardinal directions + 4 diagonal directions
        moves = [(-1, 0), (1, 0), (0, -1), (0, 1),
                 (-1, -1), (-1, 1), (1, -1), (1, 1)]

        while openList:
            _, current = heapq.heappop(openList)

            if current == end:
                # Reconstruct the path from end to start
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]  # Return reversed path

            for dx, dy in moves:
                neighbor = (current[0] + dx, current[1] + dy)
                row, col = neighbor
                # Check that neighbor is within bounds and not an obstacle
                if 0 <= row < n and 0 <= col < n and matrix[row][col] == 0:
                    # Determine move cost: diagonal moves cost sqrt(2) vs. 1 for cardinal moves
                    if dx != 0 and dy != 0:
                        move_cost = 1.4142135623730951
                    else:
                        move_cost = 1
                    temp_g = g_score[current] + move_cost

                    if neighbor not in g_score or temp_g < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = temp_g
                        f_score[neighbor] = temp_g + self.heuristic(neighbor, end)
                        heapq.heappush(openList, (f_score[neighbor], neighbor))

        # If no path is found, return an empty list
        return []

    def get_distance(self, path: List[Tuple[int, int]]):
        total_distance = 0  # type: float
        for i in range(len(path) - 1):
            x1, y1 = path[i - 1]
            x2, y2 = path[i]

            dx = abs(x2 - x1)
            dy = abs(y2 - y1)

            # Check if the move is diagonal
            if dx == 1 and dy == 1:
                total_distance += 1.4142135623730951
            # Check if the move is horizontal or vertical
            elif (dx == 1 and dy == 0) or (dx == 0 and dy == 1):
                total_distance += 1
            else:
                # Fallback: if moves are larger than 1 (which shouldn't happen for adjacent moves),
                # compute the distance as a series of diagonal and straight moves.
                diagonal_steps = min(dx, dy)
                straight_steps = abs(dx - dy)
                total_distance += diagonal_steps * 1.4142135623730951 + straight_steps

        return total_distance

def task2_path(
        cur_pos: List[float],
        buckets: List[List[float]],
        weights: Optional[List[List[float]]]=None):
    # Use euclidean distance by default
    if weights is None:
        weights = euclidean_weights(np.array([cur_pos] + buckets))

    opti_order = fastest_sequence(weights, alg=2)
    np_cur_pos = np.array(cur_pos)
    np_opti_order = [np.array(a) for a in opti_order]
    lines = ([trajectory.Line(start=np_cur_pos, end=np_opti_order[0], duration=np.linalg.norm(np_opti_order[0] - np_cur_pos))] +
             [trajectory.Line(start=np_opti_order[i], end=np_opti_order[i + 1], duration=np.linalg.norm(np_opti_order[i] - np_opti_order[i + 1])) for i in range(len(np_opti_order) - 1)] +
             [trajectory.Line(start=np_opti_order[-1], end=np_cur_pos, duration=np.linalg.norm(np_opti_order[-1] - np_cur_pos))])
    duration = sum(t.duration for t in lines)
    for i, traj in enumerate(lines[:-1]):
        traj.next = lines[i + 1]
    return duration, lines[0].path, lines[0].velocity
"""