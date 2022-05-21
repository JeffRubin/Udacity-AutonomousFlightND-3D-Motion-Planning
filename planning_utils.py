from enum import Enum
from queue import PriorityQueue
import numpy as np
import math


def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles and non-obstacle feature heights
    # -1 = obstacle
    # otherwise the value is the height of any feature at that location (feature is below drone_altitude);
    # if there is no feature at that location, the value is 0
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        obstacle_height_no_safety = alt + d_alt
        if obstacle_height_no_safety + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = -1
        else: # do not use safety_distance for this since there is no feature within the safety_distance (e.g. can't land inside the safety distance)
            grid_range = [
                int(np.clip(north - d_north - north_min, 0, north_size - 1)),
                int(np.clip(north + d_north - north_min, 0, north_size - 1)),
                int(np.clip(east - d_east - east_min, 0, east_size - 1)),
                int(np.clip(east + d_east - east_min, 0, east_size - 1)),
            ]
            grid[grid_range[0]:grid_range[1] + 1, grid_range[2]:grid_range[3] + 1] = obstacle_height_no_safety


    return grid, int(north_min), int(east_min)


class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (1, 0, 1)
    SOUTH = (-1, 0, 1)
    NORTH_EAST = (1, 1, np.sqrt(2))
    SOUTH_EAST = (-1, 1, np.sqrt(2))
    SOUTH_WEST = (-1, -1, np.sqrt(2))
    NORTH_WEST = (1, -1, np.sqrt(2))

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid_actions = list(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle

    if x - 1 < 0 or grid[x - 1, y] == -1:
        valid_actions.remove(Action.SOUTH)
    if x + 1 > n or grid[x + 1, y] == -1:
        valid_actions.remove(Action.NORTH)
    if y - 1 < 0 or grid[x, y - 1] == -1:
        valid_actions.remove(Action.WEST)
    if y + 1 > m or grid[x, y + 1] == -1:
        valid_actions.remove(Action.EAST)
    if x + 1 > n or y + 1 > m or grid[x+1, y+1] == -1:
        valid_actions.remove(Action.NORTH_EAST)
    if x - 1 < 0 or y + 1 > m or grid[x-1, y+1] == -1:
        valid_actions.remove(Action.SOUTH_EAST)
    if x - 1 < 0 or y - 1 < 0 or grid[x-1, y-1] == -1:
        valid_actions.remove(Action.SOUTH_WEST)
    if x + 1 > n or y - 1 < 0 or grid[x+1, y-1] == -1:
        valid_actions.remove(Action.NORTH_WEST)

    return valid_actions


def a_star(grid, h, start, goal):

    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False

    # The current implementation will not find a path if the goal is an obstacle
    # (and will take a lot of time to determine that there is no path)
    if grid[goal[0], goal[1]] == -1:
        print('The goal is within an obstacle...not possible to get there')
        return path, path_cost
    
    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:              
            current_cost = branch[current_node][0]
            
        if current_node == goal:        
            print('Found a path.')
            found = True
            break
        else:
            for action in valid_actions(grid, current_node):
                # get the tuple representation
                da = action.delta
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                branch_cost = current_cost + action.cost
                queue_cost = branch_cost + h(next_node, goal)
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost, next_node))
             
    if found:
        # retrace steps, backwards (goal to start)
        path.append(goal)
        if len(branch) > 0:
            n = goal
            path_cost = branch[n][0]
            while branch[n][1] != start:
                path.append(branch[n][1])
                n = branch[n][1]
            path.append(branch[n][1])
        else: # there is only 1 point (start = goal)
            path_cost = 0
            # goal was already appended to the path
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************') 
    return path[::-1], path_cost


def heuristic(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position))


# Collinearity algorithm used for waypoint path pruning
# p inputs are sequential waypoints on the path
def collinearity_int(p1, p2, p3):
    det_val = (p1[0] * (p2[1] - p3[1])) - (p2[0] * (p1[1] - p3[1])) + (p3[0] * (p1[1] - p2[1]))
    return (det_val == 0)


# Bresenham algorithm used for waypoint path pruning
# xy inputs are waypoints on the path
def bresenham(x1, y1, x2, y2):
    """
    Generalized to work for all x1, x2, y1, y2 which must all be integers
    """
    # x2 is assumed to be >= x1; if not, swap the points so this is true
    if x1 > x2:
        x1, y1, x2, y2 = x2, y2, x1, y1

    cells = []
    cells.append([x1, y1])

    dx = x2 - x1
    x_inc = 1  # this is always true since x2 >= x1 (per swap logic above)
    dy = y2 - y1
    if dy > 0:
        y_inc = 1
    else:  # dy <= 0
        y_inc = -1

    # equation used here (lhs is the left side of this, rhs is the right side of this)
    # dy > 0: cur_y + y_inc <= y1 + (dy/dx)*((cur_x + x_inc) - x1)
    # dy <= 0: cur_y + y_inc >= y1 + (dy/dx)*((cur_x + x_inc) - x1)
    # if so, increment cur_y
    # if not (or equal), increment cur_x
    # note that both cur_x and cur_y can be increment if the inequality is equal
    # note dx can not be negative (per swap logic above)

    # cache this since it does not change
    rhs_offset = dx * y1

    cur_x = x1
    cur_y = y1

    while (cur_x, cur_y) != (x2, y2):
        lhs = dx * (cur_y + y_inc)
        rhs = rhs_offset + (dy * (cur_x + x_inc - x1))

        # be conservative
        # commented out since a safety margin will be used
        '''
        if lhs == rhs:
            cells.append([(cur_x + 1),cur_y])
            cells.append([cur_x,(cur_y + 1)])
        '''

        if dy > 0:
            if lhs <= rhs:
                cur_y += y_inc
            if lhs >= rhs:
                cur_x += x_inc
        else:  # dy <= 0
            if lhs <= rhs:
                cur_x += x_inc
            if lhs >= rhs:
                cur_y += y_inc

        cells.append([cur_x, cur_y])

    return np.array(cells)


# Prunes input path using collinearity check
def prune_path_collinearity(path):
    if len(path) > 2:
        pruned_path = list()
        pruned_path.append(path[0])
        for i in range(len(path) - 2):
            if not collinearity_int(path[i], path[i+1], path[i+2]):
                pruned_path.append(path[i+1])
        pruned_path.append(path[-1])
    else:
        pruned_path = path

    return pruned_path


# Prunes input path using Bresenham where obstacles are defined by the input grid
def prune_path_bresenham(path, grid):
    if len(path) > 2:
        pruned_path = list()
        pruned_path.append(path[0])
        start = path[0]

        for i in range(len(path) - 2):
            ray = bresenham(start[0], start[1], path[i + 2][0], path[i + 2][1])
            for p in ray:
                if grid[p[0], p[1]] == -1:
                    pruned_path.append(path[i + 1])
                    start = path[i + 1]
                    break

        pruned_path.append(path[-1])
    else:
        pruned_path = path

    return pruned_path


# Computes heading (in radians) from point p1 to point p2
def plane_heading(p1_x, p1_y, p2_x, p2_y):
    return math.atan2(p2_y-p1_y, p2_x-p1_x)
