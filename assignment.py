import numpy as np
from scipy.optimize import linear_sum_assignment
from single_agent_planner import a_star, compute_heuristics, get_makespan
import math
def euclidean_distance(start, goal):
    return math.sqrt((start[0]-goal[0])**2 + (start[1]-goal[1])**2)

def greedy_target_assignment(starts, goals):
    """"
    starts - list of (x,y) coordinatates 
    goals - list of (x,y)
    return object is a list of nested tuples ((x1,y1) (x2,y2))
    possibly use hungarian algorithm
    """
    return_starts = []
    return_goals = []
    c_goals = goals.copy()
    for x in starts:
        minimum = 9999999
        for y in c_goals:
            distance = euclidean_distance(x, y)
            if distance < minimum:
                minimum = distance
                minimum_distance = y
        c_goals.remove(minimum_distance)
        return_starts.append(x)
        return_goals.append(minimum_distance)
    return return_starts, return_goals


def hungarian(starts, goals):
    # populate the costs matrix, which will be a n x n matrix in every case
    # entry at cost[i][j] is the euclidean distance from start position i to goal position i 
    cost = []
    return_starts = []
    return_goals = []
    for start in range(len(starts)):
        row_cost = []
        for goal in range(len(goals)):
            row_cost.append(euclidean_distance(starts[start],goals[goal]))
        cost.append(row_cost)
    cost_array = np.array(cost)
    row_ind, col_ind = linear_sum_assignment(cost_array)
    for start in range(len(row_ind)):
        #assigned_targets.append((self.starts[row_ind[start]], self.goals[col_ind[start]]))
        return_starts.append(starts[row_ind[start]])
        return_goals.append(goals[col_ind[start]])
    return return_starts, return_goals

def optimal_hungarian(my_map, starts, goals):
    # populate the costs matrix, which will be a n x n matrix in every case
    # entry at cost[i][j] is the euclidean distance from start position i to goal position i 
    cost = []
    return_starts = []
    return_goals = []
    heuristics = []
    for goal in goals:
        #print(goal)
        heuristics.append(compute_heuristics(my_map, goal))
    for start in range(len(starts)):
        row_cost = []
        for goal in range(len(goals)):
            row_cost.append(get_makespan(a_star(my_map, starts[start] ,goals[goal], heuristics[goal], start, [])))
        cost.append(row_cost)
    cost_array = np.array(cost)
    row_ind, col_ind = linear_sum_assignment(cost_array)
    for start in range(len(row_ind)):
        #assigned_targets.append((self.starts[row_ind[start]], self.goals[col_ind[start]]))
        return_starts.append(starts[row_ind[start]])
        return_goals.append(goals[col_ind[start]])
    return return_starts, return_goals


def makespan_greedy_target_assignment(my_map, starts, goals):
    """"
    starts - list of (x,y) coordinatates 
    goals - list of (x,y)
    return object is a list of nested tuples ((x1,y1) (x2,y2))
    possibly use hungarian algorithm
    """
    return_starts = []
    return_goals = []
    heuristics = []
    for goal in goals:
        #print(goal)
        heuristics.append(compute_heuristics(my_map, goal))
    c_goals = goals.copy()
    for x in range(len(starts)):
        minimum = 9999999
        for y in range(len(c_goals)):
            distance = get_makespan(a_star(my_map, starts[x] ,goals[y], heuristics[y], x, []))
            if distance < minimum:
                minimum = distance
                minimum_distance = c_goals[y]
        c_goals.remove(minimum_distance)
        return_starts.append(starts[x])
        return_goals.append(minimum_distance)
    return return_starts, return_goals
