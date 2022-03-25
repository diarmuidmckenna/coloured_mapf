import heapq

def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst

def get_makespan(paths):
    maximum = 0
    for path in paths:
        if len(path)>maximum:
            maximum = len(path)
    return maximum


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
               continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))
    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(agent, constraints):
    ##############################
    # Task 1.2/1.3: Return a table that constains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.
    # find a max timestep
    # iterate through every i <= timestep
    # for every constraint that equals i, append it to the list stored at constraint[i]
    # issue needs to be resolved for 
    constraint_table = []
    goal_locs = {}
    for i in range(0,4000):
        timeStepConstraints = []
        for con in constraints:
            if con['timestep']==i and con['agent']==agent:
                if (len(con['loc'])==1): 
                    timeStepConstraints.append((con['loc'][0][0], con['loc'][0][1]))
                elif (len(con['loc'])==2):
                    timeStepConstraints.append(((con['loc'][0][0], con['loc'][0][1]), (con['loc'][1][0], con['loc'][1][1])))
            elif type(con['timestep'])==tuple and con['agent']==agent:
                try:
                    goal_locs[con['timestep'][0]].append(((con['loc'][0][0], con['loc'][0][1])))
                except:
                    goal_locs[con['timestep'][0]] = [((con['loc'][0][0], con['loc'][0][1]))]     
        for j in goal_locs.keys():
            if j <= i:
                for item in goal_locs[j]: 
                    timeStepConstraints.append(item)
        constraint_table.append(timeStepConstraints)
    return constraint_table


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.
    #print(curr_loc, next_loc, constraint_table[next_time] )
    for x in constraint_table[next_time]:
        if type(x[0])==tuple: # move movement is nested tuple
            if ((curr_loc, next_loc)) == x: # traversing same edge
                return True
            elif ((next_loc, curr_loc))  == x: # swapping vertices
                return True
            elif next_loc==x[1]: ## travelling to same vertex
                return True
        else: # wait movement
            if next_loc==x: # next_loc s the location of an agent waiting on a vertex
                return True
    return False

def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """
    #print(constraints)
    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.
    constraint_table = build_constraint_table(agent, constraints)
    open_list = []
    closed_list = dict()
    earliest_goal_timestep = 0
    h_value = h_values[start_loc]
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'time': 0,  'parent': None}
    push_node(open_list, root)
    closed_list[((root['loc']), root['time'])] = root
    while len(open_list) > 0:
        curr = pop_node(open_list)
        #############################
        # Task 1.4: Adjust the goal test condition to handle goal constraints
        # Can only return if no agent with a higher priority will move to that vertex in the future
        if curr['loc'] == goal_loc:
            moreConstraints = False
            for timeStep in range(curr['time'], 200):
                if goal_loc in constraint_table[timeStep]:
                    moreConstraints = True
            if moreConstraints==False:
                return get_path(curr)
        for dir in range(5):
            child_loc = move(curr['loc'], dir)
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
               continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            if is_constrained(curr['loc'], child_loc, curr['time']+1, constraint_table)==False:
                child = {'loc': child_loc,
                        'g_val': curr['g_val'] + 1,
                        'h_val': h_values[child_loc],
                        'time': curr['time']+1,
                        'parent': curr}
                if (child['loc'],child['time']) in closed_list:
                    existing_node = closed_list[((child['loc']), child['time'])]
                    if compare_nodes(child, existing_node):
                        closed_list[((child['loc']), child['time'])] = child
                        push_node(open_list, child)
                else:
                    closed_list[((child['loc']),  child['time'])] = child
                    push_node(open_list, child)
    return None  # Failed to find solutions
