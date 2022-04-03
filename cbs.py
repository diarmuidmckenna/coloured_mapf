import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_makespan


def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.
    if len(path1) > len(path2):
        for time in range(len(path1)-1):
            # check for vertex collisions
            p1_curr_loc = path1[time]
            p1_next_loc = path1[time+1]
            if time >= len(path2)-1:
                p2_curr_loc = path2[-1]
                p2_next_loc = path2[-1]
            else:
                p2_curr_loc = path2[time]
                p2_next_loc = path2[time+1]
            if p1_next_loc==p2_next_loc:
                return {'time': time+1, 'loc':[p1_next_loc]}
            elif (p1_curr_loc, p1_next_loc) == (p2_next_loc, p2_curr_loc):
                return {'time': time+1, 'loc':[(p1_curr_loc, p1_next_loc)]}
    else: 
        for time in range(len(path2)-1):
            # check for vertex collisions
            p2_curr_loc = path2[time]
            p2_next_loc = path2[time+1]
            if time >= len(path1)-1:
                p1_curr_loc = path1[-1]
                p1_next_loc = path1[-1]
            else:
                p1_curr_loc = path1[time]
                p1_next_loc = path1[time+1]
            if p1_next_loc==p2_next_loc:
                return {'time': time+1, 'loc':[p2_next_loc]}
            elif (p2_curr_loc, p2_next_loc) == (p1_next_loc, p1_curr_loc):
                return {'time': time+1, 'loc':[(p2_curr_loc, p2_next_loc)]}
    return None





def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    # given a set of paths, add goal_locs as permanent obstacles once reached by agent
    collisions = []
    for i in range(len(paths)):
        for j in range(i+1, len(paths)):
            collision = detect_collision(paths[i], paths[j])
            if collision is not None:
                collisions.append({'a1': i, 'a2': j, 'loc': collision['loc'],'timestep': collision['time']}) 
    return collisions


def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep
    if len(collision['loc'])==1:
        # vertex collision
        return[{'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep']},
        {'agent': collision['a2'], 'loc': collision['loc'], 'timestep': collision['timestep']}]
    else:
        # edge collision
        return[{'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep']},
        {'agent': collision['a2'], 'loc': collision['loc'], 'timestep': collision['timestep']}]


def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly

    pass


class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], self.num_of_generated, node))
        #print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, id, node = heapq.heappop(self.open_list)
        #print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True, extra_constraints=[]):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
            'constraints': [],
            'paths': [],
            'collisions': []}
        for con in extra_constraints:
                for agent in range(self.num_of_agents):
                    root['constraints'].append({
                            'agent':  agent,
                            'loc': con['loc'],
                            'timestep': con['timestep']
                        })
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)
        root['cost'] = get_makespan(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)
        start = timer.perf_counter()
        while len(self.open_list)>0 and timer.perf_counter-start<60:
            node = self.pop_node()
            node['collisions'] = detect_collisions(node['paths'])
            if len(node['collisions']) == 0:
                node['cost'] = get_makespan(node['paths'])
                #print(node['cost'])
                return node['paths']
            collision = node['collisions'][0]
            constraints = standard_splitting(collision)
            for con in constraints:
                child_node = {}
                child_node['paths'] = node['paths'].copy()
                child_node['constraints'] = node['constraints'].copy()
                child_node['constraints'].append(con)
                agent = con['agent']
                path = a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent],
                          agent, child_node['constraints'])
                if len(path)!=0:
                    child_node['paths'][agent] = path
                    child_node['collisions'] = detect_collisions(child_node['paths'])
                    child_node['cost'] = get_makespan(child_node['paths'])
                    self.push_node(child_node)  
        raise BaseException("No solutions")                    


    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_makespan(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))

