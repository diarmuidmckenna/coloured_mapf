import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost, get_makespan
import random
class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """
        ##print(my_map)

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)
        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))
    
    def calculate_delays(self, result):
        delays = []
        for agent in range(len(result)):
            actual_cost = len(result[agent])-1
            unrestricted_cost = len(a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent],
                          agent, []))-1
            delay= actual_cost-unrestricted_cost
            delays.append(delay)
        return delays

    def restrictedRandomWalk(self, agent, result, neighbourhood, size): 
        directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0,0)] 
        t = random.randint(0, len(result[agent])-1) # pick a random timestep 
        x = result[agent][t] # vertex at timestep t
        possible_locations = [] 
        for y in range(5): # move and wait coordinates stored in directions
            new_location = (x[0]+directions[y][0],x[1]+directions[y][1]) # add coordinations to x to yield the new
            # check if new_location exists on the map and also is a traversible edge
            try:
                if new_location[0]>=0 and new_location[0]<len(self.my_map) \
                and new_location[1] >= 0 or new_location[1] < len(self.my_map[0]) and  self.my_map[new_location[0]][new_location[1]] is False:
                    distance_to_goal = t+1+len(a_star(self.my_map, new_location, self.goals[agent], self.heuristics[agent], agent, []))-1 
                    # distance to goal is the timestep +1 plus the shortest path from new_location to goal_location 
                    if distance_to_goal < len(result[agent])-1: # if distance to goal is smaller than current path 
                        #then new_location will possibly get the agent to goal in a shorter time
                        possible_locations.append(new_location)
            except:
                pass
        while len(possible_locations)>0 and len(neighbourhood)<size:
            random.shuffle(possible_locations) # pick a random possible location
            y = possible_locations[0]
            for other_agent in result: # iterate through paths
                if result.index(other_agent)!=agent: # agent can't collide with itself
                    try: # try is used in case of an index error
                        if len(other_agent)-1<t+1: # if agent is waiting in its goal location 
                            if y==other_agent[-1]: # if its last location(goal location) is on the path 
                                neighbourhood.append(result.index(other_agent))
                        else:    
                            edge_traversed = other_agent[t:t+2]
                            # 2 agents move to same vertex at same timestep
                            if y==other_agent[t+1]:  
                                neighbourhood.append(result.index(other_agent))
                            # 2 agents try to traverse the same edge at the same time   
                            elif (edge_traversed[0])==(x,y):
                                neighbourhood.append(result.index(other_agent))
                            # 2 agents swap vertices    
                            elif (edge_traversed[0])==(y,x):
                                neighbourhood.append(result.index(other_agent))
                    except:
                        pass
            possible_locations.remove(y)
        return neighbourhood
            

    def generate_neighbourhood(self, result, size, type, tabu_list):
        neighbourhood = []
        if type == "RANDOM":
            neighbourhood = random.sample(range(0,self.num_of_agents), size)
        elif type == "AGENT": 
            # calculate the delay for each agent, if agent 0 is chosen delay is 0  
            # tabu list is globally maintained and is an input to each iteration of agent based neighbourhood generation
            delays = self.calculate_delays(result)
            noDelays = True
            for a in range(len(result)):
                if delays[a]!=0: 
                    noDelays= False
            if noDelays: # very simple, if no delays then optimal path is already obtained
                return neighbourhood, tabu_list
            max_delay_agent = delays.index(max(delays)) # agent with largest delay 
            while max_delay_agent in tabu_list: # if agent is in tabu list, choose a different agent
                max_delay = delays[max_delay_agent] # make a copy of the value of max_delay
                delays[max_delay_agent] = 0 # temporarily set to 0
                max_delay_agent = delays.index(max(delays)) # get the new max delay 
                delays[max_delay_agent] = max_delay # reset the agent in tabu_list back to 0
            if len(tabu_list) == len(result) or delays[max_delay_agent]==0: # if tabu_list has all of the agents or the current "max" delay agent has a delay of 0
                # then reset tabu list to 0 because an agent with a delay of 0 doesn't need to be done again
                tabu_list = []
                max_delay_agent = delays.index(max(delays)) 
                tabu_list.append(max_delay_agent)
            neighbourhood.append(max_delay_agent) # add max-delay agent to the list 
            # add max delay agent to neighbourhood and tabu list
            count =0 # counter in case max_delay agent has less than size agents blocking them from a shorter path
            random_agent = max_delay_agent # this value is changed in the while loop so making a copy is necessary 
            while len(neighbourhood) < size and count <10:
                neighbourhood = self.restrictedRandomWalk(random_agent, result, neighbourhood, size)
                random.shuffle(neighbourhood)
                random_agent = neighbourhood[0]
                count +=1
        elif "MAP": # finding intersection vertices
            vertices = [] # is a list of tuples corresponding to a x y coordinates of intersecton vertices
            for x in range(1,len(self.my_map)-1): # we don't want any border vertices
                for y in range(1,len(self.my_map[x])-1): 
                    if self.my_map[x][y] is False and self.my_map[x][y-1] is False and self.my_map[x-1][y] is False and self.my_map[x+1][y] is False and self.my_map[x][y+1] is False:
                        vertices.append((x,y)) # finding intersection vertices and adding them to a list
            random.shuffle(vertices) # in order to randomly select one of the intersection vertices 
            queue = [] # 
            visited = [] # used to keep track of vertices already covered 
            queue.append(vertices[0]) # add a random intersection vertex to the queue
            directions = [(0, -1), (1, 0), (0, 1), (-1, 0)] # used to move the vertex and check all vertices around 
            while len(neighbourhood)<size and len(queue)>0: 
                x = queue.pop(0)
                visited.append(x)
                if x in vertices: ## check if its intersection vertex
                    neighbourhood = self.get_intersection_agents(x, result, neighbourhood, size) # get agents that traverse this vertex
                for y in range(4):
                    new_location = (x[0]+directions[y][0],x[1]+directions[y][1])
                    try:
                        if new_location not in visited and new_location[0]>=0 and new_location[0]<len(self.my_map) \
                        and new_location[1] >= 0 or new_location[1] < len(self.my_map[0]) and  self.my_map[new_location[0]][new_location[1]] is False:
                            queue.append(new_location)
                            visited.append(new_location)
                    except:
                        pass
        return neighbourhood, tabu_list
    
    def get_intersection_agents(self, vertex, paths,neighbourhood, size):
        times_visited = []
        for agent in paths:
            try:
                times_visited.append(agent.index(vertex))
            except:
                pass
        if len(times_visited)==0:
            pass
        else:
            max_t = max(times_visited)
            t = random.randint(0, max_t)
            delta = 0
            while len(neighbourhood)<size and delta<=max(t, max_t-t):
                for agent in range(len(paths)):
                    if agent in neighbourhood:
                        pass
                    else:
                        try:
                            if vertex == paths[agent][t+delta]:
                                if len(neighbourhood)==3:
                                    break
                                else:
                                    neighbourhood.append(agent)
                            elif vertex == paths[agent][t-delta]: 
                                if len(neighbourhood)==3:
                                    break
                                else:
                                    neighbourhood.append(agent)
                        except:
                            pass
                delta +=1
        return neighbourhood
                        


    def replan_paths_for_neighbourhood(self, result, neighbourhood):
        # result is a list of paths, path at index x is the path for agent x 
        # neighbourhood is an arbirtrary sized list of agents, len(neighbourhood) < len(result)
        constraints = [] # reset constraints to empty list
        for i in range(self.num_of_agents): # rebuild new constraints list
            if i in neighbourhood: # don't add contraints for agents in neighborhood
                pass
            else:
                constraints = self.generate_constraints(i, result[i], constraints) # add constraints for already planned paths
                # so agents that aren't in LNS must add constraints for agents with lower priority and also agents in neighbourhood
                constraints = self.generate_constraints_for_neighbourhood(neighbourhood, result[i], constraints ) # if agent 0 is chosen, agents with lower priority still must include it on their constraint tree 
        new_results = result.copy()    
        for r in range(len(neighbourhood)): ## attempting to fix LNS constraints as agents in same neighbourhood have collisions
            path = a_star(self.my_map, self.starts[neighbourhood[r]], self.goals[neighbourhood[r]], self.heuristics[neighbourhood[r]],
                            neighbourhood[r], constraints)
            if path is None:
                return result
            constraints = self.generate_constraints_for_other_neighbours(r, neighbourhood, path, constraints )
            new_results[neighbourhood[r]] = path
        if get_sum_of_cost(result) <= get_sum_of_cost(new_results):
            return result
        elif get_sum_of_cost(result) == get_sum_of_cost(new_results) and get_makespan(result) < get_makespan(new_results):
            return result
        return new_results
        
    
    def generate_constraints(self, i, path, constraints):
        for index in range(len(path)):
            for j in range(i+1, self.num_of_agents):
                if (index<len(path)-1):
                    constraints.append({
                        'agent': j,
                        'loc': [ path[index] ],
                        'timestep': index
                    })
                    if index!=0:
                        constraints.append({
                            'agent': j,
                            'loc': [path[index-1], path[index]],
                            'timestep': index
                        })
                else:
                    constraints.append({
                            'agent': j,
                            'loc': [path[len(path)-2], path[len(path)-1]],
                            'timestep': index
                        })
                    constraints.append({
                        'agent': j,
                        'loc': [ path[len(path)-1] ],
                        'timestep': (len(path)-1, "future")
                        })
        return constraints

    def generate_constraints_for_other_neighbours(self, i, LNS, path, constraints):
        for index in range(len(path)):
            for j in range(i+1, len(LNS)):
                if (index<len(path)-1): # find a sophisticated way to shuffle the chosen agents such that the constraints can apply to all agents planned after, possibly a LNSAgent List and remove yourself at the enc 
                    constraints.append({
                        'agent': LNS[j],
                        'loc': [ path[index] ],
                        'timestep': index
                    })
                    if index!=0:
                        constraints.append({
                            'agent': LNS[j],
                            'loc': [path[index-1], path[index]],
                            'timestep': index
                        })
                else:
                    constraints.append({
                            'agent': LNS[j],
                            'loc': [path[len(path)-2], path[len(path)-1]],
                            'timestep': index
                        })
                    constraints.append({
                        'agent': LNS[j],
                        'loc': [ path[len(path)-1] ],
                        'timestep': (len(path)-1, "future")
                            })
        return constraints

    def generate_constraints_for_neighbourhood(self, neighbourhood, path, constraints):
        for index in range(len(path)):
            for j in range(len( neighbourhood)):
                if (index<len(path)-1): # find a sophisticated way to shuffle the chosen agents such that the constraints can apply to all agents planned after, possibly a LNSAgent List and remove yourself at the enc 
                    constraints.append({
                        'agent':  neighbourhood[j],
                        'loc': [ path[index] ],
                        'timestep': index
                    })
                    if index!=0:
                        constraints.append({
                            'agent':  neighbourhood[j],
                            'loc': [path[index-1], path[index]],
                            'timestep': index
                        })
                else:
                    constraints.append({
                            'agent':  neighbourhood[j],
                            'loc': [path[len(path)-2], path[len(path)-1]],
                            'timestep': index
                        })
                    constraints.append({
                        'agent':  neighbourhood[j],
                        'loc': [ path[len(path)-1] ],
                        'timestep': (len(path)-1, "future")
                            })
        return constraints




    def find_solution(self,extra_constraints=[]):
        """ Finds paths for all agents from their start locations to their goal locations."""
        constraints=[]
        if len(extra_constraints)>0:
            # generate constraint for all agents
            #pass
            for con in extra_constraints:
                for agent in range(self.num_of_agents):
                    constraints.append({
                            'agent':  agent,
                            'loc': con['loc'],
                            'timestep': con['timestep']
                        })


        result = []
        for i in range(self.num_of_agents):  # Find path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            if path is None:
                raise BaseException('No solutions')
            constraints = self.generate_constraints(i, path, constraints)
            result.append(path)
        return result
        

