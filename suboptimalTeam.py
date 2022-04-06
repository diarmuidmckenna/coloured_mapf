from single_agent_planner import get_makespan
import time as timer
import heapq
import numpy as np
from prioritized import PrioritizedPlanningSolver
import time
from writeResults import writeResults, writeResultsForExperiment1
from cbs import CBSSolver
from assignment import hungarian, greedy_target_assignment
from large_neighbourhood_search import Large_neighbourhood_search



class suboptimalTeam:
    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - dictionary where keys are team numbers and values are lists of start positions e.g (x,y)
        goals       - dictionary where keys are team numbers and values are lists of goal positions
        assigned_targets - dict where keys are team numbers and values are lists of nested tuples represented coupled start and goal positions
        """
        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.assigned_targets = dict()
        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []
    
    def get_cost(self,paths):
        """
        paths is a dictionary where keys are teama numbers and values are nested lists"""
        makespan = 0
        for team in paths.keys():
            new_makepan = get_makespan(paths[team])
            if new_makepan>makespan:
                makespan=new_makepan
        return makespan

    def detect_collision(self, team1, team2):
        """
        team1 - list of paths, one for each agent in the team
        team2 - list of paths, one for each agent in the team"""
        for path1 in team1:
            for path2 in team2:
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

                



    def detect_collisions(self, paths):
        """
        paths - dictionary where keys are team numbers and values are lists of paths"""
        collisions = []
        for team1 in paths.keys():
            for team2 in paths.keys():
                if team1 != team2:
                    collision = self.detect_collision(paths[team1], paths[team2])
                    if collision is not None:
                        collisions.append({'t1': team1, 't2': team2, 'loc': collision['loc'],'timestep': collision['time']}) 
        return collisions

    def standard_splitting(self, collision):
        if len(collision['loc'])==1:
        # vertex collision
            return[{'team': collision['t1'], 'loc': collision['loc'], 'timestep': collision['timestep']},
        {'team': collision['t2'], 'loc': collision['loc'], 'timestep': collision['timestep']}]
        else:
        # edge collision
            return[{'team': collision['t1'], 'loc': collision['loc'], 'timestep': collision['timestep']},
        {'team': collision['t2'], 'loc': collision['loc'], 'timestep': collision['timestep']}]


    def disjoint_splitting(self, collision):
        pass


    def target_assignment(self, starts, goals, method="hungarian"):
        #target_assigner = Assignment(starts, goals)
        assigned_targets = []
        if method=="greedy":
            assigned_targets=greedy_target_assignment(starts, goals)
        else:
            assigned_targets=hungarian(starts, goals)
        return assigned_targets

    

    def push_node(self, node):
        heapq.heappush(self.open_list, (len(node['collisions']) ,self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        #print(self.num_of_expanded)
        return node

    def classic_LNS(self, node):
    # ---------------------------------------------------------------------------------------------------------
        # applying LNS to solution
        for team in self.assigned_targets.keys(): 
            if node['cost'] < get_makespan(node['paths'][team]):
                node['cost'] = get_makespan(node['paths'][team])
        #print(node['cost'])
        num_agents_per_team = dict()
        # have a dictionary of offsets i.e. tuples with starting index and ending index
        offset = 0
        for key in node['paths'].keys():
            #print(offset)
            num_agents_per_team[key] = (offset, offset+len(node['paths'][key]))
            offset += len(node['paths'][key])
        # print initial makespan
        # get #agents on each team
        # pass in node[paths].values()
        #print(node['paths'].values())
        LNS_starts=[]
        LNS_goals=[] 
        LNS_paths=[]
        for key in node['paths'].keys():
            for path in node['paths'][key]:
                #print(path)
                LNS_starts.append(path[0])
                LNS_goals.append(path[-1])
                LNS_paths.append(path)
        #print(len(LNS_paths))
        #print(len(LNS_goals))
        solver = PrioritizedPlanningSolver(self.my_map, LNS_starts, LNS_goals)
        #print((node['paths'].values()))
        new_paths = solver.LNS(LNS_paths, 4)
        for key in node['paths'].keys():
            #print(num_agents_per_team[key])
            node['paths'][key] = new_paths[num_agents_per_team[key][0]:num_agents_per_team[key][1]]



    def find_solution(self, LNS, target_assignment):
        start_time = time.perf_counter()
        for key in self.starts.keys():
            team_starts, team_goals = self.target_assignment(self.starts[key], self.goals[key], method=target_assignment)
            self.assigned_targets[key] = []
            for index in range(len(team_starts)):
                self.assigned_targets[key].append((team_starts[index], team_goals[index]))
        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': dict(),
                'collisions': []}
        ###### LOW LEVEL
        for team in self.assigned_targets.keys():  # Find initial path for each team, using prioritized planning for each team
            goals = []
            starts = []
            for agent in self.assigned_targets[team]:
                starts.append(agent[0])
                goals.append(agent[1])
            solver = PrioritizedPlanningSolver(self.my_map, starts, goals) # edit prioritized to allow passed in constraints
            try:
                paths = solver.find_solution()
                root['paths'][team]=paths
            except: 
                return "NO SOLUTIONS"
        ##### END OF LOW LEVEL
        root['cost'] += self.get_cost(root['paths'])
        root['collisions'] = self.detect_collisions(root['paths'])
        self.push_node(root)
        while len(self.open_list)>0 and time.perf_counter()-start_time<180:
            node = self.pop_node()
            node['collisions'] = self.detect_collisions(node['paths'])
            if len(node['collisions']) == 0:
                node['cost'] = self.get_cost(node['paths'])
                if LNS=="classic":
                    self.classic_LNS(node)
                elif LNS=="team":
                    initial_cost = node['cost']
                    LNS = Large_neighbourhood_search(self.my_map)
                    initial_result, initial_target_assignment = node['paths'], self.assigned_targets
                    for replanner in ["prioritized", "cbs"]:
                        iterations = 0
                        start = time.perf_counter()
                        n=0
                        result, assigned_targets =  initial_result, initial_target_assignment 
                        done=False
                        while done==False:
                            iterations +=1 
                            result, assigned_targets, n = LNS.team_heuristic(result, assigned_targets, replanner, n)
                            if float(time.perf_counter()-start<float(60)):
                                new_cost = self.get_cost(result)
                                self.assigned_targets=assigned_targets
                            else:
                                writeResultsForExperiment1(initial_cost, "suboptimalCBS")
                                writeResults(initial_cost, new_cost, "suboptimal", replanner, iterations)
                                done=True
                else: 
                    writeResultsForExperiment1(node['cost'], "suboptimalCBS")
                return node['paths'], self.assigned_targets
            collision = node['collisions'][0]
            constraints = self.standard_splitting(collision)
            for con in constraints:
                child_node = {}
                child_node['paths'] = node['paths'].copy()
                child_node['constraints'] = node['constraints'].copy()
                child_node['constraints'].append(con)
                team = con['team'] 
                starts =[]
                goals=[]           
                for agent in self.assigned_targets[team]:
                    starts.append(agent[0])
                    goals.append(agent[1])
                solver = PrioritizedPlanningSolver(self.my_map, starts, goals)
                paths = solver.find_solution(extra_constraints=child_node['constraints'])
                child_node['paths'][team] = paths
                child_node['collisions'] = self.detect_collisions(child_node['paths'])
                child_node['cost']=self.get_cost(child_node['paths'])
                self.push_node(child_node)  
        writeResultsForExperiment1("NO SOLUTION", "suboptimalCBS")       
        return None, None                
