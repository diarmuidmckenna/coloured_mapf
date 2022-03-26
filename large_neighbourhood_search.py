from single_agent_planner import get_makespan
import random
from cbs import CBSSolver
from prioritized import PrioritizedPlanningSolver
from assignment import Assignment
class Large_neighbourhood_search:
    def __init__(self, my_map):
        self.my_map = my_map
    def generate_constraints(self,paths, assigned_targets, team, constraints):
        for path in paths:
            for index in range(len(path)):
                for j in range(len(assigned_targets)):
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

    def generate_constraints_for_same_team(self, neighbourhood, path, constraints):
        for index in range(len(path)):
            for j in range(len(neighbourhood)):
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

    def team_heuristic(self, result, assigned_targets, replanner, n=4):
        #####################################################################
        largest_makespan = 0
        new_result = result.copy()
        # Find team, with largest makespan 
        for team in result.keys():
            team_makespan = get_makespan(result[team])
            if team_makespan>largest_makespan:
                largest_makespan=team_makespan
                largest_makespan_team= team
        print(largest_makespan_team)
        print("Initial Makespan " + str(largest_makespan))
        # find index(agent) w/ longest path in largest makespan team
        if len(result[largest_makespan_team])<=n:
            neighbourhood = []
            for i in range(len(result[largest_makespan_team])):
                neighbourhood.append(i)
        else:
            neighbourhood = random.sample(range(1, len(result[largest_makespan_team]+1)), n)
        # generate a random neighbourhood of size n, longest_path_index, may not be in here but if not we will remove one and add it
        longest_path = len(result[largest_makespan_team][0])
        longest_path_index = 0
        for path in range(len(result[largest_makespan_team])):
            if len(result[largest_makespan_team][path])>longest_path:
                longest_path= len(result[largest_makespan_team][path])
                longest_path_index = path
        if longest_path_index not in neighbourhood:
            neighbourhood.pop()
            neighbourhood.append(longest_path_index)
        # shuffle target assignments for that team
        starts = []
        goals = []
        #for start, goal in assigned_targets[largest_makespan_team]:
        #    starts.append(start)
        #    goals.append(goal)
        for agent in range(len(assigned_targets[largest_makespan_team])):
            if agent in neighbourhood:
                starts.append(assigned_targets[largest_makespan_team][agent][0])
                goals.append(assigned_targets[largest_makespan_team][agent][1])
        random.shuffle(starts)
        random.shuffle(goals)
        target_assigner = Assignment(starts, goals)
        new_starts, new_goals = target_assigner.greedy_target_assignment()
        # for every other team, for every agent in that team, generate constraints so that neighbourhood team treats other teams as moving obsacles
        # shuffle target assignments
        print(neighbourhood)
        constraints = []
        for team in result.keys():
            if team==largest_makespan_team:
                for agent in range(len(result[largest_makespan_team])):
                    if agent not in neighbourhood:
                        constraints = self.generate_constraints_for_same_team(neighbourhood, result[largest_makespan_team][agent], constraints)
            else:
                constraints = self.generate_constraints(result[team],assigned_targets[team], largest_makespan_team, constraints)
        # replan the paths for that team
        if replanner=="prioritized":
            solver = PrioritizedPlanningSolver(self.my_map, new_starts, new_goals)
            paths = solver.find_solution(extra_constraints=constraints)
        else:
            solver = CBSSolver(self.my_map, new_starts, new_goals)
            paths = solver.find_solution(extra_constraints=constraints)
        # if the new makespan of the team is large
        new_makespan = get_makespan(paths)
        if new_makespan<largest_makespan:
            new_result[largest_makespan_team] = paths
            # change target assignments dictionary to reflect new changes
            assigned_targets[largest_makespan_team]=[]
            for agent in paths:
                assigned_targets[largest_makespan_team].append((agent[0],agent[-1]))
        largest_makespan = 0
        for team in new_result.keys():
            team_makespan = get_makespan(new_result[team])
            if team_makespan>largest_makespan:
                largest_makespan=team_makespan
                largest_makespan_team= team
        print("New Makespan " + str(largest_makespan))
        return new_result, assigned_targets