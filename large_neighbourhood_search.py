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
                for j in range(len(assigned_targets[team])):
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

    def team_heuristic(self, result, assigned_targets, replanner):
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
        # shuffle target assignments for that team
        starts = []
        goals = []
        for start, goal in assigned_targets[largest_makespan_team]:
            starts.append(start)
            goals.append(goal)
        random.shuffle(starts)
        random.shuffle(goals)
        target_assigner = Assignment(starts, goals)
        new_starts, new_goals = target_assigner.greedy_target_assignment()
        # for every other team, for every agent in that team, generate constraints so that neighbourhood team treats other teams as moving obsacles
        # shuffle target assignments
        constraints = []
        for team in result.keys():
            if team==largest_makespan_team:
                pass
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