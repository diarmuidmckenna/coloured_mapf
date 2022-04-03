from single_agent_planner import get_makespan
import random
from cbs import CBSSolver
from prioritized import PrioritizedPlanningSolver
from assignment import greedy_target_assignment
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

    def team_heuristic(self, result, assigned_targets, replanner, n):
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
        if n>7:
            # choose another team at random and add them to the neighbourhood
            other_team = random.randint(1,max(result.keys()))
            while other_team==largest_makespan_team:
                other_team = random.randint(1,max(result.keys()))
            neighbourhood = [largest_makespan_team, other_team]
            print(neighbourhood)
            # start, goals are dictionaries
            new_starts = []
            new_goals = []
            for team in neighbourhood:
                team_starts = []
                team_goals = []
                for agent in range(len(assigned_targets[team])):
                    team_starts.append(assigned_targets[team][agent][0])
                    team_goals.append(assigned_targets[team][agent][1])
                random.shuffle(team_starts)
                random.shuffle(team_goals)
                new_team_starts, new_team_goals = greedy_target_assignment(team_starts, team_goals)
                new_starts = new_starts + new_team_starts
                new_goals = new_goals + new_team_goals
            print(new_starts, new_goals)
            constraints = []
            for team in result.keys():
                if team in neighbourhood:
                    pass
                else:
                    constraints = self.generate_constraints(result[team],assigned_targets[team], largest_makespan_team, constraints)
                    constraints = self.generate_constraints(result[team],assigned_targets[team], other_team, constraints)
        else:
            starts = []
            goals = []
            for agent in range(len(assigned_targets[largest_makespan_team])):
                starts.append(assigned_targets[largest_makespan_team][agent][0])
                goals.append(assigned_targets[largest_makespan_team][agent][1])
            random.shuffle(starts)
            #random.shuffle(goals)
            new_starts, new_goals = greedy_target_assignment(starts, goals)
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
            try:
                solver = PrioritizedPlanningSolver(self.my_map, new_starts, new_goals)
                paths = solver.find_solution(extra_constraints=constraints)
            except:
                return result, assigned_targets, n+1
        else:
            try:
                solver = CBSSolver(self.my_map, new_starts, new_goals)
                paths = solver.find_solution(extra_constraints=constraints)
            except:
                return result, assigned_targets, n+1
        # if the new makespan of the team is large
        new_makespan = get_makespan(paths)
        if n>7:
            num_agents_per_team = len(result[1])
            if new_makespan<largest_makespan:
                n=0
                new_result[largest_makespan_team] = paths[0:num_agents_per_team]
                new_result[other_team] = paths[num_agents_per_team:num_agents_per_team*2]
                # change target assignments dictionary to reflect new changes
                for team in neighbourhood:
                    assigned_targets[team]=[]
                    for agent in new_result[team]:
                        assigned_targets[team].append((agent[0],agent[-1]))
            elif new_makespan==largest_makespan:
                n=n+1
                new_result[largest_makespan_team] = paths[0:num_agents_per_team]
                new_result[other_team] = paths[num_agents_per_team:num_agents_per_team*2]
                # change target assignments dictionary to reflect new changes
                for team in neighbourhood:
                    assigned_targets[team]=[]
                    for agent in new_result[team]:
                        assigned_targets[team].append((agent[0],agent[-1]))
            else:
                n=n+1
        else:
            if new_makespan<largest_makespan:
                n=0
                new_result[largest_makespan_team] = paths
                # change target assignments dictionary to reflect new changes
                assigned_targets[largest_makespan_team]=[]
                for agent in paths:
                    assigned_targets[largest_makespan_team].append((agent[0],agent[-1]))
            elif new_makespan==largest_makespan:
                n=n+1
                new_result[largest_makespan_team] = paths
                # change target assignments dictionary to reflect new changes
                assigned_targets[largest_makespan_team]=[]
                for agent in paths:
                    assigned_targets[largest_makespan_team].append((agent[0],agent[-1]))
            else:
                n=n+1
        largest_makespan = 0
        for team in new_result.keys():
            team_makespan = get_makespan(new_result[team])
            if team_makespan>largest_makespan:
                largest_makespan=team_makespan
                largest_makespan_team= team
        print("New Makespan " + str(largest_makespan))
        return new_result, assigned_targets, n
