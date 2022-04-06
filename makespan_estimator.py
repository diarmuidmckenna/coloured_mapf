from assignment import hungarian, makespan_greedy_target_assignment, optimal_hungarian
from single_agent_planner import compute_heuristics, a_star, get_makespan
from writeResults import writeMakespan 
import random
import argparse
class MakespanEstimator:
    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """
        self.my_map = my_map
        for team in starts.keys():
            starts[team], goals[team] = optimal_hungarian(my_map, starts[team], goals[team])
        self.indexes_of_teams, self.starts, self.goals = self.transform_to_classic_MAPF(starts, goals)
        self.num_of_agents = len(self.goals)
        self.CPU_time = 0
        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(self.my_map, goal))
        paths = []
        for i in range(self.num_of_agents):
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                        i, [])
            paths.append(path)
        makespan = get_makespan(paths)
        for _ in range(400):
            self.my_map = my_map
            for team in starts.keys():
                random.shuffle(starts[team])
                random.shuffle(goals[team])
                starts[team], goals[team] = makespan_greedy_target_assignment(my_map, starts[team], goals[team])
            self.indexes_of_teams, self.starts, self.goals = self.transform_to_classic_MAPF(starts, goals)
            self.num_of_agents = len(self.goals)
            self.CPU_time = 0
            # compute heuristics for the low-level search
            self.heuristics = []
            for goal in self.goals:
                self.heuristics.append(compute_heuristics(self.my_map, goal))
            paths = []
            for i in range(self.num_of_agents):
                path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                            i, [])
                paths.append(path)
            new_makespan = get_makespan(paths)
            if new_makespan<makespan:
                makespan=new_makespan
            
        writeMakespan(makespan)

    def transform_to_classic_MAPF(self, starts, goals):
        """
        starts - dictionary where keys are team numbers and values and lists of tuples denoting starting (x,y) coordinates
        goals - dictionary where keys are team numbers and values and lists of tuples denoting goal (x,y) coordinates
        returns 2 lists: starts and goals and a dictionary which stores what indexes of the start and goals belong to each team
        purpose is to not bias any team but to choose agents priority in a round robin way
        ASSUMPTION - TEAMS MUST ALL HAVE SAME NUMBER OF AGENTS FOR THIS TO BE CORRECT
        """
        new_goals = [] 
        new_starts = []
        indexes_of_teams = dict()
        index = 0
        for agent in range(len(starts[1])):
            for key in starts.keys():
                new_starts.append(starts[key][agent])
                new_goals.append(goals[key][agent])
                try:
                    indexes_of_teams[key].append(index)
                except:
                    indexes_of_teams[key] = [index]
                index +=1 
        return indexes_of_teams, new_starts, new_goals