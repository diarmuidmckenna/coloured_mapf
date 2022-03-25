from assignment import Assignment
from single_agent_planner import compute_heuristics, a_star, get_makespan
from writeResults import writeMakespan 
class MakespanEstimator:
    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """
        for team in starts.keys():
            target_assigner=Assignment(starts[team], goals[team])
            starts[team], goals[team] = target_assigner.hungarian()
            self.indexes_of_teams, self.starts, self.goals = self.transform_to_classic_MAPF(starts, goals)
            self.num_of_agents = len(self.goals)
            self.CPU_time = 0
            # compute heuristics for the low-level search
            self.heuristics = []
            for goal in self.goals:
                self.heuristics.append(compute_heuristics(my_map, goal))
        paths = []
        for team in starts.keys():
            for i in range(starts[team]):  # Find path for each agent
                path = a_star(self.my_map, self.starts[team][i], self.goals[i], self.heuristics[i],
                            i, [])
                paths.append(path)
        makespan = get_makespan(paths)
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

