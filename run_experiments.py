#!/usr/bin/python
import argparse
import glob
from pathlib import Path
from teamPrioritized import TeamPrioritizedPlanningSolver
from single_agent_planner import get_makespan
from suboptimalTeam import suboptimalTeam
from visualize import Animation
#from single_agent_planner import get_sum_of_cost

SOLVER = "CBS"

def print_mapf_instance(my_map, starts, goals):
    print('Start locations')
    print_locations(my_map, starts)
    print('Goal locations')
    print_locations(my_map, goals)


def print_locations(my_map, locations):
    starts_map = [[-1 for _ in range(len(my_map[0]))] for _ in range(len(my_map))]
    for team in locations.keys():
        for (x,y) in locations[team]:
            starts_map[x][y] = team
        to_print = ''
    for x in range(len(my_map)):
        for y in range(len(my_map[0])):
            if starts_map[x][y] >= 0:
                to_print += str(starts_map[x][y]) + ' '
            elif my_map[x][y]:
                to_print += '@ '
            else:
                to_print += '. '
        to_print += '\n'
    print(to_print)


def import_mapf_instance(filename):
    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open(filename, 'r')
    # first line: #rows #columns
    line = f.readline()
    rows, columns = [int(x) for x in line.split(' ')]
    rows = int(rows)
    columns = int(columns)
    # #rows lines with the map
    my_map = []
    for r in range(rows):
        line = f.readline()
        my_map.append([])
        for cell in line:
            if cell == '@' or cell=="T":
                my_map[-1].append(True)
            elif cell == '.':
                my_map[-1].append(False)
        # #teams
    line = f.readline()
    num_teams = int(line)
    # #agents lines with the start/goal positions
    starts = dict()
    goals = dict()
    # starts
    start_count = 0
    goal_count = 0
    for a in range(1, num_teams+1):
        start = []
        goal = []
        line = f.readline()
        line = line.split(' ')
        num_agents = len(line)//2
        for agent in range(num_agents):
            sx = int(line[agent*2])
            sy = int(line[agent*2+1])
            if my_map[sx][sy] is False:
                start_count +=1
                start.append((sx,sy))
            else:
                print(sx,sy) 
        line = f.readline()
        line = line.split(' ')
        for agent in range(num_agents):
            gx = int(line[agent*2])
            gy = int(line[agent*2+1])
            if my_map[gx][gy] is False:
                goal.append((gx,gy))
                goal_count+=1
            else:
                print(gx,gy)
        '''
        if len(start)>len(goal):
            start = start[0:len(goal)]
        elif len(goal)>len(start):
            goal = goal[0:len(start)]
        '''        
        starts[a]= start
        goals[a] = goal
    f.close()
    print(start_count, goal_count)
    return my_map, starts, goals

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs various MAPF algorithms')
    parser.add_argument('--instance', type=str, default=None,
                        help='The name of the instance file(s)')
    parser.add_argument('--replanner', type=str, default=None,
                        help='The solver to use (one of: {CBS,Prioritized}), defaults to ' + str(SOLVER))
    parser.add_argument('--solver', type=str, default=None,
                        help='The solver to use (one of: {suboptimalTeam,Prioritized}), defaults to ' + str(SOLVER))
    args, unknown = parser.parse_known_args()


    result_file = open("results.csv", "w", buffering=1)
    print(sorted(glob.glob(args.instance)))
    for file in sorted(glob.glob(args.instance)):
        print("***Import an instance***")
        print(file)
        my_map, starts, goals = import_mapf_instance(file)
        print_mapf_instance(my_map, starts, goals)
        if args.solver=="suboptimalTeam":
            solver = suboptimalTeam(my_map,  starts, goals)
            paths, assigned_targets = solver.find_solution(args.replanner, LNS="team", target_assignment="hungarian")
            if paths == None:
                print("NO PATHS")
        elif args.solver=="Prioritized":
            solver = TeamPrioritizedPlanningSolver(my_map, starts, goals, team=True)
            paths, assigned_targets = solver.find_solution(args.replanner,LNS="team")
        #print(paths)


        print("***Test paths on a simulation***")          
        #a = Animation(my_map, assigned_targets, paths)
        # animation.save("output.mp4", 1.0)
        #a.show()

        # last update getting collisions, next task is to pass in constraints into prioritized solver