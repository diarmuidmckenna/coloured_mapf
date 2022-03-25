import argparse
def writeResults( initial, new, solver, replanner):
    parser = argparse.ArgumentParser(description='Runs various MAPF algorithms')
    parser.add_argument('--instance', type=str, default=None,
                        help='The name of the instance file(s)')
    parser.add_argument('--batch', action='store_true', default=False,
                        help='Use batch output instead of animation')
    parser.add_argument('--replanner', type=str, default="Prioritised",
                        help='Use batch output instead of animation')
    parser.add_argument('--solver', type=str, default="Prioritised",
                        help='Use batch output instead of animation')
    args = parser.parse_args()
    args = args.instance.strip("instances")
    new_args = args.split('/')
    file = open("experiment2Results/"+solver+"/"+new_args[0]+"/"+new_args[1]+"/"+new_args[2]+"/"+replanner.lower()+"Result.txt", "a")
    file.write(new_args[3] + " initial: " + str(initial) + "; new: "+ str(new))
    file.close()

def writeResultsForExperiment1(cost,planner ):
    parser = argparse.ArgumentParser(description='Runs various MAPF algorithms')
    parser.add_argument('--instance', type=str, default=None,
                        help='The name of the instance file(s)')
    parser.add_argument('--batch', action='store_true', default=False,
                        help='Use batch output instead of animation')
    parser.add_argument('--replanner', type=str, default="Prioritised",
                        help='Use batch output instead of animation')
    parser.add_argument('--solver', type=str, default="Prioritised",
                        help='Use batch output instead of animation')
    args, unknown = parser.parse_known_args()
    args = args.instance.strip("instances")
    new_args = args.split('/')
    print("experiment1Results"+new_args[0]+"/"+new_args[1]+"/"+new_args[2]+"/"+new_args[3]+"/"+planner+"_result.txt")
    file = open("experiment1Results"+new_args[0]+"/"+new_args[1]+"/"+new_args[2]+"/"+new_args[3]+"/"+planner+"_result.txt", "a")
    file.write(new_args[4] + " cost: " + str(cost) + "\n")
    file.close()

