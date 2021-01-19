#!/usr/bin/python
import argparse
import glob
from pathlib import Path
import cbs
from independent import IndependentSolver
from prioritized import PrioritizedPlanningSolver
from visualize import Animation
from single_agent_planner import get_sum_of_cost
import time
import matplotlib.pyplot as plt
import numpy as np

SOLVER = "CBS"

def print_mapf_instance(my_map, starts, goals):
    print('Start locations')
    print_locations(my_map, starts)
    print('Goal locations')
    print_locations(my_map, goals)


def print_locations(my_map, locations):
    starts_map = [[-1 for _ in range(len(my_map[0]))] for _ in range(len(my_map))]
    for i in range(len(locations)):
        starts_map[locations[i][0]][locations[i][1]] = i
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
            if cell == '@':
                my_map[-1].append(True)
            elif cell == '.':
                my_map[-1].append(False)
    # #agents
    line = f.readline()
    num_agents = int(line)
    # #agents lines with the start/goal positions
    starts = []
    goals = []
    for a in range(num_agents):
        line = f.readline()
        sx, sy, gx, gy = [int(x) for x in line.split(' ')]
        starts.append((sx, sy))
        goals.append((gx, gy))
    f.close()
    return my_map, starts, goals


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs various MAPF algorithms')
    parser.add_argument('--instance', type=str, default=None,
                        help='The name of the instance file(s)')
    parser.add_argument('--batch', action='store_true', default=False,
                        help='Use batch output instead of animation')
    parser.add_argument('--disjoint', action='store_true', default=False,
                        help='Use the disjoint splitting')
    parser.add_argument('--solver', type=str, default=SOLVER,
                        help='The solver to use (one of: {CBS,Independent,Prioritized}), defaults to ' + str(SOLVER))
    parser.add_argument('--heuristic', type=str, default=None,
                        help='The solver to use (one of: {CG,DG,WDG}), defaults to None')

    args = parser.parse_args()

    result_file = open("results.csv", "w", buffering=1)

    errorbarMean = []
    errorbarStd = []
    expanded_list = []
    generated_list = []
    for file in sorted(glob.glob(args.instance)):
                
        print("***Import an instance***")
        my_map, starts, goals = import_mapf_instance(file)
        print_mapf_instance(my_map, starts, goals)

        if args.solver == "CBS":
            print("***Run CBS***")
            #run batch for 5 times each
            if args.batch:
                runs = 5
                times = []
                h_mean = []
                h_std = []
                h_expanded_list = []
                h_generated_list = []
                heuristics = [None, 'CG', 'DG', 'WDG']
                
                for h in heuristics:
                    for i in range(runs):
                        st = time.time()
                        
                        #CBS algorithm
                        solver = cbs.CBSSolver(my_map, starts, goals, h)
                        paths = solver.find_solution(args.disjoint)
                        
                        
                        end = time.time()
                        times.append(end - st)
                    expanded = solver.num_of_expanded
                    generated = solver.num_of_generated
                    h_expanded_list.append(expanded)
                    h_generated_list.append(generated)
                    mean = np.mean(times)
                    std = np.std(times)
                    h_mean.append(mean)
                    h_std.append(std)
                    times = []
                expanded_list.append(h_expanded_list)
                generated_list.append(h_generated_list)
                errorbarMean.append(h_mean)
                errorbarStd.append(h_std)

            else:
                solver = cbs.CBSSolver(my_map, starts, goals, args.heuristic)
                paths = solver.find_solution(args.disjoint)
        elif args.solver == "Independent":
            print("***Run Independent***")
            solver = IndependentSolver(my_map, starts, goals)
            paths = solver.find_solution()
        elif args.solver == "Prioritized":
            print("***Run Prioritized***")
            solver = PrioritizedPlanningSolver(my_map, starts, goals)
            paths = solver.find_solution()
        else:
            raise RuntimeError("Unknown solver!")

        #errorbarMean is [[], [], []]
        #need to normalize from None, CG, DG, WDG
        #each errorbarMean is [i][0] is None, [i][1] is CG, [i][2] is DG

        cost = get_sum_of_cost(paths)
        result_file.write("{},{}\n".format(file, cost))


    if not args.batch:
        print("***Test paths on a simulation***")
        animation = Animation(my_map, starts, goals, paths)
        # animation.save("output.mp4", 1.0)
        animation.show()
    if args.batch:
        #mean for each heuristic
        M_nonePts = []
        M_cgPts = []
        M_dgPts = []
        M_wdgPts = []
        #std for each heuristic
        S_nonePts = []
        S_cgPts = []
        S_dgPts = []
        S_wdgPts = []
        x = np.arange(len(errorbarMean))
        print(len(errorbarMean))
        #separate into each heuristic
        for i in range(len(errorbarMean)):
            
            M_nonePts.append(errorbarMean[i][0])
            M_cgPts.append(errorbarMean[i][1])
            M_dgPts.append(errorbarMean[i][2])
            M_wdgPts.append(errorbarMean[i][3])
            S_nonePts.append(errorbarStd[i][0])
            S_cgPts.append(errorbarStd[i][1])
            S_dgPts.append(errorbarStd[i][2])
            S_wdgPts.append(errorbarStd[i][3])
            
        #plot run time
        plt.figure(1)
        plt.rcParams.update({'font.size': 22})
        plt.title("Run Time")
        plt.errorbar(x, M_nonePts, yerr=S_nonePts, label='None', color='red', marker='o', markersize=12)
        plt.errorbar(x, M_cgPts, yerr=S_cgPts,  label='CG', color='blue', marker='o', markersize=12)
        plt.errorbar(x, M_dgPts, yerr=S_dgPts, label='DG', color='green', marker='o', markersize=12)
        plt.errorbar(x, M_wdgPts, yerr=S_wdgPts,  label='WDG', color='orange', marker='o', markersize=12)
        plt.minorticks_on()
        plt.xlabel('Test instance number')
        plt.ylabel('Run time')
        plt.legend()
        plt.show()
        
        #node expansion
        e_none = []
        e_cg = []
        e_dg = []
        e_wdg = []        
        #node generation
        g_none = []
        g_cg = []
        g_dg = []
        g_wdg = []
        x = np.arange(len(expanded_list))
        for i in range(len(expanded_list)):
            e_none.append(expanded_list[i][0])
            e_cg.append(expanded_list[i][1])
            e_dg.append(expanded_list[i][2])
            e_wdg.append(expanded_list[i][3])
            g_none.append(generated_list[i][0])
            g_cg.append(generated_list[i][1])
            g_dg.append(generated_list[i][2])
            g_wdg.append(generated_list[i][3])
        
        #plot node expansion
        plt.figure(2)
        plt.title("Node Expansion")
        plt.plot(x, e_none, label='None', color='red', marker='o', markersize=12)
        plt.plot(x, e_cg, label='CG', color='blue', marker='o', markersize=12)
        plt.plot(x, e_dg, label='DG', color='green', marker='o', markersize=12)
        plt.plot(x, e_wdg, label='WDG', color='orange', marker='o', markersize=12)
        plt.minorticks_on()
        plt.xlabel('Test Instance Number')
        plt.ylabel('Number of Nodes Expanded')
        plt.legend()
        plt.show()
        
        #plot node generation
        plt.figure(3)
        plt.title("Node Generation")
        plt.plot(x, g_none, label='None', color='red', marker='o', markersize=12)
        plt.plot(x, g_cg, label='CG', color='blue', marker='o', markersize=12)
        plt.plot(x, g_dg, label='DG', color='green', marker='o', markersize=12)
        plt.plot(x, g_wdg, label='WDG', color='orange', marker='o', markersize=12)
        plt.minorticks_on()
        plt.xlabel('Test Instance Number')
        plt.ylabel('Number of Nodes Generated')
        plt.legend()
        plt.show()
        
    result_file.close()
