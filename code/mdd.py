# -*- coding: utf-8 -*-
"""
Created on Tue Mar 24 14:24:19 2020

@author: alex, james, john
"""
import argparse
import glob
import itertools
from single_agent_planner import move, compute_heuristics, is_constrained, build_constraint_table
import run_experiments


#Multi valued decision node
#N is node from constraint
#parent is previous MDDNode
class MDDNode(object):
    def __init__(self, loc, g_val, h_val, parent):
        if parent == None:
            self.parent = None
            self.timestep = 0
        else:
            self.parent = parent
            self.timestep = parent.timestep + 1
            
        self.children = []
        self.loc = loc
        self.g_val = g_val
        self.h_val = h_val
        
    def equals(self, node):
        return self.loc == node.loc and self.timestep == node.timestep

#Build MDD for single agent
class MDD(object):
    def __init__(self, my_map, start_loc, goal_loc, h_values, agent, constraints):
        self.root = MDDNode(start_loc, 0, h_values[start_loc], None)
        self.levels = []
        self.levels.append([])
        constraint_table = build_constraint_table(constraints, agent)
        h_value = h_values[start_loc]
        open_list = []
        closed_list = {}
        closed_list[(start_loc, 0)] = self.root
        open_list.append(self.root)
        
        while len(open_list) > 0:
            curr = open_list.pop(0)
            #print("expanding node: {}".format(curr.loc))
           
            if curr.timestep >= len(self.levels):
                self.levels.append([])
            self.levels[curr.timestep].append(curr)
            
            if curr.loc == goal_loc:
                #print("reached goal node: {}".format(curr.loc))
                return
                
            # Time limit check of the search
            agent_path_length = {}
            for a in range(agent+1):
                agent_path_length[a] = 0
            for constraint in constraints:
                if constraint['agent'] == agent: 
                    continue
                if constraint['agent'] not in agent_path_length.keys() or constraint['timestep'] > agent_path_length[constraint['agent']]:
                    agent_path_length[constraint['agent']] = constraint['timestep']
            time_upperbound = sum(agent_path_length.values()) + len(my_map)+len(my_map[0]) + 1
            if curr.timestep > time_upperbound - 1: ### change constant (10) to a more general value.
                return None

            move_check = False # Use to check if curr node can move (not stay in place)
            for dir in range(4):
                child_loc = move(curr.loc, dir)

                #check for various illegal moves
                if child_loc[0] < 0 or child_loc[1] < 0:
                    continue
                if child_loc[0] > len(my_map)-1 or child_loc[1] > len(my_map[0])-1:
                    continue
                if my_map[child_loc[0]][child_loc[1]]:
                    continue
                if is_constrained(curr.loc, child_loc, curr.timestep+1, constraint_table):
                    continue
                
                move_check = True
                if h_values[child_loc] < curr.h_val:
                    possible_move_check = True
                    if (child_loc, curr.timestep + 1) in closed_list:
                        child = closed_list[(child.loc, child.timestep)]
                        #print("generating existing node: {}".format(child.loc))
                    else:
                        child = MDDNode(child_loc, curr.g_val + 1, h_values[child_loc], curr)
                        open_list.append(child)
                        #print("generating new node: {}".format(child.loc))
                    #for i in open_list: print("open_list: {}".format(i.loc))
                    closed_list[(child.loc, child.timestep)] = child
                    curr.children.append(child)
                
            # if no possible moves add a staying in place move
            if move_check == False:   
                if is_constrained(curr.loc, child_loc, curr.timestep+1, constraint_table):
                    continue
                child_loc = move(curr.loc, 4)
                if (child_loc, curr.timestep + 1) in closed_list:
                    child = closed_list[(child.loc, child.timestep)]
                else:
                    child = MDDNode(child_loc, curr.g_val + 1, h_values[child_loc], curr)
                    open_list.append(child)
                closed_list[(child.loc, child.timestep)] = child    
                curr.children.append(child)
            
    def print_all(self):
        open_list = []
        closed_list = []
        open_list.append(self.root)

        while open_list:
            print_node = open_list.pop(0)
            if print_node not in closed_list:
                closed_list.append(print_node)
                print("\nnode (timestamp, loc): {}".format((print_node.timestep, print_node.loc)))
                for i in print_node.children:
                    print("----child loc: {}".format(i.loc))
                    open_list.append(i)

    def print_levels(self):
        for i in range(len(self.levels)):
                print("\nlevel {}: ".format(i))
                for j in range(len(self.levels[i])):
                    print("node loc: {}".format(self.levels[i][j].loc))
                    
    def getNodesInLevel(self, level):
        node_list = []
        for i in range(len(self.levels[level])):
            node_list.append(self.levels[level][i])
        return node_list
                    
    def getLocsInLevel(self, level):
        node_list = []
        for i in range(len(self.levels[level])):
            node_list.append(self.levels[level][i].loc)
        return node_list
        
    def extend_MDD(self):
        # Adds another level that is a copy of the final level.
        # Used in joint_MDD
        curr = self.levels[len(self.levels)-1][0]
        child = MDDNode(curr.loc, curr.g_val, curr.h_val, curr)
        self.levels.append([child])
        

def is_conflicting(left, right):
    deepest = left if max(len(left.levels), len(right.levels)) == len(left.levels) else right

    for i in range(len(deepest.levels)):

        if len(deepest.levels[i]) == 1:
            loc_left = left.levels[i][0].loc if i < len(left.levels) else left.getLocsInLevel(len(left.levels) - 1)[0]
            loc_right = right.levels[i][0].loc if i < len(right.levels) else right.getLocsInLevel(len(right.levels) - 1)[0]

            #vertex collision
            if loc_left == loc_right:
                return True
            
            #edge collision is only possible before the end of either MDD as after the goal node there's no more moves
            if i < len(left.levels):
                edge_left = [loc_left, left.getNodesInLevel(i - 1)[0].loc]
            else:
                edge_left = [loc_left, loc_left]
            if i < len(right.levels):
                edge_right = [loc_right, right.getNodesInLevel(i - 1)[0].loc]
            else:
                edge_right = [loc_right, loc_right]

            #edge collision
            if edge_left == edge_right[::-1]:
                return True
            
    return False


def graph_from_MDD(MDD_list):
    graph = []
    
    for i in range(len(MDD_list)):
        graph.append([])

        for j in range(len(MDD_list)):
            if i > j:
                graph[i].append(0)
            elif i == j:
                graph[i].append(0)
            else:
                if is_conflicting(MDD_list[i], MDD_list[j]):
                    #print("conflicting agents: {} & {}".format(i,j))
                    graph[i].append(1)
                else:
                    graph[i].append(0)

    #adjacency matrix is symmetrical so copy half of the entries
    for k in range(len(MDD_list)):
        for l in range(len(MDD_list)):
            if k < l:
                graph[l][k] = graph[k][l]
    return graph


### testing code
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generate MDD for agents')
    parser.add_argument('--instance', type=str, default=None,
                        help='The name of the instance file(s)')

    args = parser.parse_args()

    for file in sorted(glob.glob(args.instance)):

        print("***Import an instance***")
        my_map, starts, goals = run_experiments.import_mapf_instance(file)
        run_experiments.print_mapf_instance(my_map, starts, goals)
        MDD_list = []

        for i in range(len(starts)):
            h_values = compute_heuristics(my_map, goals[i])
            temp_MDD = MDD(my_map, starts[i], goals[i], h_values, i, [])
            print("\n=====\nagent {}:".format(i))
            temp_MDD.print_all()
            #temp_MDD.print_levels()
            MDD_list.append(temp_MDD)

        conflict_graph = graph_from_MDD(MDD_list)
        print("conflict graph: {}".format(conflict_graph))
