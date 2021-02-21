# -*- coding: utf-8 -*-
"""
Created on Tue Mar 24 14:24:19 2020

@author: alex, james, john

# TO RUN: python joint_mdd.py --instance custominstances/paper.txt
"""
import argparse
import glob
import itertools
import copy
import mdd
from single_agent_planner import move, compute_heuristics, is_constrained, build_constraint_table
import run_experiments
import cbs


#Build MDD for single agent
class Joint_MDD(object):
    def __init__(self, MDD1, MDD2):
        """
        Input: 2 MDDs
        Output: Creates a joint MDD and can output whether the two mdds are dependent (True or False)
        
        Assumes: We don't need a tree structure (parents/children) because we can check if a set of 
        MDDs are dependent if a layer inside the joint MDD is empty.        
        """
        self.MDD_list = [copy.deepcopy(MDD1), copy.deepcopy(MDD2)] 
        self.max_level = max([len(i.levels) for i in self.MDD_list])
        self.num_of_agents = 2
        self.levels = []
        
        # Equalize the size of the two MDDs
        MDD1_level = len(MDD1.levels)
        MDD2_level = len(MDD2.levels)
        if MDD1_level > MDD2_level:
            level_diff = MDD1_level - MDD2_level
            for i in range(level_diff):
                self.MDD_list[1].extend_MDD()
        elif MDD2_level > MDD1_level:
            level_diff = MDD2_level - MDD1_level
            for i in range(level_diff):
                self.MDD_list[0].extend_MDD() 
        
        # Creates all combinations of the two MDDs
        for i in range(self.max_level):
            level_list = []
            for j in range(self.num_of_agents):
                level_list.append(self.MDD_list[j].getLocsInLevel(i))
            full = list(itertools.product(*level_list))    
            self.levels.append(full)
        
        # Removes vertex contraint (i.e. ((1,1),(1,1)))
        for level in self.levels:
            for pos in level:
                if len(pos) > len(set(pos)):
                    level.remove(pos)
        
        self.remove_parentless()
        
        # Remove edge conflicts (i.e. ((1,2),(1,3)) with parent ((1,3),(1,2)))
        for i, level in enumerate(self.levels):
            to_remove = []
            for pos in level:
                if i == 0:
                    continue
                parents = self.find_parents(pos)
                intersection = set(parents).intersection(set(self.levels[i-1]))
                if len(intersection) == 1 :  ## Only remove if the only parent is the edge collision
                    parent_pos = intersection.pop()
                    if self.is_edge_collision(pos, parent_pos):
                        to_remove.append(pos)
            for rem in to_remove:
                self.levels[i].remove(rem)
        
        self.remove_parentless()
        
    def remove_parentless(self):
        for i in range(len(self.levels)-1):
            to_remove = []
            for pos in self.levels[i+1]:
                parents = self.find_parents(pos)
                intersection = set(parents).intersection(set(self.levels[i]))
                if len(intersection) == 0:
                    to_remove.append(pos)
            for rem in to_remove:
                self.levels[i+1].remove(rem)
    
    def find_parents(self, child):
        parents1 = []
        parents2 = []
        for dir in range(5):
            parents1.append(move(child[0], dir))
            parents2.append(move(child[1], dir))
        
        parents = list(itertools.product(parents1, parents2))
        return parents
    
    def is_edge_collision(self, pos1, pos2):
        if pos1[0] == pos2[1] and pos1[1] == pos2[0]:
            return True
        else:
            return False
    
    def is_dependent(self):
        for level in self.levels:
            if len(level) == 0:
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
                joint_MDD = Joint_MDD(MDD_list[i], MDD_list[j])
                if joint_MDD.is_dependent():
                    graph[i].append(1)
                else:
                    graph[i].append(0)
                    
    #adjacency matrix is symmetrical so copy half of the entries
    for k in range(len(MDD_list)):
        for l in range(len(MDD_list)):
            if k < l:
                graph[l][k] = graph[k][l]
    return graph

def weighted_graph_from_MDD(MDD_list, my_map, starts, goals):
    graph = []
    
    for i in range(len(MDD_list)):
        graph.append([])

        for j in range(len(MDD_list)):
            if i > j:
                graph[i].append(0)
            elif i == j:
                graph[i].append(0)
            else:
                joint_MDD = Joint_MDD(MDD_list[i], MDD_list[j])
                if joint_MDD.is_dependent():
                    CBS_instance = cbs.CBSSolver(my_map, [starts[i], starts[j]], [goals[i], goals[j]], None)
                    CBS_instance.find_solution()
                    graph[i].append(CBS_instance.sum_of_costs + 2 - len(MDD_list[i].levels) - len(MDD_list[j].levels))
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
        
        MDD_lists = []
        for i in range(len(starts)):
            h_values = compute_heuristics(my_map, goals[i])
            temp_MDD = mdd.MDD(my_map, starts[i], goals[i], h_values, i, [])
            MDD_lists.append(temp_MDD)
            
        #joint_MDD = Joint_MDD(MDD_lists[0], MDD_lists[1])
        #print(joint_MDD.is_dependent)
        
        print(weighted_graph_from_MDD(MDD_lists, my_map, starts, goals))
        #print(graph_from_MDD(MDD_lists))
        
