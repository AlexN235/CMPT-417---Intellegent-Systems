import time as timer
import heapq
import random
import copy
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost
import joint_mdd
import mdd
import mvc


def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.
    
    # Vertex Collision
    if len(path1) > len(path2):
        goal_state = path2[len(path2)-1]
        diff = len(path1) - len(path2)
        path2 = path2 + [goal_state]*diff
    else:
        goal_state = path1[len(path1)-1]
        diff = len(path2) - len(path1)
        path1 = path1 + [goal_state]*diff
    
    for t in range(len(path1)+1):
        if get_location(path1, t) == get_location(path2, t):
            vertex_collision = ([get_location(path1, t)], t)
            return vertex_collision
    
    # Edge Collision
    for t in range(len(path1)):
        if get_location(path1, t) == get_location(path2, t+1) and get_location(path1, t+1) == get_location(path2, t):    
            edge_collision = ([get_location(path1, t), get_location(path1, t+1)], t+1)
            return edge_collision
    return None


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    collision_list = []
    num_of_agents = len(paths)
    for outer_agent in range(num_of_agents):
        for inner_agent in range(outer_agent, num_of_agents):
            if outer_agent == inner_agent:
                continue
            collision_loc = detect_collision(paths[outer_agent], paths[inner_agent])
            if collision_loc != None:
                collision = {'a1':outer_agent, 'a2':inner_agent, 'loc':collision_loc[0], 'timestep':collision_loc[1]}
                collision_list.append(collision)
    return collision_list


def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep
    
    # Vertex collision
    if len(collision['loc']) == 1:
        constraint1 = {'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep']}
        constraint2 = {'agent': collision['a2'], 'loc': collision['loc'], 'timestep': collision['timestep']}
    # Edge collision
    else: 
        loc1 = collision['loc'][0]
        loc2 = collision['loc'][1]
        constraint1 = {'agent': collision['a1'], 'loc': [loc1, loc2], 'timestep': collision['timestep']}
        constraint2 = {'agent': collision['a2'], 'loc': [loc2, loc1], 'timestep': collision['timestep']}
    return [constraint1, constraint2]


def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly
    
    random_agent = random.randint(0,1)
    # Vertex Collision
    random_agent = 0
    if len(collision['loc']) == 1:
        if random_agent == 0:
            constraint1 = {'agent':collision['a1'], 'loc':collision['loc'], 'timestep':collision['timestep'], 'positive':True}
            constraint2 = {'agent':collision['a1'], 'loc':collision['loc'], 'timestep':collision['timestep'], 'positive':False}
        else:
            constraint1 = {'agent':collision['a2'], 'loc':collision['loc'], 'timestep':collision['timestep'], 'positive':True}
            constraint2 = {'agent':collision['a2'], 'loc':collision['loc'], 'timestep':collision['timestep'], 'positive':False}
    # Edge Collision
    else:
        loc1 = collision['loc'][0]
        loc2 = collision['loc'][1]
        if random_agent == 0:
            constraint1 = {'agent':collision['a1'], 'loc':[loc1, loc2], 'timestep':collision['timestep'], 'positive': True}
            constraint2 = {'agent':collision['a1'], 'loc':[loc1, loc2], 'timestep':collision['timestep'], 'positive': False}
        else:
            constraint1 = {'agent':collision['a2'], 'loc':[loc2, loc1], 'timestep':collision['timestep'], 'positive': True}
            constraint2 = {'agent':collision['a2'], 'loc':[loc2, loc1], 'timestep':collision['timestep'], 'positive': False}
    return [constraint1, constraint2]

#
# Please insert this function into "cbs.py" before "class CBSSolver"
# is defined.
#

def paths_violate_constraint(constraint, paths):
    assert constraint['positive'] is True
    rst = []
    for i in range(len(paths)):
        if i == constraint['agent']:
            continue
        curr = get_location(paths[i], constraint['timestep'])
        prev = get_location(paths[i], constraint['timestep'] - 1)
        if len(constraint['loc']) == 1:  # vertex constraint
            if constraint['loc'][0] == curr:
                rst.append(i)
        else:  # edge constraint
            if constraint['loc'][0] == prev or constraint['loc'][1] == curr \
                    or constraint['loc'] == [curr, prev]:
                rst.append(i)
    return rst

class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals, CT_heuristic):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """
        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)
        self.CT_heuristic = CT_heuristic

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        self.sum_of_cost = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))
        print(self.heuristics)

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'] + node['h_val'], len(node['collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True, heuristic=None):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """
        
        self.start_time = timer.time()
        
        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)
            
        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        root['h_val'] = 0

        self.push_node(root)
        # Task 3.1: Testing
        #print(root['collisions'])

        # Task 3.2: Testing
        #for collision in root['collisions']:
        #    print(standard_splitting(collision))

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit

        while len(self.open_list) > 0:
            p = self.pop_node()
            if len(p['collisions']) == 0:
                self.sum_of_costs = get_sum_of_cost(p['paths']) 
                return p['paths']
        
            collision = p['collisions'][0]
            constraints = standard_splitting(collision) #disjoint_splitting(collision)
            for constraint in constraints:                 
                q = {'cost': 0,
                    'constraints': copy.deepcopy(p['constraints'] + [constraint]),
                    'paths': copy.deepcopy(p['paths']),
                    'collisions': []}
                """ Task 4  
                if constraint['positive'] == True:
                    if paths_violate_constraint(constraint, q['paths']):
                        continue
                """
                
                agent = constraint['agent']
                path = a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent],
                          agent, q['constraints'])
                
                if path != None:
                    q['paths'][agent] = path
                    q['collisions'] = detect_collisions(q['paths'])
                    q['cost'] = get_sum_of_cost(q['paths'])

                    if self.CT_heuristic == None:
                        q['h_val'] = 0
                    else:
                        MDD_list = [mdd.MDD(self.my_map, self.starts[i], self.goals[i],\
                                compute_heuristics(self.my_map, self.goals[i]), i, q['constraints'])\
                                for i in range(len(self.starts))]
                        if self.CT_heuristic == "CG":
                            graph_inst = mdd.graph_from_MDD(MDD_list)
                            mvc_inst = mvc.BruteForceMVC(graph_inst)
                            q['h_val'] = mvc_inst.getMVC()
                        elif self.CT_heuristic == "DG":
                            graph_inst = joint_mdd.graph_from_MDD(MDD_list)
                            mvc_inst = mvc.BruteForceMVC(graph_inst)
                            q['h_val'] = mvc_inst.getMVC()
                        elif self.CT_heuristic == "WDG":
                            graph_inst = joint_mdd.weighted_graph_from_MDD(MDD_list, self.my_map, self.starts, self.goals)
                            ewmvc_inst = mvc.BruteForceEWMVC(graph_inst)
                            q['h_val'] = ewmvc_inst.getEWMVC()

                    self.push_node(q)
                        
        if path is None:
            raise BaseException('No solutions')
                
        self.print_results(root)
        return root['paths']

    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
