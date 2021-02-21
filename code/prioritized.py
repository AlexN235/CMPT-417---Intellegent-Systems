import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []
        constraints = []

        for i in range(self.num_of_agents):  # Find path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)             
            if path is None:
                raise BaseException('No solutions')
            result.append(path)

            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches
            for node_index in range(len(path)):
                for agent_index in range(self.num_of_agents):
                    if agent_index != i:
                        # Adding Vertex Constraints
                        vertex_constraint = {'agent': agent_index, 'loc':[path[node_index]], 'timestep': node_index}
                        constraints.append(vertex_constraint)
                        
                        # Adding Edge Constraints
                        for dir in range(4):
                            curr_pos = move(path[node_index], dir)
                            if self.my_map[curr_pos[0]][curr_pos[1]]:
                                continue
                            edge_constraint = {'agent': agent_index, 'loc':[curr_pos, path[node_index]], 'timestep':node_index+1}
                            constraints.append(edge_constraint)
            
            # Calculate the edge_constraints for goal state nodes            
            path_upperbound = 0
            for p in result:
                path_upperbound += len(p)
            path_upperbound += len(self.my_map)+len(self.my_map[0]) ### Look for something more specific to find upper bound
            for index, route in enumerate(result):
                for i in range(path_upperbound-len(route)):
                    goal_state = route[len(route)-1]
                    time = len(route) + i - 1
                    # create constraints for every agent
                    for a in range(self.num_of_agents):
                        if a == index: continue
                        for dir in range(4):
                            curr_pos = move(goal_state, dir)
                            if self.my_map[curr_pos[0]][curr_pos[1]]:
                                continue
                            edge_constraint = {'agent': a, 'loc':[curr_pos, goal_state], 'timestep':time+1}
                            constraints.append(edge_constraint)
            ##############################

        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result
        
        
def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]
