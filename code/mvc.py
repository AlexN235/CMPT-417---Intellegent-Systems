"""
CMPT 417 Group Project
"""
import itertools
import copy
import math
    
class BruteForceMVC():
    """
    Implementation of a brute force method of minimum vertex cover
    Check by making a list of all possible subset of the graph and check
    all possible subsets to find the minimum.
    """
    def __init__(self, graph):
        self.graph = graph
        self.numOfVertex = len(graph)
        self.cover = list(range(0, self.numOfVertex-1))
        self.minimumCover = len(graph) # initially we have the whole graph as cover, then we attempt to find the min cover.
    
    ### Regular MVC
    def getMVC(self):
        subset = self.getAllSubsets()
        for i in subset:
            if self.checkCover(i) and len(i) < self.minimumCover:
                self.cover = i
                self.minimumCover = len(i)
        return self.minimumCover

    def getAllSubsets(self):
        vertex = list(range(0, len(self.graph)))
        subset = []
        for i in range(self.numOfVertex+1):
            set = itertools.combinations(vertex, i)
            for s in set:
                subset.append(list(s))
        return subset
        
    def checkCover(self, cover):
        """
        Checks MVC by checking a subset from the nodes given in the cover and its neightbours.
        If the subset contain every vertex, then its a cover. (check length of list)
        """
        for i, outerVertex in enumerate(self.graph):
            if i in cover:
                continue
            for j, innerVertex in enumerate(outerVertex):
                if innerVertex == 1 and j not in cover:
                    return False
        #print("checkCover returns true", len(cover), cover)
        return True
        
class BruteForceEWMVC():
    """
    Implementation of a brute force method for edge weight minimum vertex cover
    Finds the EWMVC by doing a BnBDFS on all possible assignments on the vertices.
    
    Warming: Conflict graphs that exceed 5-6 vertices will take a very long time.
             Therefore spliting the graph into connected components help reduce 
             the run time.
    """

    def __init__(self, graph):
        self.graph = graph
        self.numOfVertex = len(graph)
    
    def getEWMVC(self):
        components = self.component_split()
        full_weight = 0
        for component in components:
            assignment = self.getComponentWeight(component)
            full_weight += sum(assignment)
        return full_weight
    
    def removeDisconnectedVertex(self, graph):
        """
        Remove any disconnected single vertex from the graph (to reduce run time)
        ## Unused in this project ##
        """
        indexToRemove = []
        new_graph = graph.copy()
        for index, vertex in enumerate(new_graph):
            if sum(vertex) == 0:
                indexToRemove.append(index)
                    
        for i in range(len(indexToRemove)):
            del new_graph[indexToRemove[len(indexToRemove)-1-i]]
            
        for vertex in new_graph:
            for i in range(len(indexToRemove)):
                del vertex[indexToRemove[len(indexToRemove)-1-i]]
                
        return new_graph
    
    def component_split(self):
        """
        Takes a graph and splits it into its connect components as well as 
        deleting all edgeless vertices.
        """
        # Step 1: Identify which vertices are together.
        graph = self.graph.copy()
        connect_graph = self.graph_to_connection(graph)
        while self.combine_check(connect_graph):
            connect_graph = self.combine_connected(connect_graph)
       
        # Step 2: Group the vertices from the graph into component groups (list)
        new_graph = []
        for component in connect_graph:
            component_graph = []
            for vertex in component:
                component_graph.append(self.graph[vertex])
            new_graph.append(component_graph)
            
        # Step 3: Delete unconnected vertices from each component
        for index, component in enumerate(new_graph): 
            toDelete = list(set(list(range(self.numOfVertex))) - set(connect_graph[index]))
            toDelete.sort()
            for vertex in component:
                for d in range(len(toDelete)):
                    del vertex[toDelete[len(toDelete)-1-d]]
        
        return new_graph
    
    def graph_to_connection(self, graph):
        """
        Turns the graph(list of vertex) into a list of vertices that contains it neighbour's
        index. 
        i.e. [[0,1,0,0,0], [1,0,0,0,0], [0,0,0,4,1], [0,0,4,0,4], [0,0,1,4,0]] 
              => [[0,1], [1,0], [2,3,4], [3,2,4], [4,2,3]]
        """
        connectGraph = []
        for i, vertex in enumerate(graph):
            connection = [i]
            for j, edge in enumerate(vertex):
                if edge > 0:
                    connection.append(j)
            connectGraph.append(connection)
        
        # Delete any vertices with no edges
        toDelete = []
        for i, connect in enumerate(connectGraph):
            if len(connect) < 2:
                toDelete.append(i)
        for index in range(len(toDelete)):
            del connectGraph[toDelete[len(toDelete)-1-index]]
        
        # Sorting is neccessary to keep vertex/indices in the correct possition.
        for component in connectGraph:
            component.sort()
        return connectGraph
            
            
    def combine_connected(self, graphConnection):
        """
        Combines any list that share the same values (vertex). Only one combination per 
        function call.
        i.e [[0,1], [1,0], [2,3,4], [3,2,4], [4,2,3]] => [[0,1], [2,3,4], [3,2,4], [4,2,3]]
        """
        newConnection = graphConnection.copy()
        for out_i, outer in enumerate(graphConnection):
            for in_i, inner in enumerate(graphConnection):
                if out_i >= in_i:
                    continue
                for connect in inner:
                    if connect in outer:
                        inOuterNotInner = set(outer) - set(inner)
                        combined = list(inner) + list(inOuterNotInner)
                        del newConnection[in_i]
                        del newConnection[out_i]
                        newConnection.append(combined)
                        return newConnection
    
    def combine_check(self, list):
        """
        Return True when there are still things to combine (there are overlapping lists remaining)
        """
        for out_i, outer in enumerate(list):
            for in_i, inner in enumerate(list):
                condition = len((set(outer)-set(inner)))
                if out_i == in_i:
                    continue
                elif len(outer) - condition == 0: ## Check if there are overlapping values in outer vs inner
                    # Overlap if not zero
                    continue
                else:
                    return True        
        return False
        
    def getComponentWeight(self, component):
        """
        Input: 
            component: A list of vectices for a connected component of a graph (also works for full graph
            representation) i.e [[0, 2, 3], [2, 0, 1], [3, 1, 0]] 
        
        Output:
            A set of assignments to the vertices. When these assignments are summed together
            they produce the weight of that component.
        """
        # Parameters for initial node in DFS
        index = 0
        assignments = [None] * len(component)
        best = math.inf  
        branchUpperBound = 0
        for i in component:
            branchUpperBound += sum(i)
        
        return self.DFSRecursive(component, index, assignments, best, branchUpperBound)
        
    def DFSRecursive(self, component, index, assignments, best, branchUpperBound):
        """
        Recursive part of getComponentWeight() function.
        Input:
            component: A edge weighted graph.
            index: Index for the vertex we are assigning a values.
            assignments: List of current assignments so far. None is used as a place holder for unassigned
            best: The lowest weight value found so far. If exceeded, then prune current branch.
            branchUpperBound: A limit to what values can be assigned to the vertex.
            
        Output: 
            Returns a set of assignments.
        """
        # Returns when all vertices have been assigned a value.
        if index >= len(component):
            return assignments
        bestAssign = None
        bestSoFar = best
        for i in range(branchUpperBound):
            newAssign = assignments.copy()
            newAssign[index] = i
            if self.check_assignment(component, newAssign, index) == False:
                continue
            currAssign = self.DFSRecursive(component, index+1, newAssign, bestSoFar, branchUpperBound)
            if currAssign != None and sum(currAssign) < bestSoFar:
                bestAssign = currAssign
                bestSoFar = sum(bestAssign)
        return bestAssign
        
    def check_assignment(self, component, assignments, index):
        """
        Used in BnBDFS to ensure that the current set of assignments are valid
        For every edge, E_ij <= Xi + Xj.
        """
        i = assignments[index]
        for j, weights in enumerate(component[index]):
            if assignments[j] == None:
                continue
            if i + assignments[j] < weights:
                return False
        return True
        
if __name__ == '__main__':
    """ Graph test
    [[0, 1, 0, 0, 0, 0, 0], [1, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 1, 1], [0, 0, 0, 0, 1, 0, 1], [0, 0, 0, 0, 1, 1, 0]]
    [[0, 1, 1], [1, 0, 1], [1, 1, 0]]
    """
    graph = [[0, 1, 0, 0, 0, 0, 0], [1, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 4, 1], [0, 0, 0, 0, 4, 0, 4], [0, 0, 0, 0, 1, 4, 0]]
    #graph = [[0, 1, 0, 0, 0], [1, 0, 0, 0, 0], [0, 0, 0, 4, 1], [0, 0, 4, 0, 4], [0, 0, 1, 4, 0]]
    print("--- main function running ---")
    print("start:" ,graph)
    mvc = BruteForceEWMVC(graph)
    print('end:', mvc.getEWMVC())
    
            
        
