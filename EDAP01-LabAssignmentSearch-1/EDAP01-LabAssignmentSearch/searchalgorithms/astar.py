from searchalgorithms.base import SearchAlgorithmBase 
import numpy as np


class astar(SearchAlgorithmBase):
    def __init__(self) -> None:
        super().__init__()
                        
    def reset(self, grid, start, goal): 
        super().reset(grid, start, goal)
        # If you want to initialize other stuff, put it here
        self._parent_map = {}
    
    def heuristic(self, node):
        # Euclidean distance heuristic from node n to goal
        #return np.sqrt((node[0] - self._goal[0])**2 + (node[1] - self._goal[1])**2)

        # Manhattan distance heuristic from node n to goal
        return abs(node[0] - self._goal[0]) + abs(node[1] - self._goal[1])

    def step(self):
        """
        Performs one step of the A* Search algorithm.
        - Use Priority Queue for the frontier based on f(n) = g(n) + h(n).
        """
        
        """
        Pseudocode for A*

        Function A*(problem):
            fringe = {Node(problem.initial_state)}
            while (true):
                if (fringe.empty) return fail
                node = min_f_value(fringe) #Get node with lowest f(n) value
                if (node.state is goal) return solution
                else fringe.add(Expand(node,problem))
        """


        if self._done:
            return  # If already done, do nothing
        
        # Check if the frontier is empty. If it is, mark the search as done (failure).
        if not self._frontier:
            self._done  = True
            self._path = []  # No path found
            return

        # COntinue with the UFS algorithm
        # Frontier tuple: (node, heuristic_h, cost_g, parent)
        index_of_min_f = np.argmin([item[1] + item[2] for item in self._frontier])
        current_node, h, g, parent = self._frontier.pop(index_of_min_f)

        
        # Mark current node as explored
        if current_node not in self._explored:
            self._explored.append(current_node)
            
        # Goal check
        if current_node == self._goal:
            self._done = True
            self._cost = g
            self._path = self._reconstruct_path(current_node,current_node)
            return   
        
        # Get grid dimensions
        # Get row and column numbers of map, and of current node 
        row_num, col_num = self._grid.shape
        r,c = current_node

        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]: # Up, Down, Left, Right: (delta_row, delta_column) 
            neighbor= (r + dr, c + dc)
            h_neighbor = self.heuristic(neighbor)
            # Check if neighbor is within bounds
            if 0 <= neighbor[0] < row_num and 0 <= neighbor[1] < col_num and self._grid[neighbor[0],neighbor[1]] != 1: # 1 represents occupied cell, probably wall
                # Only add if neighbor is not already explored or not in frontier
                if neighbor not in self._explored and neighbor not in [n[0] for n in self._frontier]:
                    # Add neighbor to frontier
                    self._frontier.append((neighbor, h_neighbor, g + 1, current_node))
                    self._parent_map[neighbor] = current_node
                    #print("Current parent map: ", self._parent_map)
                    #print("Current frontier: ", self._frontier)

                    # Depth tracking
                    self._depth_map[neighbor] = self._depth_map[current_node] + 1
                    self._max_depth = max(self._max_depth, self._depth_map[neighbor])
        
                
        
        # Update max frontier size
        self._max_frontier_size = max(self._max_frontier_size, len(self._frontier))
        self._max_nodes_in_memory = max(self._max_nodes_in_memory, len(self._frontier) + len(self._explored))
    
    def _reconstruct_path(self, goal, parent):
        """Reconstructs the path from parent to goal/current node using parent pointers."""
        path = [goal]
        current = parent
        while current != self._start:
            current = self._parent_map[current]
            path.append(current)
            # Find parent of current in frontier or explore
            #for (n, _, _, p) in self._frontier + [(e, 0, 0, e) for e in self._explored]:
            #    if n == current:
            #        current = p
            #        break
        path.append(self._start)
        path.reverse()
        self._cost = len(path) - 1  # Cost is the length of the path minus one (number of moves)
        return path
         