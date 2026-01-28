from calendar import c
from searchalgorithms.base import SearchAlgorithmBase



class bfs(SearchAlgorithmBase):
    def __init__(self) -> None:
        super().__init__() 
        
    def reset(self, grid, start, goal): 
        super().reset(grid, start, goal)
        # If you want to initialize other stuff, put it here
        self._parent_map = {}
    # Adding BFS specific step implementation
    def step(self):
        """
        Performs one step of the Breadth-First Search (BFS) algorithm.
        - Use First-In-First-Out (FIFO) queue for the frontier.
        - Goal test
        - Expand nodes level by level.        
        """
        
        if self._done:
            return  # If already done, do nothing
        
        # Check if the frontier is empty. If it is, mark the search as done (failure).
        if not self._frontier:
            self._done  = True
            self._path = []  # No path found
            return

        # Continue with the BFS algorithm
        # Frontier tuple: (node, heuristic_h, cost_g, parent)
        current_node, _, g, parent = self._frontier.pop(0)
        
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
         
        # Explore neighbors
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:  # Up, Down, Left, Right: (delta_row, delta_column)
            nr, nc = r + dr, c + dc
            # Check if neighbor is within bounds
            if 0 <= nr < row_num and 0 <= nc < col_num and self._grid[nr,nc] != 1: # 1 represents occupied cell, probably wall
                neighbor = (nr, nc)
                # Only add if neighbor is not already explored or not in frontier
                if neighbor not in self._explored and neighbor not in [n[0] for n in self._frontier]:
                    # Add neighbor to frontier
                    self._frontier.append((neighbor, _, g + 1, current_node))
                    self._parent_map[neighbor] = current_node
                    
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