# A* Path Planner for Rover Entry Navigation with Safety Integration
# --------------------------------------------------
# This module implements an occupancy-grid A* planner that finds a collision-free path
# from the rover's current position to the farm entry point, avoiding both the farm boundary
# and any configured no-go zones in the SafetyModule.

import heapq
import math
import numpy as np

class AStarPlanner:
    def __init__(self, rover, safety, cell_size=0.2, padding=2.5):
        """
        rover: instance of Rover with .x, .y, .entry_point, .geofence (list of (x,y) vertices)
        safety: instance of SafetyModule containing no-go zones & geofence
        cell_size: resolution of grid in meters (smaller for better accuracy)
        padding: extra margin around bounding box
        """
        self.rover = rover
        self.safety = safety
        self.cell = cell_size
        self.margin = padding
        self._build_grid()

    def _build_grid(self):
        """Build the occupancy grid for path planning"""
        # Determine bounding box combining rover pos, entry point, geofence, and no-go zones
        xs = [self.rover.x, self.rover.entry_point[0]] + [v[0] for v in self.safety.geofence]
        ys = [self.rover.y, self.rover.entry_point[1]] + [v[1] for v in self.safety.geofence]
        
        # Include no-go zone bounds if they exist
        if hasattr(self.safety, 'no_go_zones') and self.safety.no_go_zones:
            for (x0, y0, x1, y1) in self.safety.no_go_zones:
                xs += [x0, x1]
                ys += [y0, y1]

        self.x_min = min(xs) - self.margin
        self.x_max = max(xs) + self.margin
        self.y_min = min(ys) - self.margin
        self.y_max = max(ys) + self.margin

        # Compute grid dimensions
        self.nx = int(math.ceil((self.x_max - self.x_min) / self.cell))
        self.ny = int(math.ceil((self.y_max - self.y_min) / self.cell))

        # Initialize occupancy grid: 0=free, 1=obstacle
        self.grid = np.zeros((self.ny, self.nx), dtype=np.uint8)

        # Edge buffer thickness (in cells) for the farm boundary
        edge_buffer = max(1, int(0.3 / self.cell))

        # Mark farm boundary and obstacles in grid
        for i in range(self.ny):
            for j in range(self.nx):
                x = self.x_min + (j + 0.5) * self.cell
                y = self.y_min + (i + 0.5) * self.cell
                
                # Mark farm boundary with buffer zone
                if self._is_near_fence_edge(x, y, self.safety.geofence, edge_buffer * self.cell):
                    self.grid[i, j] = 1
                
                # Mark no-go zones
                if hasattr(self.safety, 'no_go_zones') and self.safety.no_go_zones:
                    for (x0, y0, x1, y1) in self.safety.no_go_zones:
                        if x0 <= x <= x1 and y0 <= y <= y1:
                            self.grid[i, j] = 1
        
        # Make entry point traversable along with a small area around it
        entry_cell = self._to_cell(*self.rover.entry_point)
        if 0 <= entry_cell[0] < self.ny and 0 <= entry_cell[1] < self.nx:
            # Clear entry point cell
            self.grid[entry_cell[0], entry_cell[1]] = 0
            
            # Clear cells in small radius around entry point for easier approach
            entry_radius = max(1, int(0.5 / self.cell))  # 0.5m radius
            for di in range(-entry_radius, entry_radius+1):
                for dj in range(-entry_radius, entry_radius+1):
                    ni, nj = entry_cell[0] + di, entry_cell[1] + dj
                    if 0 <= ni < self.ny and 0 <= nj < self.nx:
                        # Only clear if distance to entry is within radius
                        if di*di + dj*dj <= entry_radius*entry_radius:
                            self.grid[ni, nj] = 0

        # Ensure start cell is free (rover's current position)
        self.start = self._to_cell(self.rover.x, self.rover.y)
        if 0 <= self.start[0] < self.ny and 0 <= self.start[1] < self.nx:
            self.grid[self.start[0], self.start[1]] = 0
            
        # Ensure goal cell is free (entry point)
        self.goal = self._to_cell(*self.rover.entry_point)
        if 0 <= self.goal[0] < self.ny and 0 <= self.goal[1] < self.nx:
            self.grid[self.goal[0], self.goal[1]] = 0
            
        # Debug info
        print(f"Grid size: {self.ny}x{self.nx} cells, cell size: {self.cell}m")
        print(f"Start cell: {self.start}, Goal cell: {self.goal}")
        print(f"Entry point: {self.rover.entry_point}")
        print(f"Grid value at goal: {self.grid[self.goal] if 0 <= self.goal[0] < self.ny and 0 <= self.goal[1] < self.nx else 'out of bounds'}")

    def _is_near_fence_edge(self, x, y, vertices, threshold):
        """Check if point (x,y) is near any edge of the polygon defined by vertices"""
        n = len(vertices)
        for i in range(n):
            x1, y1 = vertices[i]
            x2, y2 = vertices[(i+1) % n]
            
            # Calculate distance from point to line segment
            px, py = x, y
            
            # Vector from point 1 to point 2
            dx = x2 - x1
            dy = y2 - y1
            line_length_sq = dx*dx + dy*dy
            
            # If segment is a point, calculate distance to that point
            if line_length_sq < 1e-10:
                dist = math.hypot(px - x1, py - y1)
            else:
                # Project point onto line
                t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / line_length_sq))
                
                # Calculate closest point on line segment
                closest_x = x1 + t * dx
                closest_y = y1 + t * dy
                
                # Distance from point to closest point on line
                dist = math.hypot(px - closest_x, py - closest_y)
            
            if dist < threshold:
                return True
        return False

    def _to_cell(self, x, y):
        """Convert world coordinates to grid cell indices"""
        j = int((x - self.x_min) / self.cell)
        i = int((y - self.y_min) / self.cell)
        # Ensure indices are within grid bounds
        i = max(0, min(i, self.ny-1))
        j = max(0, min(j, self.nx-1))
        return (i, j)

    def _to_xy(self, i, j):
        """Convert grid cell indices to world coordinates (cell center)"""
        x = self.x_min + (j + 0.5) * self.cell
        y = self.y_min + (i + 0.5) * self.cell
        return x, y

    def plan(self):
        """
        Executes A* search and returns a list of (x, y) waypoints from current position to entry point.
        Returns None if no path is found.
        """
        if self.start == self.goal:
            # Already at goal, return just the goal point
            return [self._to_xy(*self.goal)]
            
        # Heuristic function: Euclidean distance
        h = lambda a, b: math.hypot(a[0]-b[0], a[1]-b[1])
        
        # Initialize priority queue
        open_heap = []  # elements: (f_score, node_id, (i,j))
        node_id = 0     # To break ties in priority queue
        heapq.heappush(open_heap, (h(self.start, self.goal), node_id, self.start))
        node_id += 1
        
        # Initialize tracking dictionaries
        came_from = {}      # Parent pointers for path reconstruction
        g_score = {}        # Cost from start to node
        g_score[self.start] = 0
        closed = set()      # Set of fully expanded nodes
        
        # Max iterations to prevent infinite loops
        max_iterations = self.nx * self.ny * 2
        iterations = 0

        while open_heap and iterations < max_iterations:
            iterations += 1
            
            # Get node with lowest f_score
            f, _, current = heapq.heappop(open_heap)
            
            # Skip if already processed
            if current in closed:
                continue
                
            # Goal check
            if current == self.goal:
                path = self._reconstruct(came_from, current)
                return self._smooth_path(path)
                
            # Mark as processed
            closed.add(current)
            
            # Generate neighbors in 8 directions
            ci, cj = current
            for di, dj in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
                ni, nj = ci+di, cj+dj
                
                # Check bounds
                if not (0 <= ni < self.ny and 0 <= nj < self.nx):
                    continue
                    
                # Check if obstacle
                if self.grid[ni, nj] == 1:
                    continue
                    
                # Calculate cost to neighbor (diagonal moves cost more)
                move_cost = math.sqrt(di*di + dj*dj)
                tentative_g = g_score.get(current, float('inf')) + move_cost
                
                neighbor = (ni, nj)
                
                # Skip if we already found a better path to this neighbor
                if neighbor in closed and tentative_g >= g_score.get(neighbor, float('inf')):
                    continue
                
                # Update if this is a better path
                if tentative_g < g_score.get(neighbor, float('inf')):
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + h(neighbor, self.goal)
                    came_from[neighbor] = current
                    heapq.heappush(open_heap, (f_score, node_id, neighbor))
                    node_id += 1
        
        print(f"A* search terminated after {iterations} iterations without finding path")
        return None

    def _reconstruct(self, came_from, current):
        """Reconstruct path from goal to start using parent pointers"""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        # Convert to real-world coordinates
        return [self._to_xy(i, j) for (i, j) in path]
        
    def _smooth_path(self, path):
        """Apply path smoothing to reduce unnecessary waypoints"""
        if len(path) <= 2:
            return path
            
        # Keep start and end points
        result = [path[0]]
        
        # Look ahead to find direct paths
        i = 0
        while i < len(path) - 1:
            current = path[i]
            
            # Try to find furthest point we can directly reach
            furthest = i + 1
            for j in range(i + 2, len(path)):
                # Check if direct path from current to path[j] is collision-free
                if self._is_line_free(current, path[j]):
                    furthest = j
                else:
                    break
                    
            # Add the furthest reachable point and continue from there
            result.append(path[furthest])
            i = furthest
            
        return result
        
    def _is_line_free(self, point1, point2):
        """Check if line between two points is collision-free"""
        x1, y1 = point1
        x2, y2 = point2
        
        # Get vector and length
        dx, dy = x2 - x1, y2 - y1
        length = math.hypot(dx, dy)
        
        # Skip if points are too close
        if length < self.cell:
            return True
            
        # Sample points along line
        steps = max(5, int(length / (self.cell * 0.5)))
        
        for i in range(steps + 1):
            t = i / steps
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            
            # Convert to grid cell
            cell = self._to_cell(x, y)
            
            # Check if cell is obstacle
            if self.grid[cell[0], cell[1]] == 1:
                return False
                
        return True
        
    def visualize_grid(self, ax=None):
        """Visualize the occupancy grid for debugging"""
        import matplotlib.pyplot as plt
        created_fig = False
        
        if ax is None:
            fig, ax = plt.subplots(figsize=(10, 8))
            created_fig = True
            
        # Create a visualizable grid (transpose for correct orientation)
        vis_grid = np.zeros((self.ny, self.nx, 3), dtype=np.uint8)
        
        # Mark free space as light gray
        vis_grid[self.grid == 0] = [240, 240, 240]
        
        # Mark obstacles as black
        vis_grid[self.grid == 1] = [0, 0, 0]
        
        # Mark start as green
        if 0 <= self.start[0] < self.ny and 0 <= self.start[1] < self.nx:
            vis_grid[self.start[0], self.start[1]] = [0, 255, 0]
        
        # Mark goal as red
        if 0 <= self.goal[0] < self.ny and 0 <= self.goal[1] < self.nx:
            vis_grid[self.goal[0], self.goal[1]] = [255, 0, 0]
        
        ax.imshow(vis_grid, origin='lower')
        ax.set_title("A* Planning Grid")
        
        # Add gridlines
        ax.grid(True, which='both', color='lightgray', linewidth=0.5)
        ax.set_xticks(np.arange(-.5, self.nx, 1))
        ax.set_yticks(np.arange(-.5, self.ny, 1))
        ax.set_xticklabels([])
        ax.set_yticklabels([])
        
        # Mark start and goal with text
        if 0 <= self.start[0] < self.ny and 0 <= self.start[1] < self.nx:
            ax.text(self.start[1], self.start[0], "S", ha='center', va='center', color='white', fontsize=10, fontweight='bold')
        if 0 <= self.goal[0] < self.ny and 0 <= self.goal[1] < self.nx:
            ax.text(self.goal[1], self.goal[0], "G", ha='center', va='center', color='white', fontsize=10, fontweight='bold')
            
        if created_fig:
            plt.tight_layout()
        
        return ax

# Usage in main simulation:
# from astar_algo import AStarPlanner
# planner = AStarPlanner(rover, safety, cell_size=0.2, padding=2.5)
# waypoints = planner.plan()
# if waypoints:
#     for wx, wy in waypoints[1:]:  # Skip first waypoint (current position)
#         navigate_to_point(rover, wx, wy, ax, fig, rover_patch)