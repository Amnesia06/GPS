import numpy as np
import matplotlib.pyplot as plt

class BorderNavigator:
    def __init__(self, boundary_points, rover, entry_point):
        self.boundary_points = boundary_points
        self.rover = rover
        self.entry_point = entry_point
        self.offset_boundary_points = []
        # Compute the offset path immediately with 0.5m offset
        self.compute_offset_boundary_path(0.5)

    def compute_offset_boundary_path(self, offset_distance):
        """Compute an offset path inside the boundary by the given distance"""
        offset_points = []
        n = len(self.boundary_points)
        for i in range(n):
            prev = (i - 1) % n
            next_ = (i + 1) % n
            V1 = np.array(self.boundary_points[i]) - np.array(self.boundary_points[prev])
            V2 = np.array(self.boundary_points[next_]) - np.array(self.boundary_points[i])
            N1 = np.array([-V1[1], V1[0]]) / np.linalg.norm(V1)
            N2 = np.array([-V2[1], V2[0]]) / np.linalg.norm(V2)
            N_avg = (N1 + N2) / np.linalg.norm(N1 + N2)
            P_offset = np.array(self.boundary_points[i]) + offset_distance * N_avg
            offset_points.append(P_offset.tolist())

        self.offset_boundary_points = offset_points
        return offset_points

    def find_closest_offset_point(self):
        """Find the closest point on the offset boundary path"""
        min_dist = float('inf')
        closest_idx = 0
        for idx, point in enumerate(self.offset_boundary_points):
            dist = self.rover.distance_to(*point)
            if dist < min_dist:
                min_dist = dist
                closest_idx = idx
        return closest_idx, self.offset_boundary_points[closest_idx], min_dist

    def get_next_offset_point(self, current_idx, step):
        """Get the next point on the offset boundary path"""
        n = len(self.offset_boundary_points)
        next_idx = (current_idx + step) % n
        return next_idx, self.offset_boundary_points[next_idx]

    def find_closest_boundary_point(self):
        """Find the closest point on the original boundary path"""
        min_dist = float('inf')
        closest_idx = 0
        for idx, point in enumerate(self.boundary_points):
            dist = self.rover.distance_to(*point)
            if dist < min_dist:
                min_dist = dist
                closest_idx = idx
        return closest_idx, self.boundary_points[closest_idx], min_dist

    def get_next_boundary_point(self, current_idx, step):
        """Get the next point on the original boundary path"""
        n = len(self.boundary_points)
        next_idx = (current_idx + step) % n
        return next_idx, self.boundary_points[next_idx]
    
    def calculate_optimal_direction(self, target_idx):
        """Calculate the optimal direction to reach the target index"""
        current_idx, _, _ = self.find_closest_offset_point()
        n = len(self.offset_boundary_points)
        # Calculate clockwise and counterclockwise distances
        clockwise_dist = (target_idx - current_idx) % n
        counterclockwise_dist = (current_idx - target_idx) % n
        
        # Return 1 for clockwise, -1 for counterclockwise
        return 1 if clockwise_dist <= counterclockwise_dist else -1

    def visualize_path(self, ax, fig):
        """Visualize current paths and rover position"""
        # Clear previous paths if any
        for line in ax.lines:
            line.remove()
            
        # Plot original boundary in green
        boundary_x = [p[0] for p in self.boundary_points]
        boundary_y = [p[1] for p in self.boundary_points]
        boundary_x.append(boundary_x[0])  # Close the loop
        boundary_y.append(boundary_y[0])
        ax.plot(boundary_x, boundary_y, 'g-', linewidth=2, label='Farm Boundary')
        
        # Plot offset path in blue (this is the rover's waypath)
        offset_x = [p[0] for p in self.offset_boundary_points]
        offset_y = [p[1] for p in self.offset_boundary_points]
        offset_x.append(offset_x[0])  # Close the loop
        offset_y.append(offset_y[0])
        ax.plot(offset_x, offset_y, 'b-', linewidth=2, label='Rover Waypath (0.5m offset)')
        
        # Plot entry point
        ax.plot(self.entry_point[0], self.entry_point[1], 'ro', markersize=10, label='Entry Point')
        
        # Update the legend
        ax.legend()
        fig.canvas.draw()
        plt.pause(0.1)

    def navigate_to_entry(self, entry_idx, ax, fig, rover_patch):
        # Visualize paths at the beginning
        self.visualize_path(ax, fig)
        
        # Phase 1: Move directly to the offset path (blue waypath)
        print("🚜 Phase 1: Moving toward blue waypath (0.5m offset)...")
        max_attempts = 100
        attempts = 0
        offset_idx, offset_point, offset_dist = self.find_closest_offset_point()
        
        while offset_dist > 0.1 and attempts < max_attempts:
            attempts += 1
            target_heading = self.rover.calculate_heading_to(*offset_point)
            if attempts == 1 or attempts % 10 == 0:
                rover_patch = visualize_turn(self.rover, target_heading, ax, fig, rover_patch)
            dist_to_move = min(0.5, offset_dist / 2)
            self.rover.move_forward(dist_to_move, ax, fig, rover_patch)
            rover_patch = update_rover_visualization(self.rover, ax, fig, rover_patch)
            offset_idx, offset_point, offset_dist = self.find_closest_offset_point()
            if attempts % 5 == 0:
                print(f"🔄 Moving toward offset path - Distance: {offset_dist:.2f}m")

        print(f"✅ Reached offset path at position ({offset_point[0]:.2f}, {offset_point[1]:.2f})")
        plt.pause(1.0)

        # Phase 2: Follow offset path toward the entry point
        print("🚜 Phase 2: Following offset path to near entry point...")
        direction = self.calculate_optimal_direction(entry_idx)
        lookahead = 10
        last_heading = self.rover.heading
        attempts = 0
        n = len(self.offset_boundary_points)
        max_attempts = 300

        while attempts < max_attempts:
            attempts += 1
            next_idx, next_point = self.get_next_offset_point(offset_idx, direction * lookahead)
            target_heading = self.rover.calculate_heading_to(*next_point)
            if abs(target_heading - last_heading) > 5:
                rover_patch = visualize_turn(self.rover, target_heading, ax, fig, rover_patch)
                last_heading = target_heading
            dist_to_move = min(1.0, self.rover.distance_to(*next_point) / 2)
            self.rover.move_forward(dist_to_move, ax, fig, rover_patch)
            rover_patch = update_rover_visualization(self.rover, ax, fig, rover_patch)
            offset_idx, _, _ = self.find_closest_offset_point()

            # Check proximity to entry point by index difference
            idx_diff = min(abs(offset_idx - entry_idx), n - abs(offset_idx - entry_idx))
            if idx_diff <= 5:
                print(f"✅ Near entry point at offset index {offset_idx}")
                break

            dist_to_entry = self.rover.distance_to(*self.entry_point)
            lookahead = 5 if dist_to_entry < 3 else 10
            if attempts % 20 == 0:
                print(f"🔄 Offset path progress - Distance to entry: {dist_to_entry:.2f}m")
                self.visualize_path(ax, fig)

        # Phase 3: Move directly to the entry point
        print("🚜 Phase 3: Moving directly to entry point...")
        attempts = 0
        max_attempts = 50
        while self.rover.distance_to(*self.entry_point) > 0.1 and attempts < max_attempts:
            attempts += 1
            target_heading = self.rover.calculate_heading_to(*self.entry_point)
            rover_patch = visualize_turn(self.rover, target_heading, ax, fig, rover_patch)
            dist_to_move = min(0.5, self.rover.distance_to(*self.entry_point) / 2)
            self.rover.move_forward(dist_to_move, ax, fig, rover_patch)
            rover_patch = update_rover_visualization(self.rover, ax, fig, rover_patch)

        if self.rover.distance_to(*self.entry_point) <= 0.1:
            print("✅ Successfully reached farm entry point!")
            return True, rover_patch
        else:
            print("⚠️ Could not reach entry point after attempts")
            return False, rover_patch