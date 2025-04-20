import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import CubicSpline
import math
import time
from farm_safety import SafetyModule
safety = SafetyModule()

class RowNavigator:
    def __init__(self, rover):
        self.rover = rover
        self.current_waypoint_index = 0
        self.interpolated_path = []  # Path of discrete waypoints
        self.waypoint_threshold = 0.1  # Keep the precise threshold
        self.column_spacing = 1.5
        self.column_height = 15
        self.zigzag_pattern = True  # This only affects waypoint generation pattern
        self.movement_speed = 0.10 # Increased from 0.05 to 0.2 for faster movement

    def identify_current_row(self):
        """Task 2.ii: Identify which row the closest point belongs to"""
        if not self.interpolated_path or self.current_waypoint_index < 0:
            print("⚠️ No path or closest point identified yet")
            return None
        
        # Calculate which row this belongs to based on X position
        closest_point = self.interpolated_path[self.current_waypoint_index]
        row_index = int((closest_point[0] - self.interpolated_path[0][0]) / self.column_spacing)
        
        print(f"\n📋 Task Current point belongs to Row #{row_index + 1}")
        print(f"   Row X-position: {closest_point[0]:.3f}")
        
        # Determine if we're closer to the top or bottom of the row
        if row_index % 2 == 0:
            # Even rows go bottom to top
            position = "bottom" if closest_point[1] < (self.interpolated_path[0][1] + self.column_height/2) else "top"
        else:
            # Odd rows go top to bottom
            position = "top" if closest_point[1] > (self.interpolated_path[0][1] + self.column_height/2) else "bottom"
        
        print(f"   Position in row: Near {position}")
        print(f"   ✓ Defining Row #{row_index + 1} as the first row task")
        self.first_row = row_index
        return row_index

    def determine_next_row_direction(self):
        """Task Determine direction to the next row"""
        current_row = self.identify_current_row()
        if current_row is None:
            return None
        
        # In our zigzag pattern:
        # - If at even row and near top → go right
        # - If at odd row and near bottom → go right
        # - Otherwise we're in the middle of a row, so we continue in the current direction
        
        closest_point = self.interpolated_path[self.current_waypoint_index]
        y_threshold = self.interpolated_path[0][1] + self.column_height * 0.9  # Near top
        y_bottom_threshold = self.interpolated_path[0][1] + self.column_height * 0.1  # Near bottom
        
        if (current_row % 2 == 0 and closest_point[1] > y_threshold) or \
        (current_row % 2 == 1 and closest_point[1] < y_bottom_threshold):
            next_row_direction = "right"
            next_row = current_row + 1
        elif current_row % 2 == 0:
            next_row_direction = "up"
            next_row = current_row
        else:
            next_row_direction = "down"
            next_row = current_row
        
        print(f"\n📋 Task Next row direction: {next_row_direction}")
        if next_row != current_row:
            print(f"   Next row will be Row #{next_row + 1}")
        
        return next_row_direction

    def generate_rows(self, start_x, start_y, num_strips=5, strip_length=None, spacing=None):
        if spacing is None:
            spacing = self.column_spacing
        if strip_length is None:
            strip_length = self.column_height

        bottom_y = start_y
        top_y = start_y + strip_length

        self.interpolated_path = []
        
        for i in range(num_strips):
            x = start_x + i * spacing
            
            # Generate waypoints for the vertical segment
            if i % 2 == 0:
                # Even columns: bottom to top
                y_points = np.linspace(bottom_y, top_y, 5)
            else:
                # Odd columns: top to bottom
                y_points = np.linspace(top_y, bottom_y, 5)
            
            # Add all waypoints for this column
            for y in y_points:
                self.interpolated_path.append((x, y))
            
            # Add horizontal transition to the next column (except for the last column)
            if i < num_strips - 1:
                next_x = start_x + (i + 1) * spacing
                transition_y = top_y if i % 2 == 0 else bottom_y
                
                # Use fewer intermediate points for horizontal transitions
                intermediate_points = 3  # Reduced from 5 for faster movement
                for j in range(1, intermediate_points):
                    interp_x = x + (next_x - x) * j / intermediate_points
                    self.interpolated_path.append((interp_x, transition_y))
                
                # Add the transition point explicitly
                self.interpolated_path.append((next_x, transition_y))

        return self.interpolated_path

    def distance(self, p1, p2):
        """Calculate Euclidean distance between two points"""
        return math.hypot(p2[0] - p1[0], p2[1] - p1[1])
    
    def calculate_heading(self, p1, p2):
        """Calculate heading angle between two points (in degrees)"""
        return math.degrees(math.atan2(p2[1] - p1[1], p2[0] - p1[0])) % 360
    
    def heading_difference(self, current, target):
        """Calculate the difference between current and target heading"""
        diff = (target - current + 540) % 360 - 180
        return diff

    def find_closest_waypoint(self):
        """Find the closest waypoint in the path"""
        if not self.interpolated_path:
            print("⚠️ No valid path")
            return None
        min_dist = float('inf')
        closest_idx = 0
        for i, point in enumerate(self.interpolated_path):
            dist = self.distance((self.rover.x, self.rover.y), point)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        self.current_waypoint_index = closest_idx
        print(f"🔍 Found closest waypoint: #{closest_idx}")
        print(f"   Current position: ({self.rover.x:.3f}, {self.rover.y:.3f})")
        print(f"   Waypoint position: ({self.interpolated_path[closest_idx][0]:.3f}, {self.interpolated_path[closest_idx][1]:.3f})")
        print(f"   Distance: {min_dist:.3f}m")
        return self.interpolated_path[closest_idx]

    def align_to_path(self, ax=None, fig=None, rover_patch=None):
        """Align rover heading to the direction of the next waypoint"""
        if not self.interpolated_path or self.current_waypoint_index >= len(self.interpolated_path) - 1:
            print("⚠️ No active path or at end of path")
            return False
        
        # Get current position and next waypoint
        current_pos = (self.rover.x, self.rover.y)
        next_pt = self.interpolated_path[self.current_waypoint_index + 1]
        
        # Calculate desired heading to next waypoint
        desired_heading = self.calculate_heading(current_pos, next_pt)
        
        if ax and fig and rover_patch:
            from farm_entry import visualize_turn
            rover_patch = visualize_turn(self.rover, desired_heading, ax, fig, rover_patch)
            print(f"🔄 Aligned rover to heading: {desired_heading:.1f}°")
            print(f"   Current position: ({self.rover.x:.3f}, {self.rover.y:.3f})")
        else:
            self.rover.heading = desired_heading
            print(f"🔄 Aligned rover to heading: {desired_heading:.1f}°")
            print(f"   Current position: ({self.rover.x:.3f}, {self.rover.y:.3f})")
        return True

    def move_precisely_to_point(self, target_point, ax=None, fig=None, rover_patch=None):
        """Move precisely to a specific point using larger, faster steps"""
        from farm_entry import update_rover_visualization, visualize_turn
        
        max_attempts = 100  # Reduced from 200 for faster execution
        attempts = 0
        step_size = self.movement_speed  # Larger step size for faster movement
        
        while attempts < max_attempts:
            # Calculate distance to target
            current_pos = (self.rover.x, self.rover.y)
            dist_to_target = self.distance(current_pos, target_point)
            
            # Check if we've reached the target
            if dist_to_target <= self.waypoint_threshold:
                # Force exact position to ensure precision
                self.rover.x = target_point[0]
                self.rover.y = target_point[1]
                if ax and fig and rover_patch:
                    rover_patch = update_rover_visualization(self.rover, ax, fig, rover_patch)
                    plt.pause(0.01)  # Faster visualization update
                print(f"   Final position: ({self.rover.x:.3f}, {self.rover.y:.3f})")
                return True
            
            # Align heading precisely to target
            desired_heading = self.calculate_heading(current_pos, target_point)
            heading_diff = self.heading_difference(self.rover.heading, desired_heading)
            
            if abs(heading_diff) > 1:  # Keep the strict heading alignment
                if ax and fig and rover_patch:
                    rover_patch = visualize_turn(self.rover, desired_heading, ax, fig, rover_patch)
                else:
                    self.rover.heading = desired_heading
            
            # Move a larger step toward the target
            move_dist = min(step_size, dist_to_target)

# Calculate the target position based on heading and move distance
            target_x = self.rover.x + move_dist * math.cos(math.radians(self.rover.heading))
            target_y = self.rover.y + move_dist * math.sin(math.radians(self.rover.heading))
            path = [(self.rover.x, self.rover.y), (target_x, target_y)]

            # Check safety before moving
            from farm_safety import safety  # Make sure this is imported at the top of the file
            status, recovery_data = safety.check_safety([self.rover.x, self.rover.y], self.rover.heading, path)

            if status == 'safe':
                    if ax and fig and rover_patch:
                        success = self.rover.move_forward(move_dist, ax, fig, rover_patch)
                        rover_patch = update_rover_visualization(self.rover, ax, fig, rover_patch)
                        plt.pause(0.01)  # Faster updates for quicker animation
                    else:
                        success = self.rover.move_forward(move_dist)
            elif status == 'drift':
                    # Handle drift scenario
                    pos, heading, drift_status, updated_data = safety.handle_drift(
                        [self.rover.x, self.rover.y], self.rover.heading, recovery_data)
                    
                    # Update rover position and heading
                    self.rover.x, self.rover.y = pos
                    self.rover.heading = heading
                    success = True

                    if ax and fig and rover_patch:
                        rover_patch = update_rover_visualization(self.rover, ax, fig, rover_patch)
                        plt.pause(0.01)
                        print(f"🔄 Drift corrected: New heading {heading:.1f}°")

            elif status in ['no-go', 'outside']:
    # Handle no-go zone or boundary violation
                pos, heading, violation_status = safety.handle_no_go_violation(
                    [self.rover.x, self.rover.y], self.rover.heading, recovery_data)
                
                # Update rover position and heading
                self.rover.x, self.rover.y = pos
                self.rover.heading = heading
                success = False  # Mark as unsuccessful to trigger alternative path finding
                
                # Update visualization if available
                if ax and fig and rover_patch:
                    rover_patch = update_rover_visualization(self.rover, ax, fig, rover_patch)
                    plt.pause(0.01)
                    print(f"⚠️ Safety violation handled: {violation_status}")            


            if not success:
                print("⚠️ Movement blocked, attempting to adjust")
                print(f"   Current position: ({self.rover.x:.3f}, {self.rover.y:.3f})")
                # Try a very small sidestep and continue
                sidestep_angle = (desired_heading + 90) % 360
                if ax and fig and rover_patch:
                    rover_patch = visualize_turn(self.rover, sidestep_angle, ax, fig, rover_patch)
                    self.rover.move_forward(0.1, ax, fig, rover_patch)
                    rover_patch = visualize_turn(self.rover, desired_heading, ax, fig, rover_patch)
                else:
                    self.rover.heading = sidestep_angle
                    self.rover.move_forward(0.1)
                    self.rover.heading = desired_heading
            
            attempts += 1
            
            # Only print status updates every 5 steps to reduce console output
            if attempts % 5 == 0:
                print(f"   Moving: distance remaining = {dist_to_target:.3f}m")
                print(f"   Current position: ({self.rover.x:.3f}, {self.rover.y:.3f})")
        
        print(f"⚠️ Failed to reach point after {max_attempts} attempts")
        print(f"   Current position: ({self.rover.x:.3f}, {self.rover.y:.3f})")
        return False
    
    def navigate_direct_to_closest(self, ax=None, fig=None, rover_patch=None):
    
       closest_point = self.find_closest_waypoint()
       if not closest_point:
         return False
        
       print(f"\n🚜 Navigating directly to closest point on path: ({closest_point[0]:.3f}, {closest_point[1]:.3f})")
    
    # Set the rover position directly to the closest point (teleport)
       self.rover.x = closest_point[0]
       self.rover.y = closest_point[1]
    
    # Update visualization if available
       if ax and fig and rover_patch:
         from farm_entry import update_rover_visualization
         rover_patch = update_rover_visualization(self.rover, ax, fig, rover_patch)
         plt.pause(0.01)

       print(f"✅ Successfully reached the closest waypoint #{self.current_waypoint_index}")
       print(f"   Final position: ({self.rover.x:.3f}, {self.rover.y:.3f})")
       return True


    def determine_next_task(self):
        """Task 2.i: Determine the next task (next waypoint in the row)"""
        if not self.interpolated_path or self.current_waypoint_index >= len(self.interpolated_path) - 1:
            print("⚠️ No more waypoints available")
            return None
        
        next_idx = self.current_waypoint_index + 1
        next_point = self.interpolated_path[next_idx]
        
        print(f"\n📋 Determined next task - waypoint #{next_idx}")
        print(f"   Next waypoint: ({next_point[0]:.3f}, {next_point[1]:.3f})")
        print(f"   Current position: ({self.rover.x:.3f}, {self.rover.y:.3f})")
        print(f"   Distance: {self.distance((self.rover.x, self.rover.y), next_point):.3f}m")
        
        return next_point

    def align_to_next_task(self, ax=None, fig=None, rover_patch=None):
        """ Align rover direction to the orientation of next task"""
        next_point = self.determine_next_task()
        if not next_point:
            return False
        
        # Get current position
        current_pos = (self.rover.x, self.rover.y)
        
        # Calculate desired heading to next waypoint
        desired_heading = self.calculate_heading(current_pos, next_point)
        
        print(f"\n🔄 Aligning rover to next task orientation")
        
        if ax and fig and rover_patch:
            from farm_entry import visualize_turn
            rover_patch = visualize_turn(self.rover, desired_heading, ax, fig, rover_patch)
            print(f"   Aligned rover to heading: {desired_heading:.1f}°")
            print(f"   Current position: ({self.rover.x:.3f}, {self.rover.y:.3f})")
        else:
            self.rover.heading = desired_heading
            print(f"   Aligned rover to heading: {desired_heading:.1f}°")
            print(f"   Current position: ({self.rover.x:.3f}, {self.rover.y:.3f})")
        
        return True

    def navigate_all_rows(self, ax=None, fig=None, rover_patch=None):
        """Navigate through the path with the updated task structure"""
        print(f"\n🚜 Starting navigation of path...\n")
        if not self.interpolated_path:
            print("⚠️ No path generated - call generate_rows first")
            return False
            
        # Task 1: Navigate directly to the closest point (no intermediate points)
        print("\n📋 Task 1: Navigate directly to closest point on path")
        if not self.navigate_direct_to_closest(ax, fig, rover_patch):
            print("⚠️ Failed to reach the closest waypoint. Aborting.")
            return False
        
        # Now at closest point (start of row), proceed with next tasks
        
        # Task 2.i: Determine next task
        next_point = self.determine_next_task()
        if not next_point:
            print("⚠️ No next task available. Ending navigation.")
            return False
        
        # Task 2.ii: Align to next task orientation
        if not self.align_to_next_task(ax, fig, rover_patch):
            print("⚠️ Failed to align to next task. Attempting to continue anyway.")
        
        # Task 3: Navigate through the identified row
        print("\n📋 Task 3: Start navigating in the identified row")
        return self.navigate_path(ax, fig, rover_patch)

    def navigate_path(self, ax=None, fig=None, rover_patch=None):
        """Task 3: Navigate through the path, strictly following all waypoints"""
        from farm_entry import update_rover_visualization, visualize_turn
        import matplotlib.pyplot as plt
        
        print(f"\n🚜 Task Navigating path with {len(self.interpolated_path) - self.current_waypoint_index - 1} remaining waypoints...\n")
        current_row = self.identify_current_row()
        print(f"\n🚜 Starting navigation of Row #{current_row + 1} (first task)\n")
    
        # Start from current waypoint and visit all remaining waypoints
        while self.current_waypoint_index < len(self.interpolated_path) - 1:
            # Task 3.ii: Read the current location
            current_pos = (self.rover.x, self.rover.y)
            print(f"\n📍 Task Current location: ({self.rover.x:.3f}, {self.rover.y:.3f})")
            
            # Task 3.iii: Find the closest next waypoint in the row
            next_idx = self.current_waypoint_index + 1
            next_point = self.interpolated_path[next_idx]
            dist_to_next = self.distance(current_pos, next_point)
            
            print(f"🎯 Task Found next waypoint #{next_idx}: ({next_point[0]:.3f}, {next_point[1]:.3f})")
            print(f"   Distance to next waypoint: {dist_to_next:.3f}m")
            
            # Use the precise movement method to reach the exact waypoint
            reached = self.move_precisely_to_point(next_point, ax, fig, rover_patch)
            
            if reached:
                self.current_waypoint_index = next_idx
                print(f"✅ Reached waypoint #{next_idx} exactly")
                new_row = self.identify_current_row()
                if new_row != current_row:
                    current_row = new_row
                    next_direction = self.determine_next_row_direction()
                    print(f"\n🚜 Now navigating Row #{current_row + 1}")
                    print(f"   Next direction: {next_direction}")
            else:
                print(f"⚠️ Failed to reach waypoint #{next_idx}")
                # Even if we can't reach it perfectly, we still advance to the next waypoint
                self.current_waypoint_index = next_idx
        
        print(f"✅ Reached end of path")
        print(f"   Final position: ({self.rover.x:.3f}, {self.rover.y:.3f})")
        return True