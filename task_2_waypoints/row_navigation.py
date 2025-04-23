import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import CubicSpline
import math
import time
from matplotlib.patches import Polygon
import matplotlib.transforms as transforms
from farm_safety import SafetyModule

safety = SafetyModule()
STEP = 0.4  # Reduced back to original value for smoother movement (was 0.8)
TOLERANCE = 0.5  # Slightly reduced for better precision
MAX_ATTEMPTS = 200  # Kept the same
DEBUG = False
ANIMATION_SPEED = 0.05  # Increased for slowerment

# Moved from farm_entry.py
class Rover:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.last_heading = 0.0  # Added memory for last heading
        self.history = []
        self.geofence = None
        self.entry_point = None
        self.inside_fence = False
        self.command_count = 0  # Add command count tracking
        self.blocked_directions = set()  # Set of blocked directions


    # Add to Rover class
    def log_movement(self, movement_type, distance=None, angle=None):
        """Log movement commands to the terminal"""
        if movement_type == "forward":
            print(f"⬆️ COMMAND: Move forward {distance:.2f}m")
        elif movement_type == "backward":
            print(f"⬇️ COMMAND: Move backward {distance:.2f}m") 
        elif movement_type == "turn_left":
            print(f"↩️ COMMAND: Turn left {angle:.1f}°")
        elif movement_type == "turn_right":
            print(f"↪️ COMMAND: Turn right {angle:.1f}°")
        elif movement_type == "stop":
            print("🛑 COMMAND: Stop")
        self.command_count += 1
            
    def calculate_heading_to(self, tx, ty):
        dx, dy = tx - self.x, ty - self.y
        if abs(dx)<1e-6 and abs(dy)<1e-6:
            return self.heading
        ang = math.degrees(math.atan2(dy, dx))
        return ang if ang>=0 else ang+360

    def set_position(self, x, y, heading=None, force=False, add_to_history=True):
        if self.geofence and not force:
            in_fence = self.is_inside_farm(x, y)
            if not in_fence:
                return False
        # Proceed with setting position
        if self.geofence and not force:
            if self.entry_point and self.distance_to(*self.entry_point) <= 0.8:
                self.inside_fence = True
            in_fence = self.is_inside_farm(x, y)
            if (in_fence and not self.inside_fence) or (not in_fence and self.inside_fence):
                print("⚠️ Movement blocked: would cross fence boundary")
                return False
        
        # Store previous position and heading before updating
        prev_x, prev_y = self.x, self.y
        prev_heading = self.heading
        
        # Update position with limited decimal places (point 2)
        self.x = round(x, 2)
        self.y = round(y, 2)
        
        if heading is not None:
            self.heading = round(heading % 360, 1)
            # Update last_heading when heading changes
            self.last_heading = self.heading
            
        if add_to_history:
            self.history.append((self.x, self.y))
            
        self.command_count += 1  # Increment command count
        
        # Add GPS-like position reporting (point 5)
        print(f"📍 GPS: Position [{self.x:.2f}, {self.y:.2f}], Heading: {self.heading:.1f}°")
        
        return True

    def set_geofence(self, vertices, entry_point):
        self.geofence = vertices
        self.entry_point = entry_point
        self.inside_fence = self.is_inside_farm(self.x, self.y)

    def distance_to(self, tx, ty):
        return math.hypot(tx - self.x, ty - self.y)

    def is_inside_farm(self, x, y):
        if not self.geofence:
            return True
        # Simple boundary check
        min_x = min(v[0] for v in self.geofence)
        max_x = max(v[0] for v in self.geofence)
        min_y = min(v[1] for v in self.geofence)
        max_y = max(v[1] for v in self.geofence)
        return min_x <= x <= max_x and min_y <= y <= max_y
    
    def move_forward(self, distance, ax=None, fig=None, rover_patch=None):
        # Log the movement command to terminal
        self.log_movement("forward", distance=distance)
        
        rad = math.radians(self.heading)
        target_x = self.x + distance * math.cos(rad)
        target_y = self.y + distance * math.sin(rad)
        success = self.set_position(target_x, target_y)
        if success and ax and fig and rover_patch:
            update_rover_visualization(self, ax, fig, rover_patch)
            plt.pause(0.05)  # Add pause for smoother animation
        return success
    
    # Added method to detect if rover is stuck
    def detect_and_resolve_stuck(self):
        # If this method is called, we assume potential stuck situation
        # Return recommendation for new heading if needed
        if len(self.blocked_directions) > 3:
            # Clear blocked directions and suggest a completely new heading
            self.blocked_directions.clear()
            # Try perpendicular to current heading
            new_heading = (self.heading + 90) % 360
            return new_heading
        return None

# Moved from farm_entry.py
def update_rover_visualization(rover, ax, fig, rover_patch=None):
    if rover_patch is None:
        verts = np.array([[0.7, 0], [-0.3, 0.4], [-0.3, -0.4]])
        rover_patch = Polygon(verts, closed=True, fc='blue', ec='black')
        ax.add_patch(rover_patch)
    tr = transforms.Affine2D().rotate_deg(rover.heading).translate(rover.x, rover.y)
    rover_patch.set_transform(tr + ax.transData)
    if hasattr(ax, 'path_line') and len(rover.history) > 1:
        ax.path_line.set_data(*zip(*rover.history))
    fig.canvas.draw_idle()
    plt.pause(0.01)
    return rover_patch

# Moved from farm_entry.py
import matplotlib.pyplot as plt

def visualize_turn(rover, target_heading, ax, fig, rover_patch=None, rotation_speed_factor=1):
    """
    Turn the rover to face a new heading using the shortest possible rotation.
    """
    # Calculate turn direction and angle
    delta_angle = ((target_heading - rover.heading) + 180) % 360 - 180
    direction = "turn_right" if delta_angle > 0 else "turn_left"
    abs_angle = abs(delta_angle)
    
    # Log the turning command
    rover.log_movement(direction, angle=abs_angle)
    
    current = rover.heading
    # Normalize headings to [0, 360)
    current = current % 360
    target_heading = target_heading % 360

    # Calculate clockwise and counterclockwise angles
    clockwise = (target_heading - current) % 360
    counterclockwise = (current - target_heading) % 360
    # Choose the shortest direction
    if clockwise <= counterclockwise:
        diff = clockwise
    else:
        diff = -counterclockwise

    # If already aligned or very close, snap to new heading without animation
    if abs(diff) < 2:
        rover.heading = target_heading
        return update_rover_visualization(rover, ax, fig, rover_patch)

    # Calculate steps for smooth animation (fewer steps for small angles)
    steps = max(3, int(abs(diff) / 10))  # Fewer steps for faster turns
    step_ang = diff / steps
    # Adjust pause duration based on angle size and speed factor
    pause_duration = min(0.05, 0.03 * (180 / max(1, abs(diff)))) / rotation_speed_factor

    # Animate the turn
    for i in range(1, steps + 1):
        rover.heading = (current + step_ang * i) % 360
        rover_patch = update_rover_visualization(rover, ax, fig, rover_patch)
        plt.pause(pause_duration)

    # Snap to exact final heading to avoid floating-point errors
    rover.heading = target_heading
    rover_patch = update_rover_visualization(rover, ax, fig, rover_patch)

    # Store the last navigated heading
    rover.last_heading = rover.heading

    return rover_patch
class RowNavigator:
    def __init__(self, rover):
        self.rover = rover
        self.interpolated_path = []
        self.current_waypoint_index = 0
        self.waypoint_threshold = 0.3
        self.column_spacing = 1.5
        self.column_height = 15
        self.movement_speed = 0.3
        self.current_row = 0  # Track which row we're on
        self.zigzag_pattern = True  # Enable zigzag pattern by default
        self.rows_data = []  # Store information about each row
        
    def generate_rows(self, start_x, start_y, num_strips=5, strip_length=None, spacing=None):
        if spacing is None:
            spacing = self.column_spacing
        if strip_length is None:
            strip_length = self.column_height
            
        bottom_y = start_y
        top_y = start_y + strip_length
        self.interpolated_path = []
        self.rows_data = []  # Reset rows data
        
        print(f"\n🌾 Generating {num_strips} rows with spacing {spacing:.2f}m")
        print(f"🌾 Row length: {strip_length:.2f}m")
        
        for i in range(num_strips):
            x = start_x + i * spacing
            
            # Determine row direction based on zigzag pattern
            going_up = (i % 2 == 0)
            direction_str = "↑ UP" if going_up else "↓ DOWN"
            
            # Create row points
            if going_up:
                y_points = np.linspace(bottom_y, top_y, 5)
                row_start = (x, bottom_y)
                row_end = (x, top_y)
            else:
                y_points = np.linspace(top_y, bottom_y, 5)
                row_start = (x, top_y)
                row_end = (x, bottom_y)
                
            # Store row data for reporting
            self.rows_data.append({
                'index': i,
                'x_pos': x,
                'direction': direction_str,
                'start': row_start,
                'end': row_end
            })
            
            # Add waypoints for this row
            for y in y_points:
                self.interpolated_path.append((x, y))
                
            # Add transition to next row if not the last row
            if i < num_strips - 1:
                next_x = start_x + (i + 1) * spacing
                transition_y = top_y if going_up else bottom_y
                self.interpolated_path.append((next_x, transition_y))
                
        # Print row information (point 4)
        print("\n📋 Row Navigation Plan:")
        for row in self.rows_data:
            print(f"   Row {row['index']+1}: X-position {row['x_pos']:.2f}m, Direction {row['direction']}")
            
        return self.interpolated_path

    def distance(self, p1, p2):
        return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

    def calculate_heading(self, p1, p2):
        return math.degrees(math.atan2(p2[1] - p1[1], p2[0] - p1[0])) % 360

    def heading_difference(self, current, target):
        diff = (target - current + 540) % 360 - 180
        return diff

    def smooth_turn(self, target_heading, ax=None, fig=None, rover_patch=None):
        heading_diff = self.heading_difference(self.rover.heading, target_heading)
        
        if abs(heading_diff) < 1:
            self.rover.heading = target_heading
            self.rover.last_heading = target_heading  # Remember heading
            if ax and fig and rover_patch:
                rover_patch = update_rover_visualization(self.rover, ax, fig, rover_patch)
            return rover_patch
        
        # Choose shortest turn direction and log the movement
        direction = "turn_right" if heading_diff > 0 else "turn_left"
        self.rover.log_movement(direction, angle=abs(heading_diff))
        
        # Choose shortest turn direction
        steps = max(5, int(abs(heading_diff) / 15))
        angle_step = heading_diff / steps
        
        for i in range(steps):
            self.rover.heading = (self.rover.heading + angle_step) % 360
            if ax and fig and rover_patch:
                rover_patch = update_rover_visualization(self.rover, ax, fig, rover_patch)
                plt.pause(0.02)
                
        self.rover.heading = round(target_heading, 1)  # Ensure exact heading with limited decimals
        self.rover.last_heading = self.rover.heading  # Remember heading
        
        if ax and fig and rover_patch:
            rover_patch = update_rover_visualization(self.rover, ax, fig, rover_patch)
        return rover_patch

    def move_precisely_to_point(self, target_point, ax=None, fig=None, rover_patch=None):
        max_attempts = 40
        attempts = 0
        
        print(f"🎯 Moving to point: ({target_point[0]:.2f}, {target_point[1]:.2f})")
        
        while attempts < max_attempts:
            current_pos = (self.rover.x, self.rover.y)
            dist_to_target = self.distance(current_pos, target_point)
            
            if attempts % 5 == 0:
                print(f"   Distance to target: {dist_to_target:.2f}m")
                
            if dist_to_target <= self.waypoint_threshold:
                print(f"✅ Reached target within {self.waypoint_threshold}m threshold")
                # Force exact position to avoid accumulation errors
                self.rover.set_position(target_point[0], target_point[1], force=True)
                if ax and fig and rover_patch:
                    rover_patch = update_rover_visualization(self.rover, ax, fig, rover_patch)
                return True
                
            desired_heading = self.calculate_heading(current_pos, target_point)
            
            # Only turn if heading is significantly different
            if abs(self.heading_difference(self.rover.heading, desired_heading)) > 5:
                self.smooth_turn(desired_heading, ax, fig, rover_patch)
                
            move_dist = min(self.movement_speed, dist_to_target * 0.8)
            
            # Log the forward movement
            self.rover.log_movement("forward", distance=move_dist)
            
            path = [(self.rover.x, self.rover.y),
                    (self.rover.x + move_dist * math.cos(math.radians(self.rover.heading)),
                    self.rover.y + move_dist * math.sin(math.radians(self.rover.heading)))]
            
            status, _ = safety.check_safety([self.rover.x, self.rover.y], self.rover.heading, path)
            
            if status == 'safe':
                success = self.rover.move_forward(move_dist, ax, fig, rover_patch)
                if not success:
                    print("⚠️ Movement failed - obstacle detected")
                    return False
            else:
                print(f"⚠️ Safety check failed: {status}")
                return False
                    
            attempts += 1
            plt.pause(0.01)
                
        print("⚠️ Max attempts reached")
        return False

    def navigate_to_starting_point(self, ax=None, fig=None, rover_patch=None):
        if not self.interpolated_path:
            return False
            
        starting_point = self.interpolated_path[0]
        
        # Enhanced terminal output (point 4.i)
        print(f"\n🚩 Starting point: ({starting_point[0]:.2f}, {starting_point[1]:.2f})")
        if self.rows_data:
            print(f"🌾 This is the beginning of Row 1, moving {self.rows_data[0]['direction']}")
            
        # Navigate to starting point
        result = self.move_precisely_to_point(starting_point, ax, fig, rover_patch)
        
        if result:
            self.current_waypoint_index = 0
            self.current_row = 0
            print(f"✅ Successfully reached Row 1 starting point")
        return result

    def determine_next_task(self):
        if not self.interpolated_path or self.current_waypoint_index >= len(self.interpolated_path) - 1:
            return None
        next_idx = self.current_waypoint_index + 1
        return self.interpolated_path[next_idx]

    def align_to_next_task(self, ax=None, fig=None, rover_patch=None):
        next_point = self.determine_next_task()
        if not next_point:
            return False
            
        current_pos = (self.rover.x, self.rover.y)
        desired_heading = self.calculate_heading(current_pos, next_point)
        
        # Check if we already have the right heading (avoid unnecessary turns)
        current_heading = self.rover.heading
        if abs(self.heading_difference(current_heading, desired_heading)) < 5:
            print(f"✓ Already aligned to correct heading: {current_heading:.1f}°")
            return True
            
        print(f"🧭 Aligning from {current_heading:.1f}° to {desired_heading:.1f}°")
        
        if ax and fig and rover_patch:
            rover_patch = self.smooth_turn(desired_heading, ax, fig, rover_patch)
        else:
            self.smooth_turn(desired_heading)
            
        print(f"✓ Aligned to heading: {self.rover.heading:.1f}°")
        return True

    def align_to_next_task(self, ax=None, fig=None, rover_patch=None):
        next_point = self.determine_next_task()
        if not next_point:
            return False
        desired_heading = self.calculate_heading((self.rover.x, self.rover.y), next_point)
        if ax and fig and rover_patch:
            rover_patch = self.smooth_turn(desired_heading, ax, fig, rover_patch)
        else:
            self.smooth_turn(desired_heading)
        return True

    def navigate_all_rows(self, ax=None, fig=None, rover_patch=None):
        if not self.interpolated_path:
            print("⚠️ No path generated - call generate_rows first")
            return False
        if not self.navigate_to_starting_point(ax, fig, rover_patch):
            return False
        while self.current_waypoint_index < len(self.interpolated_path) - 1:
            next_point = self.determine_next_task()
            if not next_point:
                break
            if not self.align_to_next_task(ax, fig, rover_patch):
                continue
            if self.move_precisely_to_point(next_point, ax, fig, rover_patch):
                self.current_waypoint_index += 1
        return True

    def navigate_path(self, ax=None, fig=None, rover_patch=None):
        """
        Navigate through all waypoints in the interpolated path with enhanced reporting.
        """
        print(f"🚜 COMMAND: Navigate zigzag path with {len(self.interpolated_path)} points")
    
    # Add command logging for each path segment
        for i in range(self.current_waypoint_index, len(self.interpolated_path)-1):
            current_point = self.interpolated_path[i]
            next_point = self.interpolated_path[i+1]
            print(f"➡️ COMMAND: Move from ({current_point[0]:.2f}, {current_point[1]:.2f}) to ({next_point[0]:.2f}, {next_point[1]:.2f})")
        if not self.interpolated_path:
            print("⚠️ No path generated - call generate_rows first")
            return False
            
        # Check if we're already at a waypoint
        if self.current_waypoint_index >= len(self.interpolated_path):
            print("⚠️ Navigation complete - already at end of path")
            return True
            
        # Track current row during navigation
        current_row_index = 0
        for i, row_data in enumerate(self.rows_data):
            if self.current_waypoint_index >= i*5:  # Rough estimation
                current_row_index = i
        
        print(f"\n🌾 Currently at Row {current_row_index+1}, Waypoint {self.current_waypoint_index}")
        print(f"🧭 Row direction: {self.rows_data[current_row_index]['direction']}")
        
        success = True
        while self.current_waypoint_index < len(self.interpolated_path) - 1:
            next_waypoint_index = self.current_waypoint_index + 1
            next_point = self.interpolated_path[next_waypoint_index]
            
            # Determine if we're changing rows
            new_row_index = current_row_index
            for i, row_data in enumerate(self.rows_data):
                if abs(next_point[0] - row_data['x_pos']) < 0.1:  # Close to this row's x-position
                    new_row_index = i
            
            # Report row transition if applicable
            if new_row_index != current_row_index:
                print(f"\n🔄 Transitioning from Row {current_row_index+1} to Row {new_row_index+1}")
                print(f"🧭 New row direction: {self.rows_data[new_row_index]['direction']}")
                current_row_index = new_row_index
            
            # Navigate to next point with detailed reporting
            print(f"\n🚗 Navigating to waypoint {next_waypoint_index} at ({next_point[0]:.2f}, {next_point[1]:.2f})...")
            
            # First align to the next point
            if not self.align_to_next_task(ax, fig, rover_patch):
                print("⚠️ Failed to align to next waypoint")
                success = False
                break
                
            # Then move to the next point
            if self.move_precisely_to_point(next_point, ax, fig, rover_patch):
                print(f"✅ Reached waypoint {next_waypoint_index} in Row {current_row_index+1}")
                self.current_waypoint_index = next_waypoint_index
            else:
                print(f"⚠️ Failed to reach waypoint {next_waypoint_index}")
                success = False
                break
                
            # Pause briefly for visualization
            if ax and fig:
                plt.pause(0.05)
        
        # Special case for the last waypoint
        if success and self.current_waypoint_index == len(self.interpolated_path) - 1:
            print("\n✅ Successfully navigated entire path")
            
            # Report completion of final row
            final_row = len(self.rows_data) - 1
            print(f"🎉 Completed Row {final_row+1} - All rows navigated!")
            
            # Make sure we're exactly at the final point
            final_point = self.interpolated_path[-1]
            if self.rover.distance_to(*final_point) > 0.1:  # Small tolerance
                print(f"📍 Final adjustment to end point ({final_point[0]:.2f}, {final_point[1]:.2f})...")
                self.rover.set_position(final_point[0], final_point[1], force=True)
                if ax and fig and rover_patch:
                    rover_patch = update_rover_visualization(self.rover, ax, fig, rover_patch)
                    plt.pause(0.5)  # Extended pause at the end
        
        return success
def find_best_path_angle(rover, tx, ty, blocked_angles=None):
    direct = math.degrees(math.atan2(ty - rover.y, tx - rover.x)) % 360
    if not blocked_angles or int(direct/10)*10 not in blocked_angles:
        return direct
    for off in range(10,360,10):
        for sign in (1,-1):
            ta = (direct+sign*off)%360
            if int(ta/10)*10 not in blocked_angles:
                return ta
    import random; return random.randint(0,359)

def get_float(prompt):
    while True:
        try:
            return float(input(prompt))
        except ValueError:
            print("⚠️ Please enter a valid number.")

def navigate_to_point(rover, tx, ty, ax, fig, rover_patch=None, step_size=STEP, tolerance=TOLERANCE):
    print(f"\n🚗 Navigating to point ({tx:.3f}, {ty:.3f})...\n")
    dist = rover.distance_to(tx,ty)
    attempts=0; last_dist=float('inf'); alt=False; blocked=0
    while dist>tolerance and attempts<MAX_ATTEMPTS:
        attempts+=1
        rec = rover.detect_and_resolve_stuck()
        if rec is not None:
            rover_patch = visualize_turn(rover, rec, ax, fig, rover_patch)
            alt=True; continue
        if attempts%5==0:
            if dist>last_dist*0.95 and not alt:
                print("⚠️ Limited progress detected, trying alternative approach...")
                rover.blocked_directions.clear()
                angle=(rover.heading+90+attempts%90)%360
                rover_patch=visualize_turn(rover,angle,ax,fig,rover_patch)
                step_size=min(step_size*2,dist/2); alt=True
            else:
                alt=False; step_size=min(STEP,dist/2)
            last_dist=dist
        if blocked>2:
            tgt=find_best_path_angle(rover,tx,ty,rover.blocked_directions)
            blocked=0
        else:
            tgt=rover.calculate_heading_to(tx,ty)
        diff=(tgt-rover.heading+180)%360-180
        if abs(diff)>5:
            rover_patch=visualize_turn(rover,tgt,ax,fig,rover_patch)
        step=min(step_size,dist)
        
        # Set up movement parameters
        target_x = rover.x + step * math.cos(math.radians(rover.heading))
        target_y = rover.y + step * math.sin(math.radians(rover.heading))
        path = [(rover.x, rover.y), (target_x, target_y)]
        
        # Safety check before movement
        status, recovery_data = safety.check_safety([rover.x, rover.y], rover.heading, path)
        
        ok = False  # Default to unsuccessful movement
        if status == 'safe':
            # Safe to proceed with normal movement
            ok = rover.move_forward(step, ax, fig, rover_patch)
        elif status == 'drift':
            # Handle drift scenario
            pos, heading, drift_status, updated_data = safety.handle_drift(
                [rover.x, rover.y], rover.heading, recovery_data)
            
            # Update rover position and visualize
            rover.set_position(pos[0], pos[1], heading, add_to_history=True)
            rover_patch = update_rover_visualization(rover, ax, fig, rover_patch)
            
            # Update drift data or clear it if recovered
            if drift_status == 'recovered':
                ok = True
            else:
                recovery_data = updated_data
                ok = False
                blocked += 1
        elif status in ['no-go', 'outside']:
            # Handle no-go zone or boundary violation
            pos, heading, violation_status = safety.handle_no_go_violation(
                [rover.x, rover.y], rover.heading, recovery_data)
            
            # Update rover position and visualize
            rover.set_position(pos[0], pos[1], heading, add_to_history=True)
            rover_patch = update_rover_visualization(rover, ax, fig, rover_patch)
            
            if violation_status == 'recovered':
                ok = True
            else:
                ok = False
                blocked += 1
        
        # Update visualization
        rover_patch = update_rover_visualization(rover, ax, fig, rover_patch)
        dist = rover.distance_to(tx, ty)
        
        if not ok:
            blocked += 1
            if blocked >= 2:
                ch = 45 + blocked * 15
                ch = min(ch, 180)
                rover_patch = visualize_turn(rover, (rover.heading + ch) % 360, ax, fig, rover_patch)
        else:
            blocked = 0
            
    if dist <= tolerance:
        print(f"✅ Reached target point ({rover.x:.3f}, {rover.y:.3f})")
        return True, rover_patch
        
    print("🔄 Making final approach attempt with larger step size...")
    direct = rover.calculate_heading_to(tx, ty)
    rover_patch = visualize_turn(rover, direct, ax, fig, rover_patch)
    rover.move_forward(dist * 0.9, ax, fig, rover_patch)
    fd = rover.distance_to(tx, ty)
    
    if fd <= tolerance * 1.5:
        print(f"✅ Reached target point on final attempt ({rover.x:.3f}, {rover.y:.3f})")
        return True, rover_patch
        
    print(f"⚠️ Could not reach target point. Current position: ({rover.x:.3f}, {rover.y:.3f})")
    print(f"   Distance to target: {fd:.3f}")
    return False, rover_patch

def follow_path_precisely(rover, waypoints, ax, fig, rover_patch):
    """
    Follows the planned path with ultra-high precision by enforcing strict path adherence
    
    Args:
        rover: Rover instance
        waypoints: List of (x,y) points to follow
        ax: Matplotlib axis
        fig: Matplotlib figure
        rover_patch: Visual representation of rover
    
    Returns:
        bool: True if path followed successfully, False otherwise
        rover_patch: Updated rover patch
    """
    print(f"🛣️ COMMAND: Follow path with {len(waypoints)} waypoints")

    if not waypoints or len(waypoints) < 2:
        print("⚠️ Path too short or empty")
        return False, rover_patch
    
    print(f"\n🛣️ Following planned path with {len(waypoints)} waypoints...")
    
    # Constants for strict path following - adjusted for speed
    PATH_STEP = 0.6  # Increased step size for faster movement (was 0.2)
    PATH_TOLERANCE = 0.05  # Small tolerance to enforce strict adherence
    ANIMATION_SPEED = 0.005  # Faster animation (was 0.01)
    ROTATION_STEP_FACTOR = 2  # Rotate faster
    
    # Start with current position
    start_idx = 0
    # Find closest waypoint if we're not already at the first one
    if rover.distance_to(*waypoints[0]) > PATH_TOLERANCE:
        closest_idx = 0
        min_dist = float('inf')
        for i, wp in enumerate(waypoints):
            dist = rover.distance_to(*wp)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # If we're closer to a waypoint further along the path, start from there
        if closest_idx > 0 and min_dist < PATH_TOLERANCE:
            start_idx = closest_idx
            print(f"Starting from waypoint {start_idx} which is closest to current position")
        else:
            # We need to first move to the first waypoint
            print(f"Moving to the first waypoint at ({waypoints[0][0]:.2f}, {waypoints[0][1]:.2f})")
            initial_heading = rover.calculate_heading_to(*waypoints[0])
            rover_patch = visualize_turn(rover, initial_heading, ax, fig, rover_patch, rotation_speed_factor=ROTATION_STEP_FACTOR)
            
            # Don't teleport - move properly to first waypoint
            init_distance = rover.distance_to(*waypoints[0])
            if init_distance > PATH_TOLERANCE:
                segments = max(2, int(init_distance / PATH_STEP))
                step_dist = init_distance / segments
                for _ in range(segments):
                    success = rover.move_forward(step_dist, ax, fig, rover_patch)
                    if not success:
                        # If blocked, try with smaller steps
                        half_step = step_dist / 2
                        if half_step > 0.1:  # Don't try with too small steps
                            success = rover.move_forward(half_step, ax, fig, rover_patch)
                    rover_patch = update_rover_visualization(rover, ax, fig, rover_patch)
                    plt.pause(ANIMATION_SPEED)
    
    # For path visualization
    actual_path = []
    path_line = None
    
    # Traverse waypoints
    for i in range(start_idx, len(waypoints)-1):
        current_wp = waypoints[i]
        next_wp = waypoints[i+1]
        
        print(f"\n📍 Moving from waypoint {i} to {i+1}: ({current_wp[0]:.2f}, {current_wp[1]:.2f}) → ({next_wp[0]:.2f}, {next_wp[1]:.2f})")
        
        # Calculate segment vector and length
        segment_vec = (next_wp[0] - current_wp[0], next_wp[1] - current_wp[1])
        segment_len = math.hypot(*segment_vec)
        
        if segment_len < 0.01:  # Skip tiny segments
            continue
            
        # Unit vector along segment
        unit_vec = (segment_vec[0]/segment_len, segment_vec[1]/segment_len)
        
        # Align precisely to segment direction with faster rotation
        segment_heading = math.degrees(math.atan2(segment_vec[1], segment_vec[0])) % 360
        rover_patch = visualize_turn(rover, segment_heading, ax, fig, rover_patch, rotation_speed_factor=ROTATION_STEP_FACTOR)
        
        # Before starting segment, ensure we're exactly at the start point (if not already there)
        if rover.distance_to(*current_wp) > PATH_TOLERANCE:
            # Move to start point without teleporting
            remaining_dist = rover.distance_to(*current_wp)
            segments = max(2, int(remaining_dist / PATH_STEP))
            step_dist = remaining_dist / segments
            
            for _ in range(segments):
                if rover.distance_to(*current_wp) <= PATH_TOLERANCE:
                    break
                success = rover.move_forward(step_dist, ax, fig, rover_patch)
                if not success:
                    # Try with smaller step if blocked
                    rover.move_forward(step_dist/2, ax, fig, rover_patch)
                rover_patch = update_rover_visualization(rover, ax, fig, rover_patch)
                plt.pause(ANIMATION_SPEED)
        
        # Calculate appropriate number of interpolated points for this segment
        num_interp = max(3, int(segment_len / PATH_STEP))
        
        # Move along the segment with precise steps
        for j in range(1, num_interp + 1):
            t = j / num_interp
            interp_point = (
                current_wp[0] + t * segment_vec[0],
                current_wp[1] + t * segment_vec[1]
            )
            
            # Always ensure heading is aligned with path
            point_heading = rover.calculate_heading_to(*interp_point)
            if abs((point_heading - rover.heading + 180) % 360 - 180) > 1:
                rover_patch = visualize_turn(rover, point_heading, ax, fig, rover_patch, 
                                            rotation_speed_factor=ROTATION_STEP_FACTOR)
            
            # Calculate exact distance to move
            move_dist = rover.distance_to(*interp_point)
            
            # Move to interpolated point without teleporting
            if move_dist > PATH_TOLERANCE:
                success = rover.move_forward(move_dist, ax, fig, rover_patch)
                
                # If direct movement fails, try with smaller steps
                if not success and move_dist > PATH_STEP:
                    smaller_step = min(PATH_STEP, move_dist/2)
                    success = rover.move_forward(smaller_step, ax, fig, rover_patch)
                
                rover_patch = update_rover_visualization(rover, ax, fig, rover_patch)
                
                # Visualize the actual path
                actual_path.append((rover.x, rover.y))
                if len(actual_path) > 1 and path_line:
                    safe_remove(path_line)
                if len(actual_path) > 1:
                    path_x, path_y = zip(*actual_path)
                    path_line = ax.plot(path_x, path_y, 'g-', linewidth=1, alpha=0.7)[0]
                
                fig.canvas.draw_idle()
                plt.pause(ANIMATION_SPEED)
                
            # Safety check if available
            if hasattr(safety, 'check_safety'):
                status, _ = safety.check_safety([rover.x, rover.y], rover.heading, [(rover.x, rover.y), interp_point])
                if status != 'safe':
                    print("⚠️ Safety violation detected during path following!")
                    return False, rover_patch
    
    # For the final waypoint, use exact positioning with proper movement
    last_wp = waypoints[-1]
    final_heading = rover.calculate_heading_to(*last_wp)
    rover_patch = visualize_turn(rover, final_heading, ax, fig, rover_patch, rotation_speed_factor=ROTATION_STEP_FACTOR)
    
    # Move directly to last waypoint without teleporting
    final_dist = rover.distance_to(*last_wp)
    if final_dist > PATH_TOLERANCE:
        # Break into smaller steps
        segments = max(2, int(final_dist / PATH_STEP))
        step_dist = final_dist / segments
        
        for _ in range(segments):
            if rover.distance_to(*last_wp) <= PATH_TOLERANCE:
                break
            success = rover.move_forward(step_dist, ax, fig, rover_patch)
            if not success:
                # Try smaller step if blocked
                rover.move_forward(step_dist/2, ax, fig, rover_patch)
            rover_patch = update_rover_visualization(rover, ax, fig, rover_patch)
            plt.pause(ANIMATION_SPEED)
    
    print("✅ Successfully followed the planned path with precision")
    return True, rover_patch

def safe_remove(element):
    if element:
        try:
            element.remove()
            return True
        except:
            if DEBUG: print(f"Warning: failed to remove {element}")
    return False



def run_simulation():
    plt.rcParams['figure.max_open_warning'] = 50
    rover = Rover()
    print("🚜 Farm Rover Navigation Simulation 🚜")
    print("=====================================")
    farm_width = get_float(" Farm width: ")
    farm_height = get_float(" Farm height: ")
    min_x = -farm_width / 2
    max_x = farm_width / 2
    min_y = -farm_height / 2
    max_y = farm_height / 2
    verts = [(min_x, min_y), (max_x, min_y), (max_x, max_y), (min_x, max_y)]
    entry_point = (0, min_y)
    rover.set_geofence(verts, entry_point)
    safety.set_geofence(verts)
    plt.ion()
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_title("Rover Farm Navigation Simulation")
    margin = max(farm_width, farm_height) * 0.2
    ax.set_xlim(min_x - margin, max_x + margin)
    ax.set_ylim(min_y - margin, max_y + margin)
    ax.grid(True)
    fence = Polygon(np.array(verts), closed=True, facecolor='lightgreen', edgecolor='darkgreen', alpha=0.3)
    ax.add_patch(fence)
    ax.scatter(entry_point[0], entry_point[1], c='purple', s=100, marker='o', label='Farm Entry', zorder=10)
    path_line, = ax.plot([], [], 'b-', alpha=0.5, label='Path')
    ax.path_line = path_line
    ax.legend(loc='upper left')
    fig.canvas.draw_idle()
    plt.pause(0.5)
    rover.set_position(entry_point[0], entry_point[1], force=True, add_to_history=False)
    rover.history.append((rover.x, rover.y))
    rover_patch = update_rover_visualization(rover, ax, fig)
    print(f"\n✅ Starting simulation at farm entry point: ({entry_point[0]:.2f}, {entry_point[1]:.2f})")
    spacing = 1.5
    start_x = min_x
    start_y = min_y
    strip_length = max_y - min_y
    num_strips = math.floor((max_x - min_x) / spacing) + 1
    row_navigator = RowNavigator(rover)
    row_navigator.generate_rows(start_x, start_y, num_strips, strip_length, spacing)
    row_navigator.navigate_all_rows(ax, fig, rover_patch)
    plt.ioff()
    plt.show(block=True)

if __name__ == "__main__":
    try:
        run_simulation()
    except KeyboardInterrupt:
        print("\n\n🛑 Simulation terminated by user.")
    except Exception as e:
        print(f"\n❌ Simulation error: {e}")