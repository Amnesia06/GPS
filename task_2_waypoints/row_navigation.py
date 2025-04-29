import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import CubicSpline
import math
import time
from matplotlib.patches import Polygon
import matplotlib.transforms as transforms
from farm_safety import SafetyModule
import csv
import os
from datetime import datetime
import random
from collections import OrderedDict
safety = SafetyModule()
STEP = 1.6  # Reduced back to original value for smoother movement (was 0.8)
TOLERANCE = 0.5  # Slightly reduced for better precision
MAX_ATTEMPTS = 200  # Kept the same
DEBUG = False
ANIMATION_SPEED = 0.001  # Increased for slowerment

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
        self.navigator = None


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
        
        # Update position with limited decimal places
        self.x = round(x, 2)
        self.y = round(y, 2)

        if hasattr(self, 'failsafe'):
            self.failsafe.update_gps_status(has_fix=True, satellites=10, hdop=1.0)
            self.failsafe.update_internet_status(connected=True, latency=0.1)
            self.failsafe.update_module_communication()
        
        if heading is not None:
            self.heading = round(heading % 360, 1)
            # Update last_heading when heading changes
            self.last_heading = self.heading
            
        if add_to_history:
            self.history.append((self.x, self.y))
            
        self.command_count += 1  # Increment command count
        
        # Get compass direction - using standard compass conversion
        compass_direction = self.get_compass_direction(self.heading)
        
        # Calculate standard compass bearing for display
        standard_bearing = (90 - self.heading) % 360
        
        # Add GPS-like position reporting with compass direction and standard bearing
        print(f"📍 GPS: Position [{self.x:.2f}, {self.y:.2f}], Heading: {self.heading:.1f}° (Compass: {standard_bearing:.1f}° {compass_direction})")

        # Use standard compass heading for logging
        compass_heading = self.get_compass_direction(self.heading)
        
        # Get or create a run ID
        if not hasattr(self, 'run_id'):
            # First time initialization
            self.run_id = self.get_next_run_id()
        
        # Calculate bearing angle (standard compass bearing)
        bearing = (90 - self.heading) % 360  # Convert to standard compass bearing
        
        log_data = {
            'timestamp': datetime.now().isoformat(),
            'run_id': self.run_id,
            'x': self.x,
            'y': self.y,
            'heading': self.heading,
            'bearing': bearing,  # Add standard bearing
            'compass_heading': compass_heading,
            'fix_quality': '3D Fix',  # Simulated fix quality
            'satellite_count': random.randint(8, 12),  # Simulated satellite count
            'deviation': (self.navigator.calculate_deviation(self.x, self.y)
                        if hasattr(self, 'navigator') and self.navigator else 0)
        }
        
        log_file_path = r'F:\GPS\task_2_waypoints\rover_log.csv'
        with open(log_file_path, 'a', newline='') as csvfile:
            # Add 'bearing' to fieldnames
            fieldnames = ['timestamp', 'run_id', 'x', 'y', 'heading', 'bearing', 'compass_heading', 
                        'fix_quality', 'satellite_count', 'deviation']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            if not os.path.exists(log_file_path) or os.path.getsize(log_file_path) == 0:
                writer.writeheader()
            writer.writerow(log_data)
        
        return True
    def get_compass_direction(self, heading):
        """
        Convert rover's heading (0° = East, 90° = North) to standard compass direction.
        Standard compass: North = 0°, East = 90°, South = 180°, West = 270°
        """
        # Convert from rover heading to standard compass bearing
        compass_heading = (90 - heading) % 360
        
        # Define standard compass bearings based on converted heading
        if 348.75 <= compass_heading or compass_heading < 11.25:
            return "North"
        elif 11.25 <= compass_heading < 33.75:
            return "North-Northeast"
        elif 33.75 <= compass_heading < 56.25:
            return "Northeast"
        elif 56.25 <= compass_heading < 78.75:
            return "East-Northeast"
        elif 78.75 <= compass_heading < 101.25:
            return "East"
        elif 101.25 <= compass_heading < 123.75:
            return "East-Southeast"
        elif 123.75 <= compass_heading < 146.25:
            return "Southeast"
        elif 146.25 <= compass_heading < 168.75:
            return "South-Southeast"
        elif 168.75 <= compass_heading < 191.25:
            return "South"
        elif 191.25 <= compass_heading < 213.75:
            return "South-Southwest"
        elif 213.75 <= compass_heading < 236.25:
            return "Southwest"
        elif 236.25 <= compass_heading < 258.75:
            return "West-Southwest"
        elif 258.75 <= compass_heading < 281.25:
            return "West"
        elif 281.25 <= compass_heading < 303.75:
            return "West-Northwest"
        elif 303.75 <= compass_heading < 326.25:
            return "Northwest"
        elif 326.25 <= compass_heading < 348.75:
            return "North-Northwest"
        else:
            return "Unknown" # Should never reach here
    def get_next_run_id(self):
        """Determine the next run ID based on existing data in the log file."""
        log_file_path = r'F:\GPS\task_2_waypoints\rover_log.csv'
        
        # If file doesn't exist, start with run 1
        if not os.path.exists(log_file_path):
            return 1
            
        try:
            # Read the existing file to find the highest run_id
            max_run_id = 0
            with open(log_file_path, 'r', newline='') as csvfile:
                reader = csv.DictReader(csvfile)
                for row in reader:
                    if 'run_id' in row:
                        try:
                            run_id = int(row['run_id'])
                            max_run_id = max(max_run_id, run_id)
                        except (ValueError, TypeError):
                            pass
            
            # Return the next run ID
            return max_run_id + 1
        except Exception as e:
            print(f"Error determining run ID: {e}")
            return 1  # Default to 1 if there's an error

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
        if self.failsafe.in_failsafe_mode:
            print("⚠️ Rover is in failsafe mode, cannot move forward.")
            return False
        
        rad = math.radians(self.heading)
        target_x = self.x + distance * math.cos(rad)
        target_y = self.y + distance * math.sin(rad)
        success = self.set_position(target_x, target_y)
        if success and ax and fig and rover_patch:
            update_rover_visualization(self, ax, fig, rover_patch)
            plt.pause(0.001)  # Add pause for smoother animation
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
    rover_scale = 3.5  # Scale factor to make rover appear larger

    if rover_patch is None:
        base_verts = np.array([[0.7, 0], [-0.3, 0.4], [-0.3, -0.4]])
        scaled_verts = base_verts * rover_scale  # Apply scaling
        rover_patch = Polygon(scaled_verts, closed=True, fc='blue', ec='black')  # Correct: uses scaled vertices
        ax.add_patch(rover_patch)
    tr = transforms.Affine2D().rotate_deg(rover.heading).translate(rover.x, rover.y)
    rover_patch.set_transform(tr + ax.transData)
    if hasattr(ax, 'path_line') and len(rover.history) > 1:
        ax.path_line.set_data(*zip(*rover.history))
    fig.canvas.draw_idle()
    plt.pause(0.001)
    return rover_patch

# Moved from farm_entry.py

def visualize_turn(rover, target_heading, ax, fig, rover_patch=None, rotation_speed_factor=4):
    """
    Turn the rover to face a new heading using the shortest possible rotation.
    Includes logging for turn diagnostics.
    """

    if rover.failsafe.in_failsafe_mode:
        print("⚠️ Rover is in failsafe mode, cannot turn.")
        return rover_patch
    # Normalize headings to [0, 360)
    current = rover.heading % 360
    target_heading = target_heading % 360
    
    # Calculate the absolute angle difference (shortest path)
    clockwise_diff = (target_heading - current) % 360
    counterclockwise_diff = (current - target_heading) % 360
    
    # Choose the smallest rotation direction
    # FIXED: Swapped direction labels to match standard compass directions
    if clockwise_diff <= counterclockwise_diff:
        angle_diff = clockwise_diff
        direction = "turn_left"  # Changed from "turn_right" because clockwise is left in standard compass
    else:
        angle_diff = -counterclockwise_diff
        direction = "turn_right"  # Changed from "turn_left" because counterclockwise is right in standard compass
    
    # Log turn details for debugging
    print(f"🔄 TURN: From {current:.1f}° to {target_heading:.1f}°, Direction: {direction}, Angle: {abs(angle_diff):.1f}°")
    
    # Skip small turns (threshold 5 degrees)
    if abs(angle_diff) < 5:
        rover.heading = target_heading
        rover.last_heading = target_heading
        print(f"✓ Skipped small turn (<5°), set heading to {target_heading:.1f}°")
        return update_rover_visualization(rover, ax, fig, rover_patch)
    
    # Log the turning command
    rover.log_movement(direction, angle=abs(angle_diff))
    
    # Calculate steps for smooth animation
    steps = max(3, min(int(abs(angle_diff) / 10), 18))
    step_ang = angle_diff / steps
    
    # Calculate pause duration
    pause_duration = min(0.01, 0.005 * (180 / max(1, abs(angle_diff)))) / rotation_speed_factor
    
    # Animate the turn
    for i in range(1, steps + 1):
        rover.heading = (current + step_ang * i) % 360
        rover_patch = update_rover_visualization(rover, ax, fig, rover_patch)
        plt.pause(pause_duration)
    
    # Ensure exact final heading
    rover.heading = target_heading % 360
    rover.last_heading = rover.heading
    
    return update_rover_visualization(rover, ax, fig, rover_patch)
class RowNavigator:
    def __init__(self, rover):
        self.rover = rover
        self.interpolated_path = []
        self.current_waypoint_index = 0
        self.waypoint_threshold = 0.3
        self.column_spacing = 1.5
        self.column_height = 15
        self.movement_speed = 2.0
        self.current_row = 0  # Track which row we're on
        self.zigzag_pattern = True  # Enable zigzag pattern by default
        self.rows_data = []  # Store information about each row

    def load_waypoints_from_csv(filename):
        """
        Load waypoints from a CSV file
        Returns a list of (x,y) tuples representing the waypoints path
        """
        
        
        if not os.path.exists(filename):
            print(f"⚠️ Waypoints file not found: {filename}")
            return []
            
        waypoints = []
        try:
            with open(filename, 'r') as csvfile:
                reader = csv.DictReader(csvfile)
                for row in reader:
                    x = float(row['x'])
                    y = float(row['y'])
                    waypoints.append((x, y))
                    
            print(f"✅ Loaded {len(waypoints)} waypoints from {filename}")
            return waypoints
        except Exception as e:
            print(f"❌ Error loading waypoints: {e}")
            return []
        

   
    def load_rows_from_csv(self, csv_filename):
        """
        Load waypoints from CSV file (with columns x, y, row_index) 
        and process them into rows_data and interpolated_path.
        """
        if not os.path.exists(csv_filename):
            print(f"⚠️ Waypoints file not found: {csv_filename}")
            return False

        raw_points = []   # will hold tuples (row_idx, x, y)
        xs, ys = [], []

        # 1) Read CSV and collect raw points plus min/max for normalization
        with open(csv_filename, 'r') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                idx = int(row['row_index'])
                x = float(row['x'])
                y = float(row['y'])
                raw_points.append((idx, x, y))
                xs.append(x)
                ys.append(y)

        if not raw_points:
            print("⚠️ No data in CSV.")
            return False

        # 2) Compute normalization offsets
        min_x, min_y = min(xs), min(ys)

        # 3) Build the full interpolated_path (normalized)
        self.interpolated_path = [
            (x - min_x, y - min_y)
            for (_, x, y) in raw_points
        ]

        # 4) Group by row_index in original order
        grouped = OrderedDict()
        for idx, x, y in raw_points:
            grouped.setdefault(idx, []).append((x - min_x, y - min_y))

        # 5) Build rows_data
        self.rows_data = []
        for idx, pts in grouped.items():
            start_pt = pts[0]
            end_pt   = pts[-1]
            direction = "↑ UP" if end_pt[1] > start_pt[1] else "↓ DOWN"
            self.rows_data.append({
                'index':     idx,
                'x_pos':     start_pt[0],
                'direction': direction,
                'start':     start_pt,
                'end':       end_pt
            })

        # 6) Print a clean plan
        print("\n📋 CSV-based Navigation Plan:")
        print(f"   Total waypoints: {len(self.interpolated_path)}")
        print(f"   Estimated rows: {len(self.rows_data)}")
        for row in self.rows_data:
            print(f"   Row {row['index']+1}: "
                  f"X-position {row['x_pos']:.2f}m, "
                  f"Direction {row['direction']}")

        return True
    

    # Add this method to your RowNavigator class
    def calculate_deviation(self, x, y):
        """Calculate how far the rover is from the current path segment."""
        if not self.interpolated_path or len(self.interpolated_path) < 2:
            return 0
            
        # Find current path segment (between current and next waypoint)
        current_index = min(self.current_waypoint_index, len(self.interpolated_path) - 2)
        next_index = current_index + 1
        
        if current_index < 0 or next_index >= len(self.interpolated_path):
            return 0
            
        p1 = self.interpolated_path[current_index]
        p2 = self.interpolated_path[next_index]
        
        # Calculate perpendicular distance to line segment
        # Line equation: Ax + By + C = 0
        A = p2[1] - p1[1]
        B = p1[0] - p2[0]
        C = p2[0]*p1[1] - p1[0]*p2[1]
        
        # Distance formula: |Ax + By + C| / sqrt(A² + B²)
        distance = abs(A*x + B*y + C) / math.sqrt(A*A + B*B) if (A*A + B*B) > 0 else 0
        
        return distance
        
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
            
            # Create row points with more intermediate points for smoother movement
            if going_up:
                # More points for smoother movement (was 5, now more)
                y_points = np.linspace(bottom_y, top_y, 8)  
                row_start = (x, bottom_y)
                row_end = (x, top_y)
            else:
                y_points = np.linspace(top_y, bottom_y, 8)
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
                
            # Add improved transition to next row if not the last row
            if i < num_strips - 1:
                next_x = start_x + (i + 1) * spacing
                transition_y = top_y if going_up else bottom_y
                
                # Add curved transition points between rows instead of just one point
                # This creates a smoother path for the rover to follow
                num_transition_points = 5  # More points for smoother curve
                for j in range(1, num_transition_points+1):
                    t = j / num_transition_points
                    # Create a slight curve for the transition
                    trans_x = x + t * (next_x - x)
                    trans_y = transition_y
                    self.interpolated_path.append((trans_x, trans_y))
                
        # Print row information
        print("\n📋 Row Navigation Plan:")
        for row in self.rows_data:
            print(f"   Row {row['index']+1}: X-position {row['x_pos']:.2f}m, Direction {row['direction']}")
            
        return self.interpolated_path

    def distance(self, p1, p2):
        return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

    def calculate_heading(self, p1, p2):
        """
        Calculate heading between two points with tolerance for small differences.
        Modified to handle position rounding precision.
        """
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        tolerance = 0.01  # Matches position rounding to 2 decimal places
        if abs(dx) < tolerance:
            print(f"📍 Vertical movement detected: dy={dy:.4f}, setting heading to {'90°' if dy > 0 else '270°'}")
            return 90.0 if dy > 0 else 270.0
        elif abs(dy) < tolerance:
            print(f"📍 Horizontal movement detected: dx={dx:.4f}, setting heading to {'0°' if dx > 0 else '180°'}")
            return 0.0 if dx > 0 else 180.0
        else:
            heading = math.degrees(math.atan2(dy, dx)) % 360
            print(f"📍 Diagonal movement: dx={dx:.4f}, dy={dy:.4f}, heading={heading:.1f}°")
            return heading
    def heading_difference(self, current, target):
        diff = (target - current + 540) % 360 - 180
        return diff

    
    def smooth_turn(self, target_heading, ax=None, fig=None, rover_patch=None):
        """
        Perform a smooth turn to the target heading.
        """
        heading_diff = self.heading_difference(self.rover.heading, target_heading)
        
        if abs(heading_diff) < 10:
            self.rover.heading = target_heading
            self.rover.last_heading = target_heading
            if ax and fig and rover_patch:
                rover_patch = update_rover_visualization(self.rover, ax, fig, rover_patch)
            print(f"✓ Skipped small turn (<10°), set heading to {target_heading:.1f}°")
            return rover_patch
        
        direction = "turn_right" if heading_diff > 0 else "turn_left"
        self.rover.log_movement(direction, angle=abs(heading_diff))
        
        steps = max(5, int(abs(heading_diff) / 30))
        angle_step = heading_diff / steps
        
        for i in range(steps):
            self.rover.heading = (self.rover.heading + angle_step) % 360
            if ax and fig and rover_patch:
                rover_patch = update_rover_visualization(self.rover, ax, fig, rover_patch)
                plt.pause(0.001)
                
        self.rover.heading = round(target_heading, 1)
        self.rover.last_heading = self.rover.heading
        
        if ax and fig and rover_patch:
            rover_patch = update_rover_visualization(self.rover, ax, fig, rover_patch)
        return rover_patch


    
    def move_precisely_to_point(self, target_point, ax=None, fig=None, rover_patch=None):
        """
        Move rover to target point with precision at constant speed.
        Includes logging for turn diagnostics.
        """
        max_attempts = 40
        attempts = 0
        last_time = time.time()  # Initialize time tracking
        
        print(f"🎯 Moving to point: ({target_point[0]:.2f}, {target_point[1]:.2f})")
        
        while attempts < max_attempts:
            if self.rover.failsafe.in_failsafe_mode:
                print("⚠️ Rover is in failsafe mode, stopping movement.")
                return False
            current_time = time.time()
            time_elapsed = current_time - last_time
            last_time = current_time
            
            current_pos = (self.rover.x, self.rover.y)
            dist_to_target = self.distance(current_pos, target_point)
            
            if attempts % 5 == 0:
                print(f"   Distance to target: {dist_to_target:.2f}m")
                
            if dist_to_target <= self.waypoint_threshold:
                print(f"✅ Reached target within {self.waypoint_threshold}m threshold")
                self.rover.set_position(target_point[0], target_point[1], force=True)
                if ax and fig and rover_patch:
                    rover_patch = update_rover_visualization(self.rover, ax, fig, rover_patch)
                return True
                
            desired_heading = self.calculate_heading(current_pos, target_point)
            
            # Only turn if heading difference is significant
            heading_diff = self.heading_difference(self.rover.heading, desired_heading)
            if abs(heading_diff) > 10:
                print(f"🧭 Initiating turn: Current {self.rover.heading:.1f}°, Desired {desired_heading:.1f}°, Diff {heading_diff:.1f}°")
                self.smooth_turn(desired_heading, ax, fig, rover_patch)
            else:
                print(f"✓ Heading diff {abs(heading_diff):.1f}° < 10°, no turn needed")
                
            # Calculate move_dist based on constant speed and elapsed time
            move_dist = self.movement_speed * time_elapsed
            if move_dist > dist_to_target:
                move_dist = dist_to_target  # Prevent overshooting
            
            
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
            plt.pause(0.001)
                
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
            """
            Align rover to the next waypoint.
            """
            next_point = self.determine_next_task()
            if not next_point:
                return False
                
            current_pos = (self.rover.x, self.rover.y)
            desired_heading = self.calculate_heading(current_pos, next_point)
            
            current_heading = self.rover.heading
            heading_diff = self.heading_difference(current_heading, desired_heading)
            
            if abs(heading_diff) < 10:
                print(f"✓ Already aligned to correct heading: {current_heading:.1f}°")
                return True
                
            print(f"🧭 Aligning from {current_heading:.1f}° to {desired_heading:.1f}°")
            
            if ax and fig:
                rover_patch = visualize_turn(self.rover, desired_heading, ax, fig, rover_patch, rotation_speed_factor=6)
            else:
                self.rover.heading = desired_heading
                self.rover.last_heading = desired_heading
                
            print(f"✓ Aligned to heading: {self.rover.heading:.1f}°")
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
        Navigate through all waypoints with improved turning and reporting.
        """
        # 1) Precompute total points
        total_pts = len(self.interpolated_path)
        print(f"🚜 COMMAND: Navigate zigzag path with {total_pts} points")

        if total_pts == 0:
            print("⚠️ No path generated - call generate_rows or load_rows first")
            return False
        if self.current_waypoint_index >= total_pts:
            print("⚠️ Navigation complete - already at end of path")
            return True

        # 2) Compute a row‐matching tolerance halfway between your actual row y‐positions
        # Rows are differentiated by their y-coordinates in zigzag pattern
        ys = sorted({pt[1] for pt in self.interpolated_path})
        unique_ys = []
        # Filter out very close y values (within 0.1)
        for y in ys:
            if not unique_ys or abs(y - unique_ys[-1]) > 0.1:
                unique_ys.append(y)
        
        if len(unique_ys) > 1:
            tol = min(abs(unique_ys[i+1] - unique_ys[i]) for i in range(len(unique_ys)-1)) / 2
        else:
            tol = self.column_spacing / 2

        # 3) Helper to find which row a point belongs to - using y-coordinate and REVERSE indexing
        def find_row(point):
            # Match point to row based on y-coordinate with REVERSED indexing
            for idx, row_y in enumerate(unique_ys):
                if abs(point[1] - row_y) <= tol:
                    # Return reversed index (if we have n rows, index should be n-idx-1)
                    return len(unique_ys) - idx - 1
            return len(unique_ys) - 1  # Default to last row if no match

        # 4) Log planned moves once, not in the loop
        current_pos = self.interpolated_path[self.current_waypoint_index]
        current_row_idx = find_row(current_pos)
        print(f"\n🌾 Starting in Row {current_row_idx+1}")
        
        # Set initial row direction
        if current_row_idx < len(self.rows_data):
            current_direction = self.rows_data[current_row_idx]['direction']
        else:
            # Infer direction from path if row data is incomplete
            next_pt = self.interpolated_path[self.current_waypoint_index + 1] if self.current_waypoint_index + 1 < total_pts else None
            if next_pt:
                if next_pt[0] > current_pos[0]:
                    current_direction = "→ East"
                elif next_pt[0] < current_pos[0]:
                    current_direction = "← West"
                else:
                    current_direction = "Unknown"
            else:
                current_direction = "Unknown"
        
        print(f"🧭 Current direction: {current_direction}")

        success = True
        last_command_point = None
        last_target_point = None  # Track the last target point to avoid duplicate commands

        # 6) Main loop
        while self.current_waypoint_index < total_pts - 1:
            if self.rover.failsafe.in_failsafe_mode:
                print("⚠️ Rover is in failsafe mode, stopping navigation.")
                return False
            next_idx = self.current_waypoint_index + 1
            next_pt = self.interpolated_path[next_idx]
            current_pt = self.interpolated_path[self.current_waypoint_index]
            
            # Only print movement command if it's a new target point (avoid duplicates)
            if next_pt != last_target_point:
                print(f"\n➡️ COMMAND: Move from ({current_pt[0]:.2f}, {current_pt[1]:.2f}) "
                    f"to ({next_pt[0]:.2f}, {next_pt[1]:.2f})")
                last_target_point = next_pt
                last_command_point = current_pt

            # Detect if row changes by comparing y-coordinates
            new_row_idx = find_row(next_pt)
            if new_row_idx != current_row_idx:
                print(f"\n🔄 Transitioning from Row {current_row_idx+1} to Row {new_row_idx+1}")
                
                # Get or infer new direction
                if new_row_idx < len(self.rows_data):
                    new_direction = self.rows_data[new_row_idx]['direction']
                else:
                    # Infer direction from next points in path
                    future_idx = next_idx + 1
                    if future_idx < total_pts:
                        future_pt = self.interpolated_path[future_idx]
                        if future_pt[0] > next_pt[0]:
                            new_direction = "→ East"
                        elif future_pt[0] < next_pt[0]:
                            new_direction = "← West"
                        else:
                            new_direction = "Unknown"
                    else:
                        new_direction = "Unknown"
                
                print(f"🧭 New row direction: {new_direction}")

                # Plan turn for the entire row
                if new_row_idx < len(self.rows_data):
                    start, end = self.rows_data[new_row_idx]['start'], self.rows_data[new_row_idx]['end']
                    if self.rows_data[new_row_idx]['direction'].startswith("↑") or self.rows_data[new_row_idx]['direction'].startswith("→"):
                        optimal = self.calculate_heading(start, end)
                    else:
                        optimal = self.calculate_heading(end, start)
                    print(f"🧭 Planning efficient turn from {self.rover.heading:.1f}° to {optimal:.1f}°")
                
                if not self.align_to_next_task(ax, fig, rover_patch):
                    print("⚠️ Failed to align for transition")
                    success = False
                    break

                current_row_idx = new_row_idx
                current_direction = new_direction
            else:
                # Only print "continuing in row" message when we're staying in the same row
                # and moving to a new target point (not repeatedly)
                if next_pt != last_target_point:
                    print(f"\n🌾 Continuing in Row {current_row_idx+1} moving {current_direction}")
                    
                # Check if we need to adjust heading within the row
                desired = self.calculate_heading((self.rover.x, self.rover.y), next_pt)
                if abs(self.heading_difference(self.rover.heading, desired)) > 3:
                    if not self.align_to_next_task(ax, fig, rover_patch):
                        print("⚠️ Failed to align in‐row")
                        success = False
                        break

            # Drive to the point
            if self.move_precisely_to_point(next_pt, ax, fig, rover_patch):
                self.current_waypoint_index = next_idx
                print(f"✅ Reached waypoint {self.current_waypoint_index+1} in Row {current_row_idx+1}")
            else:
                print(f"⚠️ Failed to reach waypoint {next_idx+1}")
                success = False
                break

            if ax and fig:
                plt.pause(0.05)

        # 7) Final summary if we made it
        if success and self.current_waypoint_index == total_pts - 1:
            print("\n✅ Successfully navigated entire path")
            final_row = find_row(self.interpolated_path[-1])
            print(f"🎉 Completed Row {final_row+1} – All rows navigated!")
            # ensure exact final positioning
            fx, fy = self.interpolated_path[-1]
            if self.rover.distance_to(fx, fy) > self.waypoint_threshold:
                print(f"📍 Final adjustment to ({fx:.2f}, {fy:.2f})...")
                self.rover.set_position(fx, fy, force=True)
                if ax and fig and rover_patch:
                    rover_patch = update_rover_visualization(self.rover, ax, fig, rover_patch)
                    plt.pause(0.5)

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


def normalize_coordinates(waypoints):
    """
    Transform coordinates from large values (like UTM) to simulation coordinate space
    """
    if not waypoints:
        return []
        
    # Find min values to use as origin
    min_x = min(point[0] for point in waypoints)
    min_y = min(point[1] for point in waypoints)
    
    # Normalize all points relative to this origin
    normalized = []
    for x, y in waypoints:
        normalized.append((x - min_x, y - min_y))
        
    print(f"✓ Normalized coordinates from ({min_x:.1f}, {min_y:.1f}) origin")
    return normalized
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
        if rover.failsafe.in_failsafe_mode:
            print("⚠️ Rover is in failsafe mode, stopping navigation.")
            return False, rover_patch
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
    PATH_STEP = 2.4  # Increased step size for faster movement (was 0.2)
    PATH_TOLERANCE = 0.05  # Small tolerance to enforce strict adherence
    ANIMATION_SPEED = 0.001  # Faster animation (was 0.01)
    ROTATION_STEP_FACTOR = 8  # Rotate faster
    
    # Start with current position
    start_idx = 0
    # Find closest waypoint if we're not already at the first one
    if rover.distance_to(*waypoints[0]) > PATH_TOLERANCE:
        closest_idx = 0
        min_dist = float('inf')
        for i, wp in enumerate(waypoints):
            if rover.failsafe.in_failsafe_mode:
                print("⚠️ Rover is in failsafe mode, stopping path following.")
                return False, rover_patch
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
    rover.navigator = row_navigator
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