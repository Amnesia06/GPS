import math
import time
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon

# --- Constants ---
STEP = 0.2
TOLERANCE = 0.3
MAX_ATTEMPTS = 100
DEBUG = False  # Set to True for detailed error messages

class Rover:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.history = []  # Empty history initially - only add after positioning
        self.command_count = 0
        self.waypoint = None
        self.geofence = None
        self.inside_fence = False
        self.entry_point = None
        self.blocked_directions = set()

    def set_geofence(self, vertices, entry_point=None):
        """Set the geofence boundary using a list of (x,y) vertices"""
        self.geofence = vertices
        # If entry point is not specified, use the first vertex
        self.entry_point = entry_point if entry_point else vertices[0]
        print("🔒 Geofence boundary set")
        print(f"🚪 Entry point set at: ({self.entry_point[0]:.3f}, {self.entry_point[1]:.3f})")
        # Check if current position is inside the geofence
        self.update_fence_status()
        
    def update_fence_status(self):
        """Check if rover is inside the geofence"""
        old_status = self.inside_fence
        if self.geofence is None:
            self.inside_fence = True
        else:
            self.inside_fence = self.is_point_in_polygon(self.x, self.y, self.geofence)
        
        if old_status != self.inside_fence:
            if self.inside_fence:
                print("✅ Rover ENTERED geofenced area!")
            else:
                print("⚠️ Rover EXITED geofenced area!")
        
        return self.inside_fence
    
    def is_point_in_polygon(self, x, y, polygon):
        """Ray casting algorithm to determine if point is in polygon"""
        n = len(polygon)
        inside = False
        p1x, p1y = polygon[0]
        for i in range(1, n + 1):
            p2x, p2y = polygon[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        return inside

    def set_position(self, x, y, force=False, add_to_history=True):
        self.command_count += 1
        old_x, old_y = self.x, self.y
        
        # Store the proposed new position
        proposed_x, proposed_y = x, y
        
        # Check if the move would take us outside the geofence
        if not force and self.geofence is not None and self.inside_fence:
            if not self.is_point_in_polygon(proposed_x, proposed_y, self.geofence):
                print(f"❌ Command #{self.command_count}: SET_POSITION - BLOCKED by geofence")
                print(f"⚠️ Position ({proposed_x:.3f}, {proposed_y:.3f}) is outside the geofence")
                return False
        
        # Update position if allowed
        self.x, self.y = proposed_x, proposed_y
        if add_to_history:
            self.history.append((self.x, self.y))
        print(f"Command #{self.command_count}: SET_POSITION")
        print(f"➡️ Rover position: ({self.x:.3f}, {self.y:.3f})")
        self.update_fence_status()
        
        if self.waypoint:
            self.report_status()
            
        return True

    def move_forward(self, distance):
        self.command_count += 1
        rad = math.radians(self.heading)
        old_x, old_y = self.x, self.y
        proposed_x = self.x + distance * math.cos(rad)
        proposed_y = self.y + distance * math.sin(rad)
        
        # Check if the move would take us outside the geofence
        if self.geofence is not None and self.inside_fence:
            if not self.is_point_in_polygon(proposed_x, proposed_y, self.geofence):
                print(f"❌ Command #{self.command_count}: MOVE_FORWARD {distance:.2f} - BLOCKED by geofence")
                print(f"⚠️ Position ({proposed_x:.3f}, {proposed_y:.3f}) would be outside the geofence")
                
                # Store the blocked direction to avoid repeatedly trying it
                self.blocked_directions.add(int(self.heading / 10) * 10)
                
                # Calculate how far we can safely move
                step = distance / 10
                for i in range(1, 10):
                    test_x = self.x + i * step * math.cos(rad)
                    test_y = self.y + i * step * math.sin(rad)
                    if not self.is_point_in_polygon(test_x, test_y, self.geofence):
                        safe_distance = (i-1) * step
                        if safe_distance > 0:
                            print(f"🛑 Moving only {safe_distance:.2f} units to stay within bounds")
                            self.x += safe_distance * math.cos(rad)
                            self.y += safe_distance * math.sin(rad)
                            self.history.append((self.x, self.y))
                        return False
                return False
        
        # Update position if allowed
        self.x, self.y = proposed_x, proposed_y
        self.history.append((self.x, self.y))
        print(f"Command #{self.command_count}: MOVE_FORWARD {distance:.2f}")
        print(f"➡️ Rover moved from ({old_x:.3f}, {old_y:.3f}) to ({self.x:.3f}, {self.y:.3f})")
        self.update_fence_status()
        
        if self.waypoint:
            self.report_status()
            
        return True

    def rotate_by(self, angle_deg):
        self.command_count += 1
        old_heading = self.heading
        self.heading = (self.heading + angle_deg) % 360
        print(f"Command #{self.command_count}: ROTATE_BY {angle_deg:.1f}°")
        print(f"🔄 Rover rotated from {old_heading:.1f}° to {self.heading:.1f}°")
        if self.waypoint:
            self.report_status()
        return True

    def calculate_heading_to(self, tx, ty):
        dx, dy = tx - self.x, ty - self.y
        return math.degrees(math.atan2(dy, dx)) % 360

    def distance_to(self, tx, ty):
        return math.hypot(tx - self.x, ty - self.y)

    def set_waypoint(self, x, y):
        self.waypoint = (x, y)
        self.blocked_directions.clear()  # Clear blocked directions when setting new waypoint

    def report_status(self):
        if not self.waypoint:
            print("\n--- STATUS REPORT ---")
            print(f"📍 Position: ({self.x:.3f}, {self.y:.3f})")
            print(f"🧭 Heading: {self.heading:.1f}°")
            print("🎯 No waypoint set")
            print(f"🔒 Inside geofence: {'Yes' if self.inside_fence else 'No'}")
            print("---------------------\n")
            return None, None
        dist = self.distance_to(*self.waypoint)
        desired = self.calculate_heading_to(*self.waypoint)
        diff = (desired - self.heading + 180) % 360 - 180
        print("\n--- STATUS REPORT ---")
        print(f"📍 Position: ({self.x:.3f}, {self.y:.3f})")
        print(f"🧭 Heading: {self.heading:.1f}°")
        print(f"🎯 Distance to waypoint: {dist:.3f}")
        print(f"🔄 Angle adj needed: {diff:.1f}°")
        print(f"🔒 Inside geofence: {'Yes' if self.inside_fence else 'No'}")
        print("---------------------\n")
        return dist, diff

def safe_remove(element):
    """Safely remove a matplotlib element if it exists"""
    if element:
        try:
            element.remove()
            return True
        except (ValueError, AttributeError) as e:
            if DEBUG:
                print(f"Warning: Failed to remove element: {e}")
            return False
    return False

def visualize_turn(rover, target_heading, ax, fig):
    """Visualize rover turning with error handling"""
    curr = rover.heading
    diff = (target_heading - curr + 180) % 360 - 180
    if abs(diff) < 5:
        return
    
    steps = max(5, min(36, abs(int(diff / 10))))
    step_ang = diff / steps

    rad_t = math.radians(target_heading)
    
    # Initialize visualization elements as None
    targ = None
    note = None
    arrow = None
    
    try:
        # Create target direction arrow
        targ = ax.arrow(rover.x, rover.y,
                      STEP * math.cos(rad_t),
                      STEP * math.sin(rad_t),
                      head_width=0.1, fc='red', ec='red', alpha=0.7)
        
        # Create annotation
        note = ax.annotate(f"Turning {abs(diff):.1f}°",
                         xy=(rover.x, rover.y),
                         xytext=(rover.x+0.5, rover.y+0.5),
                         arrowprops=dict(facecolor='black', shrink=0.05),
                         fontsize=9)

        # Animate the turn
        for i in range(steps+1):
            safe_remove(arrow)  # Remove previous arrow safely
            
            ang = (curr + i*step_ang) % 360
            rad = math.radians(ang)
            
            arrow = ax.arrow(rover.x, rover.y,
                           STEP * math.cos(rad),
                           STEP * math.sin(rad),
                           head_width=0.1, fc='blue', ec='blue')
            
            # Update display
            try:
                fig.canvas.draw_idle()
                plt.pause(0.05)
            except Exception as e:
                if DEBUG:
                    print(f"Warning: Display update error: {e}")
                break
    
    except Exception as e:
        if DEBUG:
            print(f"Warning: Visualization error: {e}")
    
    finally:
        # Clean up all visualization elements
        safe_remove(arrow)
        safe_remove(targ)
        safe_remove(note)
        
        # Update rover heading even if visualization fails
        rover.heading = target_heading

def find_best_path_angle(rover, target_x, target_y, blocked_angles=None):
    """Find the best angle to move toward considering blocked paths"""
    direct_angle = rover.calculate_heading_to(target_x, target_y)
    
    # If no angles are blocked or direct path isn't blocked, use it
    if not blocked_angles or int(direct_angle / 10) * 10 not in blocked_angles:
        return direct_angle
    
    # Try angles in increasing offsets from the direct path
    for offset in range(10, 180, 10):
        for direction in [1, -1]:  # Try both clockwise and counterclockwise
            test_angle = (direct_angle + direction * offset) % 360
            if int(test_angle / 10) * 10 not in blocked_angles:
                return test_angle
    
    # If all reasonable angles are blocked, just return a random unblocked angle
    for angle in range(0, 360, 10):
        if angle not in blocked_angles:
            return angle
    
    # If everything is blocked (unlikely), return the direct angle
    return direct_angle

def navigate_to_point(rover, target_x, target_y, ax, fig, path_line, step_size=STEP, tolerance=TOLERANCE):
    """Navigate rover to a target point with visualization"""
    print(f"\n🚗 Navigating to point ({target_x:.3f}, {target_y:.3f})...\n")
    dist = rover.distance_to(target_x, target_y)
    arrow = None
    attempts = 0
    max_attempts = MAX_ATTEMPTS  # Prevent infinite loops
    consecutive_blocked = 0
    last_dist = float('inf')  # Track if we're making progress

    while dist > tolerance and attempts < max_attempts:
        attempts += 1
        
        # Check if we're making progress
        if attempts % 10 == 0:
            if dist > last_dist * 0.95:  # If we've reduced distance by less than 5%
                print("⚠️ Limited progress detected, trying alternative approach...")
                rover.blocked_directions.clear()  # Reset blocked directions
                # Try a different approach angle
                alternative_angle = (rover.heading + 90) % 360
                visualize_turn(rover, alternative_angle, ax, fig)
                rover.heading = alternative_angle
            last_dist = dist
        
        # Find best heading considering blocked paths
        if consecutive_blocked > 3:
            tgt_heading = find_best_path_angle(rover, target_x, target_y, rover.blocked_directions)
            consecutive_blocked = 0  # Reset counter
        else:
            tgt_heading = rover.calculate_heading_to(target_x, target_y)
        
        heading_diff = (tgt_heading - rover.heading + 180) % 360 - 180
        
        # If we need to adjust heading by more than 5 degrees
        if abs(heading_diff) > 5:
            visualize_turn(rover, tgt_heading, ax, fig)
            rover.heading = tgt_heading
            rover.command_count += 1
            print(f"Cmd #{rover.command_count}: ROTATE_BY {heading_diff:.1f}° → {rover.heading:.1f}°")
            safe_remove(arrow)  # Remove previous arrow safely
        
        # Try to move forward
        step = min(step_size, dist)
        success = rover.move_forward(step)
        
        # Update visualization
        dist = rover.distance_to(target_x, target_y)
        
        try:
            # Update path line
            if len(rover.history) > 0:
                path_line.set_data(*zip(*rover.history))
            
            safe_remove(arrow)  # Remove previous arrow safely
            
            # Draw direction arrow
            rad = math.radians(rover.heading)
            arrow = ax.arrow(rover.x, rover.y,
                         step_size * math.cos(rad),
                         step_size * math.sin(rad),
                         head_width=0.1, fc='blue', ec='blue')
            
            # Update display
            fig.canvas.draw_idle()
            plt.pause(0.05)
        except Exception as e:
            if DEBUG:
                print(f"Warning: Visualization update error: {e}")
        
        # If movement was blocked, track it and try to navigate around
        if not success:
            consecutive_blocked += 1
            if consecutive_blocked >= 3:
                print("🔄 Multiple blockages detected, trying a more significant direction change...")
                # Try a more significant turn to find a path
                new_heading = (rover.heading + 45 + attempts % 4 * 15) % 360
                visualize_turn(rover, new_heading, ax, fig)
                rover.heading = new_heading
        else:
            consecutive_blocked = 0  # Reset counter on successful movement
    
    # Clean up the last arrow
    safe_remove(arrow)
    
    if dist <= tolerance:
        print(f"✅ Reached target point ({rover.x:.3f}, {rover.y:.3f})")
        return True
    else:
        print(f"⚠️ Could not reach target point. Current position: ({rover.x:.3f}, {rover.y:.3f})")
        print(f"   Distance to target: {dist:.3f}")
        return False

# --- Input Helper Functions ---
def get_float(prompt):
    while True:
        try:
            return float(input(prompt))
        except ValueError:
            print("❌ Invalid number. Please try again.")

def get_int(prompt):
    while True:
        try:
            return int(input(prompt))
        except ValueError:
            print("❌ Invalid number. Please try again.")

def get_bool(prompt):
    while True:
        response = input(prompt).lower()
        if response in ["y", "yes"]:
            return True
        elif response in ["n", "no"]:
            return False
        else:
            print("❌ Please enter Y/N.")

# --- MAIN ---
def main():
    # Enable better error handling for matplotlib
    plt.rcParams['figure.max_open_warning'] = 50
    
    rover = Rover()

    # 1) Define geofence
    print("🔧 Define geofence (farm boundary):")
    print("   Choose: 1) Rectangle  2) Custom polygon")
    fence_type = get_int("Choice (1/2): ")

    if fence_type == 1:
        print("🔧 Enter farm rectangle coordinates:")
        min_x = get_float(" Min X: ")
        max_x = get_float(" Max X: ")
        min_y = get_float(" Min Y: ")
        max_y = get_float(" Max Y: ")
        
        # Create rectangle vertices in clockwise order
        geofence_vertices = [
            (min_x, min_y),  # Bottom-left
            (max_x, min_y),  # Bottom-right
            (max_x, max_y),  # Top-right
            (min_x, max_y)   # Top-left
        ]
    else:
        print("🔧 Enter number of fence vertices:")
        num_vertices = get_int(" Number: ")
        
        geofence_vertices = []
        print("🔧 Enter each vertex coordinate:")
        for i in range(num_vertices):
            x = get_float(f" Vertex {i+1} x: ")
            y = get_float(f" Vertex {i+1} y: ")
            geofence_vertices.append((x, y))

    # Define entry point
    print("🚪 Set farm entry point:")
    print("   1) Use first vertex  2) Custom entry point")
    entry_choice = get_int("Choice (1/2): ")

    if entry_choice == 1:
        entry_point = geofence_vertices[0]
    else:
        print("🔧 Enter entry point coordinates:")
        entry_x = get_float(" Entry X: ")
        entry_y = get_float(" Entry Y: ")
        entry_point = (entry_x, entry_y)

    # Set the geofence for the rover
    rover.set_geofence(geofence_vertices, entry_point)

    # 2) User inputs for positioning and waypoint
    print("🔧 Enter starting position (outside the farm):")
    x1 = get_float(" x1: ")
    y1 = get_float(" y1: ")

    # Ensure starting point is outside the farm
    if rover.is_point_in_polygon(x1, y1, geofence_vertices):
        print("⚠️ Starting point is inside the farm. Try again with a point outside.")
        outside_farm = False
        while not outside_farm:
            x1 = get_float(" New x1 (outside farm): ")
            y1 = get_float(" New y1 (outside farm): ")
            outside_farm = not rover.is_point_in_polygon(x1, y1, geofence_vertices)
            if outside_farm:
                print("✅ Starting point is outside the farm.")
            else:
                print("⚠️ Still inside the farm. Try again.")

    print("🔧 Enter waypoint coords (inside the farm):")
    wx = get_float(" wx: ")
    wy = get_float(" wy: ")

    # Ensure waypoint is inside the farm
    if not rover.is_point_in_polygon(wx, wy, geofence_vertices):
        print("⚠️ Waypoint is outside the farm. Try again with a point inside.")
        inside_farm = False
        while not inside_farm:
            wx = get_float(" New wx (inside farm): ")
            wy = get_float(" New wy (inside farm): ")
            inside_farm = rover.is_point_in_polygon(wx, wy, geofence_vertices)
            if inside_farm:
                print("✅ Waypoint is inside the farm.")
            else:
                print("⚠️ Still outside the farm. Try again.")

    rover.set_waypoint(wx, wy)

    # 3) Plot setup with error handling
    try:
        plt.ion()
        fig, ax = plt.subplots(figsize=(10, 8))

        # Calculate plot boundaries based on all points
        all_x = [v[0] for v in geofence_vertices] + [x1, wx, entry_point[0]]
        all_y = [v[1] for v in geofence_vertices] + [y1, wy, entry_point[1]]
        margin = 1.5  # Add some margin

        mx, Mx = min(all_x) - margin, max(all_x) + margin
        my, My = min(all_y) - margin, max(all_y) + margin

        ax.set_xlim(mx, Mx)
        ax.set_ylim(my, My)
        ax.grid(True)

        # Draw geofence
        geofence_array = np.array(geofence_vertices)
        fence_polygon = Polygon(geofence_array, closed=True, fill=True, 
                              facecolor='lightgreen', edgecolor='darkgreen', 
                              alpha=0.3, label='Farm Boundary')
        ax.add_patch(fence_polygon)

        # Add entry point marker
        ax.scatter(entry_point[0], entry_point[1], c='purple', s=100, marker='o', label='Farm Entry')

        # Add start and waypoint markers
        ax.scatter(x1, y1, c='green', s=80, label='Start (Outside)')
        ax.scatter(wx, wy, c='red', s=120, marker='*', label='Waypoint')

        # Path line for rover's movement history
        path_line, = ax.plot([], [], 'b-', alpha=0.5, label='Path')
        ax.legend(loc='upper left')
        fig.canvas.draw_idle()
        plt.pause(0.5)  # Longer pause to show initial setup
    except Exception as e:
        print(f"Error in plot setup: {e}")
        print("Continuing with limited visualization...")
        # Create minimal plot for path tracking
        fig, ax = plt.subplots(figsize=(8, 6))
        path_line, = ax.plot([], [], 'b-')
        ax.set_title("Rover Navigation (Limited View)")
        fig.canvas.draw_idle()

    # 4) Set initial position without adding to history
    rover.set_position(x1, y1, force=True, add_to_history=False)  # Don't add to history yet
    
    # 5) First, navigate to the farm entry point
    print("\n🚜 Moving rover from outside farm to entry point...\n")
    
    # Now add the starting position to history - this is the first point in the path
    rover.history.append((rover.x, rover.y))
    
    try:
        path_line.set_data(*zip(*rover.history))
        fig.canvas.draw_idle()
        plt.pause(0.5)
    except Exception as e:
        if DEBUG:
            print(f"Visualization error: {e}")
    
    entry_reached = navigate_to_point(rover, entry_point[0], entry_point[1], ax, fig, path_line)

    if entry_reached:
        try:
            # Add marker for entry confirmation
            ax.scatter(rover.x, rover.y, c='cyan', s=80, marker='^', label='Entry Reached')
            ax.legend(loc='upper left')
            fig.canvas.draw_idle()
            plt.pause(1.0)  # Longer pause to highlight entry
        except Exception as e:
            if DEBUG:
                print(f"Visualization error: {e}")
        
        # Clear blocked directions before starting next navigation
        rover.blocked_directions.clear()
        
        # 6) Now navigate to the waypoint inside the farm
        print("\n🚜 Moving rover to waypoint inside farm...\n")
        waypoint_reached = navigate_to_point(rover, wx, wy, ax, fig, path_line)
        
        if waypoint_reached:
            try:
                # Add marker for waypoint confirmation
                ax.scatter(rover.x, rover.y, c='magenta', s=100, marker='o', label='Final Position')
                ax.legend(loc='upper left')
                fig.canvas.draw_idle()
                plt.pause(0.5)
            except Exception as e:
                if DEBUG:
                    print(f"Visualization error: {e}")
                    
            print(f"\n✅ Mission complete! Reached waypoint in {rover.command_count} commands.")
        else:
            print("\n⚠️ Could not reach waypoint.")
    else:
        print("\n⚠️ Could not reach farm entry point.")

    # Final plot display
    try:
        plt.ioff()
        plt.title("Rover Farm Navigation Simulation")
        plt.show()
    except Exception as e:
        print(f"Error in final plot display: {e}")
        print("Simulation completed without final visualization.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n🛑 Simulation terminated by user.")
    except Exception as e:
        print(f"\n❌ Simulation error: {e}")
        if DEBUG:
            import traceback
            traceback.print_exc()