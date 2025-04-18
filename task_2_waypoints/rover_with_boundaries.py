import math
import time
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon
import matplotlib.transforms as transforms

# --- Constants ---
STEP = 0.5  # Increased for faster navigation
TOLERANCE = 0.5  # Increased tolerance for reaching targets
MAX_ATTEMPTS = 200  # Increased maximum attempts
DEBUG = False
ANIMATION_SPEED = 0.05  # Rotation animation speed (lower is faster)

def get_float(prompt):
    """Get a float value from user with error handling"""
    while True:
        try:
            value = float(input(prompt))
            return value
        except ValueError:
            print("⚠️ Please enter a valid number.")

class Rover:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.history = []
        self.command_count = 0
        self.waypoint = None
        self.geofence = None
        self.inside_fence = False
        self.entry_point = None
        self.blocked_directions = set()
        self.stuck_count = 0  # Counter for detecting when rover is stuck
        self.last_position = (0, 0)
        self.position_history = []  # To detect cycles and stuck conditions
        self.rover_patch = None  # Visual representation of the rover
        self.fence_locked = False  # NEW: Track if rover is locked within the fence
        
    def set_position(self, x, y, heading=None, force=False, add_to_history=True):
        """Set rover position with optional heading"""
        if self.geofence and not force:
            # Check if movement would cross fence
            in_fence = self.is_point_in_polygon(x, y, self.geofence)
            
            # NEW: If locked inside fence, reject any attempt to leave
            if self.fence_locked and self.inside_fence and not in_fence:
                print(f"🔒 Movement blocked: Rover is locked inside the farm")
                return False
                
            # Handle normal fence crossing (for entering)
            if (in_fence and not self.inside_fence) or (not in_fence and self.inside_fence):
                # Would cross fence boundary without going through entry
                if not self.is_entry_point(x, y):
                    print(f"⚠️ Movement blocked: would cross fence boundary")
                    return False
                else:
                    # Crossing at entry point is OK
                    self.inside_fence = in_fence
                    # NEW: If entering, lock the rover inside
                    if in_fence:
                        self.fence_locked = True
                        print(f"🔒 Rover now locked inside farm boundaries")
            
        # Set new position
        self.x = x
        self.y = y
        if heading is not None:
            self.heading = heading % 360  # Normalize heading to 0-359
            
        # Add to history if tracking
        if add_to_history:
            self.history.append((x, y))
            
        return True
    
    def set_waypoint(self, x, y):
        """Set target waypoint"""
        self.waypoint = (x, y)
        
    def set_geofence(self, vertices, entry_point):
        """Set geofence polygon and entry point"""
        self.geofence = vertices
        self.entry_point = entry_point
        # Check if we're inside the fence initially
        self.inside_fence = self.is_point_in_polygon(self.x, self.y, vertices)
        
    def move_forward(self, distance, ax=None, fig=None, rover_patch=None):
        """Move rover forward in current heading direction"""
        if distance <= 0:
            print("⚠️ Invalid distance value <= 0")
            return False
            
        # Calculate target position
        rad = math.radians(self.heading)
        target_x = self.x + distance * math.cos(rad)
        target_y = self.y + distance * math.sin(rad)
        
        # For smoother animation of longer movements
        if ax and fig and rover_patch and distance > STEP:
            steps = min(int(distance / (STEP/2)), 5)  # Max 5 animation steps
            if steps > 1:
                step_x = (target_x - self.x) / steps
                step_y = (target_y - self.y) / steps
                
                for i in range(steps-1):
                    # Calculate intermediate position
                    next_x = self.x + step_x
                    next_y = self.y + step_y
                    
                    # Temporary update position (don't add to history yet)
                    success = self.set_position(next_x, next_y, add_to_history=False)
                    if not success:
                        return False
                        
                    # Update visualization
                    update_rover_visualization(self, ax, fig, rover_patch)
                    plt.pause(ANIMATION_SPEED/2)
        
        # Complete the movement (add to history)
        success = self.set_position(target_x, target_y)
        
        if success:
            self.command_count += 1
            print(f"Cmd #{self.command_count}: MOVE_FWD {distance:.2f}m → ({self.x:.3f}, {self.y:.3f})")
            
            # Display distance to waypoint and entry point
            if self.waypoint:
                waypoint_dist = self.distance_to(*self.waypoint)
                print(f"   📏 Distance to waypoint: {waypoint_dist:.2f}m")
            
            if self.entry_point and not self.inside_fence:
                entry_dist = self.distance_to(*self.entry_point)
                print(f"   📏 Distance to entry point: {entry_dist:.2f}m")
                
            return True
        else:
            # If movement was blocked, mark this general direction as blocked
            rounded_heading = int(self.heading / 10) * 10  # Round to nearest 10 degrees
            self.blocked_directions.add(rounded_heading)
            print(f"⚠️ Movement in direction {rounded_heading}° blocked")
            return False
    
    def calculate_heading_to(self, target_x, target_y):
        """Calculate heading angle to target point"""
        dx = target_x - self.x
        dy = target_y - self.y
        
        # Handle case where target is very close to current position
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            return self.heading  # Keep current heading if target is same as current position
        
        # Calculate angle in degrees, 0 = east, 90 = north
        angle = math.degrees(math.atan2(dy, dx))
        if angle < 0:
            angle += 360
            
        return angle
    
    def distance_to(self, target_x, target_y):
        """Calculate distance to target point"""
        return math.hypot(target_x - self.x, target_y - self.y)
    
    def is_point_in_polygon(self, x, y, vertices):
        """Check if point is inside polygon using ray casting algorithm"""
        n = len(vertices)
        inside = False
        
        p1x, p1y = vertices[0]
        for i in range(1, n + 1):
            p2x, p2y = vertices[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
            
        return inside
    
    def is_entry_point(self, x, y, tolerance=TOLERANCE):
        """Check if point is at the entry point (within tolerance)"""
        if self.entry_point:
            return self.distance_to(*self.entry_point) <= tolerance
        return False
    
    def detect_and_resolve_stuck(self):
        """Detect if rover is stuck and try to resolve it"""
        # Check if we've moved significantly in the last few commands
        current_pos = (self.x, self.y)
        
        # Add current position to history (keep last 10)
        self.position_history.append(current_pos)
        if len(self.position_history) > 10:
            self.position_history.pop(0)
            
        # If we have enough history, check for being stuck
        if len(self.position_history) >= 5:
            # Calculate max distance moved in recent history
            max_dist = 0
            for pos in self.position_history:
                dist = math.hypot(current_pos[0] - pos[0], current_pos[1] - pos[1])
                max_dist = max(max_dist, dist)
                
            # If maximum movement is very small, we might be stuck
            if max_dist < TOLERANCE/2:
                self.stuck_count += 1
                print(f"⚠️ Possible stuck condition detected ({self.stuck_count}/3)")
                
                if self.stuck_count >= 3:
                    print("🔄 Taking recovery action - making a significant turn")
                    # Clear blocked directions and try a major direction change
                    self.blocked_directions.clear()
                    self.stuck_count = 0
                    
                    # Choose a recovery heading based on recent attempts
                    recovery_angle = (self.heading + 120) % 360
                    return recovery_angle
            else:
                # Reset stuck counter if we're moving
                self.stuck_count = 0
                
        return None  # No stuck condition detected

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

def create_rover_patch():
    """Create a triangle patch to represent the rover"""
    # Create a triangle pointing to the right (0 degrees heading)
    rover_vertices = np.array([
        [0.7, 0],    # Nose
        [-0.3, 0.4], # Left wing
        [-0.3, -0.4] # Right wing
    ])
    return Polygon(rover_vertices, closed=True, fc='blue', ec='black')

def update_rover_visualization(rover, ax, fig, rover_patch=None):
    """Update the visualization of the rover on the plot"""
    if rover_patch is None:
        # First time - create the rover patch
        rover_patch = create_rover_patch()
        ax.add_patch(rover_patch)
    
    # Create a transform that rotates and positions the patch
    tr = transforms.Affine2D().rotate_deg(rover.heading).translate(rover.x, rover.y)
    rover_patch.set_transform(tr + ax.transData)
    
    # Update the path line if we have history
    if hasattr(ax, 'path_line') and len(rover.history) > 1:
        ax.path_line.set_data(*zip(*rover.history))
    
    # Update the plot
    fig.canvas.draw_idle()
    plt.pause(0.01)
    
    return rover_patch

def visualize_turn(rover, new_heading, ax, fig, rover_patch=None):
    """Visualize turning process with smooth animation"""
    current = rover.heading
    diff = (new_heading - current + 180) % 360 - 180
    
    # Skip visualization for tiny turns
    if abs(diff) < 5:
        rover.heading = new_heading
        return update_rover_visualization(rover, ax, fig, rover_patch)
    
    # Print turning command
    rover.command_count += 1
    print(f"Cmd #{rover.command_count}: ROTATE_TO {new_heading:.1f}° ({diff:.1f}° turn)")
    
    if rover.waypoint:
        waypoint_dist = rover.distance_to(*rover.waypoint)
        print(f"   📏 Distance to waypoint: {waypoint_dist:.2f}m")
    
    # More smooth steps for larger rotations
    steps = max(5, min(int(abs(diff) / 5), 36))  # Min 5, max 36 steps (10° increments for large turns)
    angle_step = diff / steps
    
    try:
        for i in range(1, steps+1):
            # Update heading
            rover.heading = (current + angle_step * i) % 360
            
            # Update visualization
            rover_patch = update_rover_visualization(rover, ax, fig, rover_patch)
            
            # Slight pause for animation
            plt.pause(ANIMATION_SPEED)
            
    except Exception as e:
        if DEBUG:
            print(f"Turn visualization error: {e}")
        rover.heading = new_heading
        
    return update_rover_visualization(rover, ax, fig, rover_patch)

def find_best_path_angle(rover, target_x, target_y, blocked_angles=None):
    """Find the best angle to move toward considering blocked paths"""
    direct_angle = rover.calculate_heading_to(target_x, target_y)
    
    # If direct path isn't blocked, use it
    if not blocked_angles or int(direct_angle / 10) * 10 not in blocked_angles:
        return direct_angle
    
    # Try angles in increasing offsets from the direct path
    for offset in range(10, 360, 10):  # Try full 360 degrees
        for direction in [1, -1]:  # Try both clockwise and counterclockwise
            test_angle = (direct_angle + direction * offset) % 360
            if int(test_angle / 10) * 10 not in blocked_angles:
                return test_angle
    
    # If all angles are blocked (unlikely), try random angles
    import random
    return random.randint(0, 359)

def navigate_to_point(rover, target_x, target_y, ax, fig, rover_patch=None, step_size=STEP, tolerance=TOLERANCE):
    """Navigate rover to a target point with improved handling of difficult paths"""
    print(f"\n🚗 Navigating to point ({target_x:.3f}, {target_y:.3f})...\n")
    dist = rover.distance_to(target_x, target_y)
    attempts = 0
    max_attempts = MAX_ATTEMPTS 
    consecutive_blocked = 0
    last_dist = float('inf')
    approach_changed = False
    
    # Adaptive step size based on distance
    adaptive_step = min(step_size, dist/2)

    while dist > tolerance and attempts < max_attempts:
        attempts += 1
        
        # Check for stuck condition and get recovery heading if needed
        recovery_heading = rover.detect_and_resolve_stuck()
        if recovery_heading is not None:
            rover_patch = visualize_turn(rover, recovery_heading, ax, fig, rover_patch)
            approach_changed = True
            continue
        
        # Periodic check if we're making progress
        if attempts % 5 == 0:
            # If we've made less than 5% progress, try a new approach
            if dist > last_dist * 0.95:
                if not approach_changed:
                    print("⚠️ Limited progress detected, trying alternative approach...")
                    rover.blocked_directions.clear()
                    # Try a completely different approach angle
                    alternative_angle = (rover.heading + 90 + attempts % 90) % 360
                    rover_patch = visualize_turn(rover, alternative_angle, ax, fig, rover_patch)
                    approach_changed = True
                    
                    # Try a larger step to break out of local minima
                    adaptive_step = min(step_size * 2, dist/2)
                else:
                    # If we already changed approach, try a more random direction
                    alternative_angle = (rover.heading + 180) % 360
                    rover_patch = visualize_turn(rover, alternative_angle, ax, fig, rover_patch)
                    
                    # Try a larger step to break out of local minima
                    adaptive_step = min(step_size * 3, dist/2)
            else:
                approach_changed = False
                adaptive_step = min(step_size, dist/2)  # Reset to normal step
                
            last_dist = dist
            
        # Find best heading considering blocked paths
        if consecutive_blocked > 2:
            tgt_heading = find_best_path_angle(rover, target_x, target_y, rover.blocked_directions)
            consecutive_blocked = 0
        else:
            tgt_heading = rover.calculate_heading_to(target_x, target_y)
        
        heading_diff = (tgt_heading - rover.heading + 180) % 360 - 180
        
        # If we need to adjust heading
        if abs(heading_diff) > 5:
            rover_patch = visualize_turn(rover, tgt_heading, ax, fig, rover_patch)
        
        # Try to move forward with adaptive step size
        step = min(adaptive_step, dist)
        success = rover.move_forward(step, ax, fig, rover_patch)
        
        # Update rover visualization
        rover_patch = update_rover_visualization(rover, ax, fig, rover_patch)
        
        # Update distance to target
        dist = rover.distance_to(target_x, target_y)
        
        # Handle blocked movement
        if not success:
            consecutive_blocked += 1
            if consecutive_blocked >= 2:
                # Try increasingly extreme direction changes
                angle_change = 45 + (consecutive_blocked * 15)
                if angle_change > 180:
                    angle_change = 180
                    
                new_heading = (rover.heading + angle_change) % 360
                rover_patch = visualize_turn(rover, new_heading, ax, fig, rover_patch)
        else:
            consecutive_blocked = 0
    
    if dist <= tolerance:
        print(f"✅ Reached target point ({rover.x:.3f}, {rover.y:.3f})")
        return True, rover_patch
    else:
        # If we failed, try one final direct approach with larger step
        print("🔄 Making final approach attempt with larger step size...")
        
        # Set heading directly to target
        direct_heading = rover.calculate_heading_to(target_x, target_y)
        rover_patch = visualize_turn(rover, direct_heading, ax, fig, rover_patch)
        
        # Try a direct move with larger step
        rover.move_forward(dist * 0.9, ax, fig, rover_patch)
        
        # Check if we're now close enough
        final_dist = rover.distance_to(target_x, target_y)
        if final_dist <= tolerance * 1.5:  # Allow slightly larger tolerance for final check
            print(f"✅ Reached target point on final attempt ({rover.x:.3f}, {rover.y:.3f})")
            return True, rover_patch
        else:
            print(f"⚠️ Could not reach target point. Current position: ({rover.x:.3f}, {rover.y:.3f})")
            print(f"   Distance to target: {final_dist:.3f}")
            return False, rover_patch

# --- MAIN ---
def main():
    # Enable better error handling for matplotlib
    plt.rcParams['figure.max_open_warning'] = 50
    
    rover = Rover()

    # 1) Define rectangular farm boundary
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

    # Define entry point (always use first vertex - bottom left corner)
    entry_point = geofence_vertices[0]
    print(f"🚪 Entry point set to bottom-left corner: ({entry_point[0]:.2f}, {entry_point[1]:.2f})")

    # Set the geofence for the rover
    rover.set_geofence(geofence_vertices, entry_point)

    # 2) User inputs for positioning and waypoint
    print("🔧 Enter starting position:")
    
    # CHANGE: Keep asking for valid starting position until one outside farm is provided
    while True:
        x1 = get_float(" x1: ")
        y1 = get_float(" y1: ")
        
        if rover.is_point_in_polygon(x1, y1, geofence_vertices):
            print("⚠️ Starting point must be outside the farm. Please enter new coordinates.")
        else:
            break
            
    print(f"✅ Valid starting position: ({x1:.2f}, {y1:.2f})")

    print("🔧 Enter waypoint coords:")
    
    # CHANGE: Keep asking for valid waypoint until one inside farm is provided
    while True:
        wx = get_float(" wx: ")
        wy = get_float(" wy: ")
        
        if not rover.is_point_in_polygon(wx, wy, geofence_vertices):
            print("⚠️ Waypoint must be inside the farm. Please enter new coordinates.")
        else:
            break
            
    print(f"✅ Valid waypoint: ({wx:.2f}, {wy:.2f})")

    rover.set_waypoint(wx, wy)

    # 3) Plot setup with error handling
    try:
        plt.ion()
        fig, ax = plt.subplots(figsize=(10, 8))

        # Calculate plot boundaries
        all_x = [v[0] for v in geofence_vertices] + [x1, wx, entry_point[0]]
        all_y = [v[1] for v in geofence_vertices] + [y1, wy, entry_point[1]]
        margin = 2.0  # Larger margin

        mx, Mx = min(all_x) - margin, max(all_x) + margin
        my, My = min(all_y) - margin, max(all_y) + margin

        ax.set_xlim(mx, Mx)
        ax.set_ylim(my, My)
        ax.grid(True)
        ax.set_title("Rover Farm Navigation Simulation")

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
        ax.path_line = path_line  # Store reference to path line
        ax.legend(loc='upper left')
        fig.canvas.draw_idle()
        plt.pause(0.5)
    except Exception as e:
        print(f"Error in plot setup: {e}")
        print("Continuing with limited visualization...")
        # Create minimal plot
        fig, ax = plt.subplots(figsize=(8, 6))
        path_line, = ax.plot([], [], 'b-')
        ax.path_line = path_line
        ax.set_title("Rover Navigation (Limited View)")
        fig.canvas.draw_idle()

    # 4) Set initial position
    rover.set_position(x1, y1, force=True, add_to_history=False)
    rover.history.append((rover.x, rover.y))
    
    # Create initial rover visualization
    rover_patch = update_rover_visualization(rover, ax, fig)
    
    # 5) Navigate to the farm entry point
    print("\n🚜 Moving rover from outside farm to entry point...\n")
    print(f"📏 Initial distance to entry point: {rover.distance_to(*entry_point):.2f}m")
    
    # Try entry point navigation with increasing max attempts and step sizes
    for attempt in range(1, 4):
        print(f"\n🔄 Entry point navigation attempt {attempt}/3...")
        entry_reached, rover_patch = navigate_to_point(
            rover, entry_point[0], entry_point[1], ax, fig, rover_patch,
            step_size=STEP * attempt, tolerance=TOLERANCE
        )
        
        if entry_reached:
            break
            
        if attempt < 3:
            # Reset blocked directions and try again
            rover.blocked_directions.clear() 
            print("🔄 Retrying entry point navigation with new parameters...")
    
    if entry_reached:
        try:
            # Add marker for entry confirmation
            ax.scatter(rover.x, rover.y, c='cyan', s=80, marker='^', label='Entry Reached')
            ax.legend(loc='upper left')
            fig.canvas.draw_idle()
            plt.pause(1.0)
        except Exception as e:
            if DEBUG:
                print(f"Visualization error: {e}")
        
        # Clear blocked directions before starting next navigation
        rover.blocked_directions.clear()
        rover.position_history.clear()
        
        # 6) Navigate to the waypoint inside the farm
        print("\n🚜 Moving rover to waypoint inside farm...\n")
        print(f"📏 Initial distance to waypoint: {rover.distance_to(*rover.waypoint):.2f}m")
        
        # Try waypoint navigation with increasing max attempts and step sizes
        for attempt in range(1, 4):
            print(f"\n🔄 Waypoint navigation attempt {attempt}/3...")
            waypoint_reached, rover_patch = navigate_to_point(
                rover, wx, wy, ax, fig, rover_patch,
                step_size=STEP * attempt, tolerance=TOLERANCE
            )
            
            if waypoint_reached:
                break
                
            if attempt < 3:
                # Reset blocked directions and try again
                rover.blocked_directions.clear()
                print("🔄 Retrying waypoint navigation with new parameters...")
        
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
            print("\n⚠️ Could not reach waypoint after multiple attempts.")
            print("   Consider adjusting sim parameters or waypoint location.")
    else:
        print("\n⚠️ Could not reach farm entry point after multiple attempts.")
        print("   Consider adjusting sim parameters or entry point location.")

    # Final plot display
    try:
        plt.ioff()
        plt.title("Rover Farm Navigation Simulation")
        plt.show(block=True)
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