import matplotlib.pyplot as plt
import numpy as np
import time
import math
import random
from matplotlib.patches import Polygon, Circle

# Import the FailSafeModule
from fail_safe import FailSafeModule

class SimpleRover:
    """A simplified rover class for testing the fail-safe module."""
    def __init__(self, x=0, y=0, heading=0):
        self.x = x
        self.y = y
        self.heading = heading
        self.history = [(x, y)]
        self.speed = 0.5  # meters per command
        self.command_count = 0
        self.blocked_directions = set()  # Directions that are blocked
        
    def set_position(self, x, y, force=False, add_to_history=True):
        """Set the rover's position."""
        self.x = x
        self.y = y
        if add_to_history:
            self.history.append((x, y))
            
    def move_forward(self, distance, ax=None, fig=None, rover_patch=None):
        """Move the rover forward by the specified distance."""
        # Convert heading to radians
        heading_rad = math.radians(self.heading)
        
        # Calculate new position
        new_x = self.x + distance * math.cos(heading_rad)
        new_y = self.y + distance * math.sin(heading_rad)
        
        # Set the new position
        self.set_position(new_x, new_y)
        self.command_count += 1
        
        # Update visualization if provided
        if ax and fig and rover_patch:
            update_visualization(self, ax, fig, rover_patch)
        
        return True
        
    def calculate_heading_to(self, target_x, target_y):
        """Calculate the heading to a target point."""
        dx = target_x - self.x
        dy = target_y - self.y
        
        # Calculate angle in degrees
        angle = math.degrees(math.atan2(dy, dx))
        
        # Ensure angle is between 0 and 360
        if angle < 0:
            angle += 360
            
        return angle
        
    def distance_to(self, target_x, target_y):
        """Calculate distance to target point."""
        return math.hypot(target_x - self.x, target_y - self.y)
        
    def log_movement(self, action, **kwargs):
        """Log a movement action. Accept any additional keyword arguments."""
        print(f"ROVER ACTION: {action}")

def update_visualization(rover, ax, fig, rover_patch=None):
    """Update the rover visualization."""
    # Clear previous rover patch if it exists
    if rover_patch:
        try:
            rover_patch.remove()
        except:
            pass
    
    # Draw rover as a triangle pointing in the heading direction
    heading_rad = math.radians(rover.heading)
    dx = 0.3 * math.cos(heading_rad)
    dy = 0.3 * math.sin(heading_rad)
    
    # Calculate perpendicular points for the base of the triangle
    perp_rad = heading_rad + math.pi/2
    pdx = 0.15 * math.cos(perp_rad)
    pdy = 0.15 * math.sin(perp_rad)
    
    # Define triangle vertices
    triangle_verts = [
        (rover.x + dx, rover.y + dy),  # front
        (rover.x - dx + pdx, rover.y - dy + pdy),  # left back
        (rover.x - dx - pdx, rover.y - dy - pdy)   # right back
    ]
    
    # Create triangle patch
    rover_patch = Polygon(np.array(triangle_verts), closed=True, 
                         facecolor='blue', edgecolor='black', zorder=10)
    ax.add_patch(rover_patch)
    
    # Update the path line - only show the actual traveled path
    x_coords, y_coords = zip(*rover.history)
    if hasattr(ax, 'actual_path_line'):
        ax.actual_path_line.set_data(x_coords, y_coords)
    
    # Draw plot
    fig.canvas.draw_idle()
    plt.pause(0.01)
    
    return rover_patch

def visualize_turn(rover, target_heading, ax, fig, rover_patch):
    """Visualize the rover turning to a target heading."""
    # Calculate the shortest direction to turn (clockwise or counter-clockwise)
    diff = (target_heading - rover.heading) % 360
    if diff > 180:
        diff -= 360
    
    # Number of steps for smooth visualization
    num_steps = 5
    
    # Turn in steps
    for i in range(1, num_steps + 1):
        # Interpolate heading
        rover.heading = (rover.heading + diff / num_steps) % 360
        
        # Update visualization
        rover_patch = update_visualization(rover, ax, fig, rover_patch)
        plt.pause(0.02)
    
    # Ensure exact target heading
    rover.heading = target_heading
    rover_patch = update_visualization(rover, ax, fig, rover_patch)
    
    return rover_patch

def generate_zigzag_path(start_x, start_y, width, height, num_rows):
    """Generate a zigzag path for the rover to follow."""
    path = []
    row_spacing = height / (num_rows - 1) if num_rows > 1 else 0
    
    for i in range(num_rows):
        y = start_y + i * row_spacing
        
        if i % 2 == 0:  # Even rows go left to right
            path.append((start_x, y))
            path.append((start_x + width, y))
        else:  # Odd rows go right to left
            path.append((start_x + width, y))
            path.append((start_x, y))
    
    return path

def run_fail_safe_test():
    """Run a test of the fail-safe module with different failure scenarios."""
    # Setup plot
    plt.figure(figsize=(12, 10))
    ax = plt.gca()
    ax.set_title("Rover Zigzag Path Demonstration", fontsize=14)
    
    # Set farm boundaries
    min_x, max_x = 0, 10
    min_y, max_y = 0, 10
    
    # Draw farm boundary
    farm_verts = [(min_x, min_y), (max_x, min_y), (max_x, max_y), (min_x, max_y)]
    farm_polygon = plt.Polygon(np.array(farm_verts), closed=True, 
                              facecolor='lightgreen', edgecolor='darkgreen', alpha=0.3)
    ax.add_patch(farm_polygon)
    
    # Set up axis
    ax.set_xlim(min_x - 1, max_x + 1)
    ax.set_ylim(min_y - 1, max_y + 1)
    ax.grid(True, linestyle='--', alpha=0.7)
    ax.set_xlabel("X Position (m)")
    ax.set_ylabel("Y Position (m)")
    
    # Initialize rover
    start_x, start_y = 1, 1
    rover = SimpleRover(start_x, start_y, heading=0)
    
    # Initialize fail-safe module
    fail_safe = FailSafeModule()
    
    # Generate zigzag path
    zigzag_path = generate_zigzag_path(start_x, start_y, max_x - 2*start_x, max_y - 2*start_y, 5)
    
    # Plot the zigzag path (planned path)
    path_x, path_y = zip(*zigzag_path)
    ax.plot(path_x, path_y, 'r--', alpha=0.5, linewidth=2, label='Planned Path')
    
    # Create smooth curves for the planned path visualization
    smooth_path_x = []
    smooth_path_y = []
    
    # Connect waypoints with more points for a smoother appearance
    for i in range(len(path_x) - 1):
        # Add multiple points between each waypoint
        for t in np.linspace(0, 1, 20):
            smooth_path_x.append(path_x[i] * (1-t) + path_x[i+1] * t)
            smooth_path_y.append(path_y[i] * (1-t) + path_y[i+1] * t)
    
    # Plot the smoother planned path
    ax.plot(smooth_path_x, smooth_path_y, 'g-', alpha=0.3, linewidth=1.5)
    
    # Mark waypoints clearly
    for i, (wx, wy) in enumerate(zigzag_path):
        ax.scatter(wx, wy, c='darkblue', s=80, edgecolor='white', zorder=5)
        ax.text(wx+0.1, wy+0.1, f"WP{i}", fontsize=9, weight='bold')
    
    # Mark start and end points
    ax.scatter(zigzag_path[0][0], zigzag_path[0][1], c='green', s=120, edgecolor='white', label='Start', zorder=6)
    ax.scatter(zigzag_path[-1][0], zigzag_path[-1][1], c='red', s=120, edgecolor='white', label='End', zorder=6)
    
    # Initialize actual path line for visualization
    actual_path_line, = ax.plot([rover.x], [rover.y], 'b-', alpha=0.8, linewidth=2.5, label='Actual Path')
    ax.actual_path_line = actual_path_line  # Store reference to update later
    
    # Create initial rover visualization
    rover_patch = update_visualization(rover, ax, fig=plt.gcf())
    
    # Legend
    ax.legend(loc='upper left', fontsize=10)
    
    # Show instructions
    print("\n=== ROVER ZIGZAG PATH DEMONSTRATION ===")
    print("The rover will follow a zigzag pattern across the farm")
    print("\nPress any key to start...")
    plt.pause(1)
    
    # Navigate through the path
    current_waypoint_index = 0
    
    # Function to get next waypoint
    def get_next_waypoint():
        nonlocal current_waypoint_index
        if current_waypoint_index < len(zigzag_path) - 1:
            current_waypoint_index += 1
        return zigzag_path[current_waypoint_index]
    
    # Main navigation loop
    while current_waypoint_index < len(zigzag_path) - 1:
        # Get the current and next waypoints
        current_wp = zigzag_path[current_waypoint_index]
        next_wp = get_next_waypoint()
        
        print(f"\nNavigating to waypoint {current_waypoint_index}: ({next_wp[0]:.2f}, {next_wp[1]:.2f})")
        
        # Calculate heading to the next waypoint
        target_heading = rover.calculate_heading_to(next_wp[0], next_wp[1])
        print(f"Turning to heading: {target_heading:.1f}°")
        
        # Turn the rover
        rover_patch = visualize_turn(rover, target_heading, ax, plt.gcf(), rover_patch)
        
        # Calculate distance to next waypoint
        distance_to_wp = rover.distance_to(next_wp[0], next_wp[1])
        
        # Move in smaller steps for smoother visualization
        step_size = 0.2  # Smaller steps for smoother movement
        remaining_distance = distance_to_wp
        
        while remaining_distance > step_size:
            rover.move_forward(step_size, ax, plt.gcf(), rover_patch)
            remaining_distance -= step_size
            plt.pause(0.01)  # Shorter pause for smoother animation
        
        # Move the remaining distance
        if remaining_distance > 0:
            rover.move_forward(remaining_distance, ax, plt.gcf(), rover_patch)
        
        # Mark the waypoint as reached
        ax.scatter(next_wp[0], next_wp[1], c='cyan', s=60, alpha=0.7, zorder=7)
    
    # Mark completion
    ax.scatter(rover.x, rover.y, c='green', s=120, marker='*', label='Mission Complete', zorder=8)
    ax.legend(loc='upper left')
    
    print("\n✅ PATH COMPLETE!")
    print("The rover has successfully followed the zigzag path")
    
    # Keep plot open
    plt.show()

if __name__ == "__main__":
    try:
        run_fail_safe_test()
    except KeyboardInterrupt:
        print("\n\n🛑 Test terminated by user.")
    except Exception as e:
        print(f"\n❌ Test error: {e}")