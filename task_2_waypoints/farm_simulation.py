import matplotlib.pyplot as plt
import numpy as np
import random
import logging_100mm
# Import our modules
from row_navigation import Rover, navigate_to_point, TOLERANCE, follow_path_precisely, update_rover_visualization, visualize_turn
from row_navigation import RowNavigator
from farm_safety import SafetyModule
from sleep_mode import FailsafeModule
debug = False
safety = SafetyModule()

def get_float(prompt):
    """Get a float value from user with error handling"""
    while True:
        try:
            value = float(input(prompt))
            return value
        except ValueError:
            print("⚠️ Please enter a valid number.")

def random_position_in_farm(min_x, max_x, min_y, max_y, safety_margin=2.0):
    """Generate a random position inside the farm with a safety margin from boundaries"""
    x = random.uniform(min_x + safety_margin, max_x - safety_margin)
    y = random.uniform(min_y + safety_margin, max_y - safety_margin)
    return x, y

def safe_remove(element):
    if element:
        try:
            element.remove()
            return True
        except:
            if debug: print(f"Warning: failed to remove {element}")
    return False

def run_simulation():
    def on_failsafe_triggered(reason):
        print(f"⚠️ Failsafe triggered: {reason.value}")
        rover.log_movement("stop")  # Stop the rover for safety

    def on_recovery_attempt(reason):
        print(f"🔄 Attempting recovery from {reason.value}")
        return True  # Assume recovery succeeds for simulation
    
    print("🚜 Farm Rover Navigation Simulation 🚜")
    print("=====================================")
    plt.rcParams['figure.max_open_warning'] = 50

    
    # Create the rover
    rover = Rover()
    gps_logger = logging_100mm.initialize_gps_logger(rover)
  
    
    # Setup the farm boundaries (only input required from user)
  # Create row navigator first to load waypoints
    navigator = RowNavigator(rover)
    failsafe = FailsafeModule()
    failsafe.update_gps_status(has_fix=True, satellites=10, hdop=1.0)
    failsafe.update_internet_status(connected=True, latency=0.1)
    failsafe.update_module_communication()
    rover.failsafe = failsafe
    rover.navigator = navigator
    failsafe.set_callbacks(on_failsafe_triggered, on_recovery_attempt)
    failsafe.start_monitoring()

    navigator.zigzag_pattern = True

# Load waypoints from CSV file
    csv_loaded = navigator.load_rows_from_csv(r"F:\GPS\task_2_waypoints\waypoints_100mm.csv")
    if not csv_loaded:
        print("❌ Failed to load waypoints from CSV. Simulation cannot proceed without waypoints.")
        return
        
    # Calculate farm boundaries based on waypoints with margin
    margin = 3.0  # Add margin around waypoints
    min_x = min(point[0] for point in navigator.interpolated_path) - margin
    max_x = max(point[0] for point in navigator.interpolated_path) + margin
    min_y = min(point[1] for point in navigator.interpolated_path) - margin
    max_y = max(point[1] for point in navigator.interpolated_path) + margin
    entry_point = (min_x, min_y)
    # Create vertices for the farm boundary
    verts = [(min_x, min_y), (max_x, min_y), (max_x, max_y), (min_x, max_y)]

    print(f"📏 Dynamic farm boundaries: X [{min_x:.2f}, {max_x:.2f}], Y [{min_y:.2f}, {max_y:.2f}]")
    
    # Create vertices for the farm boundary
    verts = [(min_x, min_y), (max_x, min_y), (max_x, max_y), (min_x, max_y)]
    
    # Generate a random entry point (we'll still set this for compatibility even though not used)
    # Choose a random side and position on that side
    side = random.randint(0, 3)
    if side == 0:  # Bottom side
        entry_x = random.uniform(min_x, max_x)
        entry_y = min_y
    elif side == 1:  # Right side
        entry_x = max_x
        entry_y = random.uniform(min_y, max_y)
    elif side == 2:  # Top side
        entry_x = random.uniform(min_x, max_x)
        entry_y = max_y
    else:  # Left side
        entry_x = min_x
        entry_y = random.uniform(min_y, max_y)
    
    entry_point = (entry_x, entry_y)
    
    # Set geofence in rover and safety module
    rover.set_geofence(verts, entry_point)
    safety.set_geofence(verts)

    # Remove no-go zone creation
    # The following block is commented out to remove the reddish square
    """
    # Add a rectangular no-go zone in the middle of the farm
    center_x = (min_x + max_x) / 2
    center_y = (min_y + max_y) / 2
    size = 1.5  # Size of the no-go zone
    safety.add_no_go_zone(center_x - size, center_y - size, center_x + size, center_y + size)
    """
    
    # Generate random starting position inside the farm
    random_x, random_y = random_position_in_farm(min_x, max_x, min_y, max_y)
    print(f"🎲 Randomly placing rover inside farm at: ({random_x:.3f}, {random_y:.3f})")
    
    # Initialize visualization
    plt.ion()
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_title("Rover Farm Navigation Simulation")
    
    # Draw farm boundary
    farm_polygon = plt.Polygon(np.array(verts), closed=True, 
                              facecolor='lightgreen', edgecolor='darkgreen', alpha=0.3)
    ax.add_patch(farm_polygon)
    
    # Remove no-go zone visualization
    # The following block is commented out to remove the reddish square
    """
    # Draw no-go zone
    no_go_verts = [
        (center_x - size, center_y - size),
        (center_x + size, center_y - size),
        (center_x + size, center_y + size),
        (center_x - size, center_y + size)
    ]
    no_go_polygon = plt.Polygon(np.array(no_go_verts), closed=True,
                               facecolor='red', edgecolor='darkred', alpha=0.3)
    ax.add_patch(no_go_polygon)
    """
    
    # Mark random start position
    ax.scatter(random_x, random_y, c='green', s=80, label='Start (Inside)')
    
    # Setup plot limits and grid

    
    # Setup rover path visualization
    path_line, = ax.plot([], [], 'b-', alpha=0.5, label='Path')
    ax.path_line = path_line
    ax.legend(loc='upper left')
    
    # Set rover starting position (inside farm)
    rover.set_position(random_x, random_y, force=True, add_to_history=False)
    rover.inside_fence = True  # Force the rover to be considered inside the farm
    rover.fence_locked = True  # Lock the rover inside the farm
    rover.history.append((rover.x, rover.y))
    rover_patch = update_rover_visualization(rover, ax, fig)
    
    print("\n🚜 TASK 1: Determining farm navigation plan with zigzag pattern...\n")

    # Create row navigator
   
    navigator.zigzag_pattern = True  # Ensure zigzag pattern is enabled

    # Generate rows within the farm using zigzag pattern
# Load waypoints from CSV file
    csv_loaded = navigator.load_rows_from_csv(r"F:\GPS\task_2_waypoints\waypoints_100mm.csv")
    if navigator.interpolated_path:
        wp_min_x = min(point[0] for point in navigator.interpolated_path)
        wp_max_x = max(point[0] for point in navigator.interpolated_path)
        wp_min_y = min(point[1] for point in navigator.interpolated_path)
        wp_max_y = max(point[1] for point in navigator.interpolated_path)
        
        # Use the wider range between farm boundaries and waypoints
        plot_min_x = min(min_x, wp_min_x)
        plot_max_x = max(max_x, wp_max_x) 
        plot_min_y = min(min_y, wp_min_y)
        plot_max_y = max(max_y, wp_max_y)
        
        # Add a larger margin
        margin = max(plot_max_x - plot_min_x, plot_max_y - plot_min_y) * 0.15
        ax.set_xlim(plot_min_x - margin, plot_max_x + margin)
        ax.set_ylim(plot_min_y - margin, plot_max_y + margin)
    else:
        # Fallback to original farm boundaries
        margin = 3
        ax.set_xlim(min_x - margin, max_x + margin)
        ax.set_ylim(min_y - margin, max_y + margin)
    ax.grid(True)

    if not csv_loaded:
         print("❌ Failed to load waypoints from CSV. Simulation cannot proceed without waypoints.")
         return
  

    safety.set_waypoints(navigator.interpolated_path)


    # Visualize zigzag row pattern
    x_coords, y_coords = zip(*navigator.interpolated_path)
    ax.plot(x_coords, y_coords, 'b-', alpha=0.5, label='Zig-Zag Path')

    # Mark start and end points
    path_start = navigator.interpolated_path[0]
    path_end = navigator.interpolated_path[-1]
    ax.scatter(path_start[0], path_start[1], c='orange', s=50, marker='s', label='Path Start')
    ax.scatter(path_end[0], path_end[1], c='red', s=50, marker='o', label='Path End')

    fig.canvas.draw_idle()
    plt.pause(0.5)

    # --- TASK 1: Navigate directly to the path start point ---
    print("\n🚜 TASK 1: Navigating directly to path start point...\n")
    print(f"🎯 Path start point: ({path_start[0]:.3f}, {path_start[1]:.3f})")
    print(f"📏 Distance to path start: {rover.distance_to(*path_start):.3f}m")
    
 

    def on_rover_wakeup():
        print("Rover has woken up! Resuming operations...")
        # Do whatever you need when rover wakes up

    # Navigate to path start
    def navigate_to_path_start(rover, safety, path_start, ax, fig, rover_patch):
          
        """
        Navigate rover to the starting point of the path using direct point-to-point moves
        with a larger step size and a slightly more generous tolerance to avoid getting stuck.
        """
        print("\n🗺️ Navigating directly to starting point...")

        reached_start, rover_patch = navigate_to_point(
            rover,
            path_start[0],
            path_start[1],
            ax,
            fig,
            rover_patch,
            step_size=1.5,   # larger increments per move
            tolerance=0.8    # accept slightly further from the exact point
        )

        return reached_start, rover_patch

     

    # Use our custom function to navigate to path start
    reached_start, rover_patch = navigate_to_path_start(rover, safety, path_start, ax, fig, rover_patch)

    if not reached_start:
        print("\n⚠️ Could not reach path start point after multiple attempts.")
        print("   Try adjusting simulation parameters or path positioning.")
        return

    # Force rover position to exactly match path start
    rover.set_position(path_start[0], path_start[1], force=True)
    rover_patch = update_rover_visualization(rover, ax, fig, rover_patch)

    # Mark path start reached
    ax.scatter(path_start[0], path_start[1], c='lime', s=80, marker='*', label='Start Reached')
    ax.legend(loc='upper left')
    fig.canvas.draw_idle()
    plt.pause(1)
    print("\n✅ TASK 1 COMPLETE: Successfully reached path start point")
    print(f"   Current position: ({rover.x:.3f}, {rover.y:.3f})")

    # --- TASK 2: Align to the path direction ---
    print("\n🚜 TASK 2: Aligning rover to path direction...\n")
    # Find next waypoint (should be index 1 since we're at index 0)
    navigator.current_waypoint_index = 0  # Force to start at the beginning of the path
    next_point = navigator.interpolated_path[1]
    desired_heading = navigator.calculate_heading((rover.x, rover.y), next_point)

    # Align to the path direction
    rover_patch = visualize_turn(rover, desired_heading, ax, fig, rover_patch)

    print(f"   Aligned rover to heading: {desired_heading:.1f}°")
    print("\n✅ TASK 2 COMPLETE: Successfully aligned to path direction")

    # --- TASK 3: Navigate through the path ---
    print("\n🚜 TASK 3: Starting path navigation pattern...\n")
    # Start navigation from the beginning of the path
    navigator.current_waypoint_index = 0
    path_success = navigator.navigate_path(ax, fig, rover_patch)
    if not path_success:
        print("\n⚠️ Failed to navigate path. Simulation halted.")
        return
    
    # Mark completion of path
    final_point = navigator.interpolated_path[-1]
    ax.scatter(final_point[0], final_point[1], c='green', s=100, marker='*', label='Mission Complete')
    ax.legend(loc='upper left')
    fig.canvas.draw_idle()
    plt.pause(1)
    
    print("\n🎉 TASK 3 COMPLETE: Successfully navigated the path")
    print("\n🏁 SIMULATION COMPLETE! 🏁")
    print(f"   Total commands executed: {rover.command_count}")
    print(f"   Final position: ({rover.x:.3f}, {rover.y:.3f})")
    
    # Keep plot open until closed manually
    plt.ioff()
    plt.show(block=True)
    logging_100mm.stop_gps_logger(rover)
    failsafe.stop_monitoring()


if __name__ == "__main__":
    try:
        run_simulation()
    except KeyboardInterrupt:
        print("\n\n🛑 Simulation terminated by user.")
    except Exception as e:
        print(f"\n❌ Simulation error: {e}")