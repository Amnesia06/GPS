import matplotlib.pyplot as plt
import numpy as np
from astar_algo import AStarPlanner

# Import our modules
from farm_entry import Rover, update_rover_visualization, visualize_turn, navigate_to_point, TOLERANCE
from row_navigation import RowNavigator
from farm_safety import SafetyModule
safety = SafetyModule()
# Set farm boundary
# Add any no-go zones
def get_float(prompt):
    """Get a float value from user with error handling"""
    while True:
        try:
            value = float(input(prompt))
            return value
        except ValueError:
            print("⚠️ Please enter a valid number.")

def run_simulation():
    
    print("🚜 Farm Rover Navigation Simulation 🚜")
    print("=====================================")
    
    # Create the rover
    rover = Rover()
    
    # Setup the geofence (farm boundaries)
    print("\n🔧 Enter farm rectangle coordinates:")
    min_x = get_float(" Min X: ")
    max_x = get_float(" Max X: ")
    min_y = get_float(" Min Y: ")
    max_y = get_float(" Max Y: ")
    
    # Create vertices for the farm boundary
    verts = [(min_x, min_y), (max_x, min_y), (max_x, max_y), (min_x, max_y)]
    
    # Allow user to manually input entry point on farm border
    print("\n🚪 Enter farm entry point coordinates (must be on farm border):")
    print(f"  Valid X range: {min_x} to {max_x}")
    print(f"  Valid Y range: {min_y} to {max_y}")
    print("  The point must be exactly on one of the farm boundary lines.")
    
    while True:
        entry_x = get_float(" Entry point X: ")
        entry_y = get_float(" Entry point Y: ")
        
        # Check if point is on the border of the farm
        on_border = False
        
        # Check if point is on horizontal borders
        if (min_x <= entry_x <= max_x) and (entry_y == min_y or entry_y == max_y):
            on_border = True
            
        # Check if point is on vertical borders
        elif (min_y <= entry_y <= max_y) and (entry_x == min_x or entry_x == max_x):
            on_border = True
            
        if on_border:
            print(f"✅ Valid entry point: ({entry_x:.2f}, {entry_y:.2f})")
            break
        else:
            print("⚠️ Entry point must be exactly on the farm border. Please try again.")
    
    entry_point = (entry_x, entry_y)
    
    # Set geofence in rover
    rover.set_geofence(verts, entry_point)
    # Set farm boundary in safety module
    safety.set_geofence(verts)

    # Rest of the code continues as before...
    

   # Modified A* integration code that properly handles visualization scopes
# Place this in the run_simulation function after the farm boundary setup
# and before the "Navigate to entry point" section

    def integrate_astar_planner(rover, safety, entry_point, ax, fig, rover_patch):
        """
        Integrates the A* planner into the simulation with proper visualization scope
        Returns: reached_entry (bool), updated_rover_patch
        """
        # Create and configure the A* planner with appropriate parameters
        print("\n🗺️ Setting up A* path planner for navigating to entry point...")
        planner = AStarPlanner(rover, safety, cell_size=0.2, padding=2.5)
        print(f"🔍 Grid dimensions: {planner.ny}x{planner.nx} cells")
        print(f"📊 Start cell: {planner.start}, Goal cell: {planner.goal}")
        
        # Optional: Visualize the grid for debugging (in a separate figure)
        try:
            grid_fig, grid_ax = plt.subplots(figsize=(8, 6))
            planner.visualize_grid(grid_ax)
            grid_fig.canvas.draw_idle()
            plt.pause(0.5)
        except Exception as e:
            print(f"Note: Grid visualization skipped: {e}")
        
        # Plan the path
        print("\n🔄 Computing optimal path to entry point...")
        waypoints = planner.plan()
        
        if waypoints:
            print(f"✅ A* found a path with {len(waypoints)} waypoints:")
            for i, (wx, wy) in enumerate(waypoints):
                if i == 0:
                    print(f"   Start: ({wx:.2f}, {wy:.2f})")
                elif i == len(waypoints) - 1:
                    print(f"   Goal: ({wx:.2f}, {wy:.2f})")
                elif i < 5 or i > len(waypoints) - 5:
                    print(f"   Waypoint {i}: ({wx:.2f}, {wy:.2f})")
                elif i == 5 and len(waypoints) > 10:
                    print(f"   ... {len(waypoints) - 10} more waypoints ...")
            
            # Visualize the planned path on the main figure
            path_x, path_y = zip(*waypoints)
            ax.plot(path_x, path_y, 'y--', linewidth=2, alpha=0.7, label='Planned Path')
            ax.legend(loc='upper left')
            fig.canvas.draw_idle()
            plt.pause(0.5)
            
            print("\n🚜 Following A* path to entry point...")
            # Skip the first waypoint (current position)
            for i, (wx, wy) in enumerate(waypoints[1:], 1):
                print(f"\n📍 Moving to waypoint {i}/{len(waypoints)-1}: ({wx:.2f}, {wy:.2f})")
                reached, rover_patch = navigate_to_point(rover, wx, wy, ax, fig, rover_patch)
                
                # Update visualization
                fig.canvas.draw_idle()
                plt.pause(0.2)
                
                if not reached:
                    print(f"⚠️ Failed to reach waypoint {i}. Attempting to continue to next waypoint.")
                    # If we've been stuck on consecutive waypoints, regenerate path from current position
                    if i > 1 and not reached:
                        print("🔄 Replanning path from current position...")
                        # Reinitialize planner with current position
                        planner = AStarPlanner(rover, safety, cell_size=0.2, padding=2.5)
                        new_waypoints = planner.plan()
                        
                        if new_waypoints and len(new_waypoints) > 1:
                            print(f"✅ Found new path with {len(new_waypoints)} waypoints")
                            # Replace remaining waypoints with new plan
                            waypoints = waypoints[:i] + new_waypoints[1:]
            
            # Check if we reached the entry point
            if rover.distance_to(*entry_point) <= TOLERANCE:
                print("\n🎯 Successfully reached entry point using A* navigation!")
                return True, rover_patch
            else:
                print(f"\n⚠️ A* navigation terminated {rover.distance_to(*entry_point):.2f}m from entry point.")
                print("🔄 Attempting direct navigation for final approach...")
                
                # Final direct approach to entry point
                reached_entry, rover_patch = navigate_to_point(
                    rover, entry_point[0], entry_point[1], ax, fig, rover_patch)
                return reached_entry, rover_patch
        else:
            print("⚠️ A* failed to find a path—falling back to direct navigate_to_point.")
            # Use the original direct navigation as fallback
            reached_entry, rover_patch = navigate_to_point(
                rover, entry_point[0], entry_point[1], ax, fig, rover_patch)
            return reached_entry, rover_patch



# Add no-go zones if needed
# For example, to add a rectangular no-go zone in the middle of the farm:
    center_x = (min_x + max_x) / 2
    center_y = (min_y + max_y) / 2
    size = 1.5  # Size of the no-go zone
    safety.add_no_go_zone(center_x - size, center_y - size, center_x + size, center_y + size)
    
    # Get rover starting position (outside farm)
    print("\n🔧 Enter starting position (must be outside the farm):")
    while True:
        x1 = get_float(" x1: ")
        y1 = get_float(" y1: ")
        if rover.is_point_in_polygon(x1, y1, verts):
            print("⚠️ Starting point must be outside the farm. Please enter new coordinates.")
        else:
            break
    print(f"✅ Valid starting position: ({x1:.3f}, {y1:.3f})")
    
    # Initialize visualization
    plt.ion()
    fig, ax = plt.subplots(figsize=(12, 10))
    ax.set_title("Rover Farm Navigation Simulation")
    
    # Draw farm boundary
    farm_polygon = plt.Polygon(np.array(verts), closed=True, 
                              facecolor='lightgreen', edgecolor='darkgreen', alpha=0.3)
    ax.add_patch(farm_polygon)
    
    # Mark entry point and start position
    ax.scatter(entry_point[0], entry_point[1], c='purple', s=100, marker='o', label='Farm Entry')
    ax.scatter(x1, y1, c='green', s=80, label='Start (Outside)')
    
    # Setup plot limits and grid
    xs = [v[0] for v in verts] + [x1, entry_point[0]]
    ys = [v[1] for v in verts] + [y1, entry_point[1]]
    margin = 3  # Add more margin
    mxx, Mxx = min(xs) - margin, max(xs) + margin
    myy, Myy = min(ys) - margin, max(ys) + margin
    ax.set_xlim(mxx, Mxx)
    ax.set_ylim(myy, Myy)
    ax.grid(True)
    
    # Setup rover path visualization
    path_line, = ax.plot([], [], 'b-', alpha=0.5, label='Path')
    ax.path_line = path_line
    ax.legend(loc='upper left')
    
    # Set rover starting position
    rover.set_position(x1, y1, force=True, add_to_history=False)
    rover.history.append((rover.x, rover.y))
    rover_patch = update_rover_visualization(rover, ax, fig)
    
    # --- TASK 1: Enter the farm at entry point ---
    print("\n🚜 TASK 1: Moving rover from outside farm to entry point...\n")
    print(f"📏 Initial distance to entry point: {rover.distance_to(*entry_point):.3f}m")
    
    reached_entry, rover_patch = integrate_astar_planner(rover, safety, entry_point, ax, fig, rover_patch)

    if not reached_entry:
        print("\n⚠️ Could not reach farm entry point after multiple attempts.")
        print("   Try adjusting simulation parameters or entry point location.")
        return

    # Force rover position to exactly match entry point
    rover.set_position(entry_point[0], entry_point[1], force=True)
    rover_patch = update_rover_visualization(rover, ax, fig, rover_patch)

    # Mark entry point reached
    ax.scatter(entry_point[0], entry_point[1], c='cyan', s=80, marker='^', label='Entry Reached')
    ax.legend(loc='upper left')
    fig.canvas.draw_idle()
    plt.pause(1)
    print("\n✅ TASK 1 COMPLETE: Successfully entered the farm")
    print(f"   Current position: ({rover.x:.3f}, {rover.y:.3f})")
    
    # Force rover position to exactly match entry point
    rover.set_position(entry_point[0], entry_point[1], force=True)
    rover_patch = update_rover_visualization(rover, ax, fig, rover_patch)
    
    # Mark entry point reached
    # Mark entry point reached
    ax.scatter(entry_point[0], entry_point[1], c='cyan', s=80, marker='^', label='Entry Reached')
    ax.legend(loc='upper left')
    fig.canvas.draw_idle()
    plt.pause(1)
    print("\n✅ TASK 1 COMPLETE: Successfully entered the farm")
    print(f"   Current position: ({rover.x:.3f}, {rover.y:.3f})")
    
    # --- TASK 2: Generate zigzag row pattern and identify closest row ---
    print("\n🚜 TASK 2: Determining farm navigation plan with zigzag pattern...\n")
    
    # Create row navigator
    navigator = RowNavigator(rover)
    navigator.zigzag_pattern = True  # Ensure zigzag pattern is enabled
    
    # Generate rows within the farm using zigzag pattern
    row_start_x = min_x + 2  # Start rows 2 units from left edge
    row_start_y = min_y + 2  # Start rows 2 units from bottom edge
    row_spacing = 1.5
    num_strips = max(3, min(10, int((max_x - min_x - 4) / row_spacing)))  # Calculate number of strips based on farm width
    
    # Generate rows and visualize them
    rows = navigator.generate_rows(
        row_start_x, row_start_y,
        num_strips=num_strips,
        strip_length=max_y - min_y - 4  # Strip height based on farm height
    )
    
    # Visualize zigzag row pattern
    x_coords, y_coords = zip(*navigator.interpolated_path)
    ax.plot(x_coords, y_coords, 'b-', alpha=0.5, label='Zig-Zag Path')
    
    # Mark start and end points
    ax.scatter(navigator.interpolated_path[0][0], navigator.interpolated_path[0][1], c='orange', s=50, marker='s', label='Path Start')
    ax.scatter(navigator.interpolated_path[-1][0], navigator.interpolated_path[-1][1], c='red', s=50, marker='o', label='Path End')
    
    fig.canvas.draw_idle()
    plt.pause(0.5)
    
    # --- TASK 2.i: Determine the closest row ---
    print("\n🚜 TASK Determining closest point on path...\n")
    navigator.find_closest_waypoint()
    # Identify which row the point belongs to
# Identify which row the point belongs to
    print("\n🚜 TASK Identifying current row...\n")
    navigator.identify_current_row()
    # After the line: navigator.identify_current_row()
    current_row = navigator.identify_current_row()
    print(f"\n✅ Row #{current_row + 1} defined as the first row task")

    # Determine direction to next row
    print("\n🚜 TASK  Determining next row direction...\n")
    next_direction = navigator.determine_next_row_direction()
    print(f"   Next row direction after completing Row #{current_row + 1}: {next_direction}")

    # Determine direction to next row
    print("\n🚜 TASK Determining next row direction...\n")
    navigator.determine_next_row_direction()
    
    # Mark the closest point
    closest_point = navigator.interpolated_path[navigator.current_waypoint_index]
    ax.scatter(closest_point[0], closest_point[1], c='yellow', s=60, marker='*', label='Starting Point0')
    ax.legend(loc='upper left')
    fig.canvas.draw_idle()
    plt.pause(0.5)
    
    print(f"✅ TASK COMPLETE: Identified closest point on path")
    
    # --- TASK 2.ii: Navigate to the closest point on the path ---
    print("\n🚜 TASK Moving to the closest point on the path...\n")
    
    target_point = navigator.interpolated_path[navigator.current_waypoint_index]
    print(f"🎯 Target point: ({target_point[0]:.3f}, {target_point[1]:.3f})")
    print(f"📏 Distance to target: {rover.distance_to(*target_point):.3f}m")
    
    # Navigate to the closest point
    reached_point = False
    for attempt in range(1, 4):
        print(f"\n🔄 Point navigation attempt {attempt}/3...")
        reached_point, rover_patch = navigate_to_point(
            rover, target_point[0], target_point[1], ax, fig, rover_patch)
        if reached_point:
            break
        if attempt < 3:
            rover.blocked_directions.clear()
            print("🔄 Retrying point navigation with new parameters...")
    
    if not reached_point:
        print("\n⚠️ Could not reach closest point after multiple attempts.")
        print("   Try adjusting simulation parameters or path positioning.")
        return
    
    # Force rover position to exactly match target point
    rover.set_position(target_point[0], target_point[1], force=True)
    rover_patch = update_rover_visualization(rover, ax, fig, rover_patch)
    
    
    # Mark point reached
    ax.scatter(target_point[0], target_point[1], c='magenta', s=80, marker='*', label='Point Reached')

    ax.legend(loc='upper left')
    fig.canvas.draw_idle()
    plt.pause(1)
    print("\n✅ TASK COMPLETE: Successfully reached closest point")
    print(f"   Current position: ({rover.x:.3f}, {rover.y:.3f})")
    
    # --- TASK 2.iii: Align to the path direction ---
    print("\n🚜 TASK Aligning rover to path direction...\n")
    navigator.align_to_path(ax, fig, rover_patch)
    print("\n✅ TASK COMPLETE: Successfully aligned to path direction")
    
    # --- TASK 3: Navigate through the path ---
    print("\n🚜 TASK 3: Starting path navigation pattern...\n")
    path_success = navigator.navigate_path(ax, fig, rover_patch)
    if not path_success:
        print("\n⚠️ Failed to navigate path. Simulation halted.")
        return
    
    # Mark completion of path
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

if __name__ == "__main__":
    try:
        run_simulation()
    except KeyboardInterrupt:
        print("\n\n🛑 Simulation terminated by user.")
    except Exception as e:
        print(f"\n❌ Simulation error: {e}")
        if 'DEBUG' in globals() and DEBUG:
            import traceback
            traceback.print_exc()








