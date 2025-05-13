import matplotlib.pyplot as plt
import time
import numpy as np
import random
import logging_100mm
# Import our modules
from row_navigation import Rover, navigate_to_point, TOLERANCE, follow_path_precisely, update_rover_visualization, visualize_turn
from row_navigation import RowNavigator
from farm_safety import SafetyModule
from sleep_mode import FailsafeModule, GPSFailsafeReason, DriftSeverity, DriftAction
# Import the health check module from the second file
from rover_health_check import RoverHealthCheck, HealthCheckFailure
# Import coordinate converter
from coordinate_converter import CoordinateConverter
import threading
from ntrip_client import NTRIPClient
from emlid_gps_integration import EmlidGPSReader,update_rover_from_emlid, setup_emlid_integration


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

def process_emlid_gps_data(rover, emlid_data):
    """
    Process GPS data from Emlid receiver and update rover position.
    This is a standalone version that doesn't rely on class methods.
    
    Args:
        rover: The rover instance
        emlid_data: Dictionary with GPS data from Emlid receiver
    
    Returns:
        bool: True if position was updated successfully, False otherwise
    """
    if not emlid_data or 'latitude' not in emlid_data or 'longitude' not in emlid_data:
        print("⚠️ Invalid or missing Emlid GPS data")
        return False
        
    try:
        # Get the coordinate converter
        converter = None
        if hasattr(rover, 'coordinate_converter'):
            converter = rover.coordinate_converter
        elif hasattr(rover, 'gps_logger') and hasattr(rover.gps_logger, 'converter'):
            converter = rover.gps_logger.converter
        else:
            print("⚠️ No coordinate converter found")
            return False
        
        # Convert lat/lon to UTM
        easting, northing = converter.latlon_to_utm_coord(
            emlid_data['latitude'], 
            emlid_data['longitude']
        )
        
        if easting is None or northing is None:
            print("⚠️ Failed to convert lat/lon to UTM")
            return False
        
        # Get the correct UTM offsets
        utm_offset_x = 0
        utm_offset_y = 0
        if hasattr(rover, 'navigator') and hasattr(rover.navigator, 'utm_offset_x'):
            utm_offset_x = rover.navigator.utm_offset_x
            utm_offset_y = rover.navigator.utm_offset_y
        
        # Calculate the local coordinates by removing the offsets
        local_x = easting - utm_offset_x
        local_y = northing - utm_offset_y
        
        # Update rover position
        rover.set_position(local_x, local_y)
        
        # Log the update if a logger is available
        if hasattr(rover, 'gps_logger'):
            if hasattr(rover.gps_logger, 'log_data_once'):
                rover.gps_logger.log_data_once()
        
        return True
    except Exception as e:
        print(f"Error processing Emlid GPS data: {e}")
        return False
def run_simulation():
    def on_failsafe_triggered(reason):
        print(f"⚠️ Failsafe triggered: {reason.value}")
        rover.log_movement("stop")  # Stop the rover for safety

    def on_recovery_attempt(reason):
        print(f"🔄 Attempting recovery from {reason.value}")
        current_time = time.time()
        if reason == GPSFailsafeReason.GPS_STALE_DATA or reason == GPSFailsafeReason.GPS_DATA_LOSS:
            failsafe.last_gps_update = current_time
        elif reason == GPSFailsafeReason.INTERNET_CONNECTION_LOST or reason == GPSFailsafeReason.INTERNET_CONNECTION_SLOW:
            failsafe.last_internet_check = current_time
        elif reason == GPSFailsafeReason.MODULE_COMMUNICATION_FAILURE:
            failsafe.last_module_comm = current_time
        return True  # Assume recovery succeeds for simulation
        
    print("🚜 Farm Rover Navigation Simulation 🚜")
    print("=====================================")
     
    # -------------------- HEALTH CHECK SECTION --------------------
    print("\n🔍 Running rover health checks before simulation...")
    
    # Create the rover first (needed by the health checker)
    rover = Rover()
    
    # Initialize the coordinate converter
    coordinate_converter = CoordinateConverter()
    rover.coordinate_converter = coordinate_converter
    
    # Initialize health checker with the rover instance
    health_checker = RoverHealthCheck(rover)
    
    try:
        # Run all health checks
        health_status = health_checker.run_all_checks(simulation_mode=True)        
        # Generate and display health report
        health_report = health_checker.generate_health_report()
        print(health_report)
        
        # Check if all systems passed
        if not all(health_status.values()):
            print("\n⚠️ One or more health checks failed. Aborting simulation.")
            print("   Please address the issues and try again.")
            return
            
        print("\n✅ All health checks passed! Proceeding with simulation.")
    
    except HealthCheckFailure as e:
        print(f"\n❌ Critical health check failure: {e}")
        print("   Simulation cannot proceed until this issue is resolved.")
        return
    # -------------------- END HEALTH CHECK SECTION --------------------
    plt.rcParams['figure.max_open_warning'] = 50
    
    failsafe = FailsafeModule()
    safety = SafetyModule(failsafe=failsafe)
    failsafe.set_safety_module(safety)

    # Initialize failsafe first
    rover.failsafe = failsafe
    failsafe.update_gps_status(has_fix=True, satellites=10, hdop=1.0)
    failsafe.update_internet_status(connected=True, latency=0.1)
    failsafe.update_module_communication()
    failsafe.set_callbacks(on_failsafe_triggered, on_recovery_attempt)
    
    # Now initialize GPS logger
    gps_logger = logging_100mm.initialize_gps_logger(rover)
    # Initialize GPS reader for NMEA and corrections
    # Initialize GPS reader and attach to rover
    emlid_reader = EmlidGPSReader(message_format='nmea')
    update_rover_from_emlid(rover, emlid_reader)

    success = emlid_reader.connect(retries=5, retry_delay=3)
    if not success:
        print("❌ Failed to connect to Emlid receiver")
        return

    emlid_reader.start_reading()
    rover.gps_reader = emlid_reader  # Attach to rover so NTRIP thread can use it

    
    
    NTRIP_CONFIG = {
    'host': 'your.ntrip.server.url',
    'port': 2101,
    'mountpoint': 'MOUNTPOINT',
    'user': 'your_username',
    'password': 'your_password'
    }


    
    ntrip_client = NTRIPClient(**NTRIP_CONFIG)
    
    # Start NTRIP corrections in a background thread
    def stream_rtcm_corrections():
        for rtcm_data in ntrip_client.get_corrections():
            if hasattr(rover, 'gps_reader'):  # Ensure EmlidGPSReader is attached
                rover.gps_reader.send_rtcm_data(rtcm_data)
    
    ntrip_thread = threading.Thread(target=stream_rtcm_corrections, daemon=True)
    ntrip_thread.start()
    # === END ADDITION ===
    # Create row navigator
    navigator = RowNavigator(rover)
    rover.navigator = navigator
    
    # Start failsafe monitoring
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
    
    print(f"📏 Dynamic farm boundaries: X [{min_x:.2f}, {max_x:.2f}], Y [{min_y:.2f}, {max_y:.2f}]")
    
    # Create vertices for the farm boundary
    verts = [(min_x, min_y), (max_x, min_y), (max_x, max_y), (min_x, max_y)]
    
    # Generate a random entry point
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
    
    # Mark random start position
    ax.scatter(random_x, random_y, c='green', s=80, label='Start (Inside)')
    
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

    # Ensure zigzag pattern is enabled
    navigator.zigzag_pattern = True  

    # Use the waypoints previously loaded from CSV
    safety.set_waypoints(navigator.interpolated_path)

    # Determine plot boundaries based on waypoints
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

    #Force rover position to exactly match path start
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

def simulate_emlid_gps_reading():
    """
    Simulate an Emlid GPS reading for testing purposes.
    Returns a dictionary with lat/lon coordinates and RTK status.
    """
    # These are example coordinates - in a real implementation,
    # you would get these from the Emlid GPS receiver
    
    # Randomly choose a solution status for demonstration
    solution_statuses = ["fixed", "float", "single", "dgps"]
    solution_status = random.choice(solution_statuses)
    
    # Determine appropriate HDOP based on solution status
    if solution_status == "fixed":
        hdop = random.uniform(0.01, 0.2)
    elif solution_status == "float":
        hdop = random.uniform(0.2, 0.5)
    elif solution_status == "dgps":
        hdop = random.uniform(0.5, 1.0)
    else:  # single
        hdop = random.uniform(1.0, 2.0)
    
    return {
        'latitude': 28.6139,              # Example latitude
        'longitude': 77.2090,             # Example longitude
        'solution_status': solution_status,  # RTK solution status from Emlid
        'satellites': random.randint(8, 15),  # Number of satellites
        'hdop': hdop                      # Horizontal dilution of precision
    }

def test_emlid_integration():
    """
    Test function to verify Emlid GPS integration with the rover system.
    """
    print("🧪 Testing Emlid GPS integration...")
    
    # Create rover instance
    rover = Rover()
    
    # Initialize coordinate converter
    converter = CoordinateConverter()
    rover.coordinate_converter = converter
    
    # Create row navigator (needed for UTM offsets)
    navigator = RowNavigator(rover)
    rover.navigator = navigator
    
    # Set default UTM offsets for testing
    navigator.utm_offset_x = 380000.0
    navigator.utm_offset_y = 2044880.0
    
    # Initialize GPS logger
    gps_logger = logging_100mm.initialize_gps_logger(rover)
    
    # Simulate Emlid GPS reading
    emlid_data = simulate_emlid_gps_reading()
    print(f"📡 Simulated Emlid GPS reading: Lat={emlid_data['latitude']}, Lon={emlid_data['longitude']}")
    
    # Use the built-in function from the logging_100mm module
    success = logging_100mm.update_rover_position_from_emlid(rover, emlid_data)
    
    # Check the result
    if success:
        print("✅ Successfully processed Emlid GPS data")
        print(f"🚜 Rover position (UTM): X={rover.x:.3f}, Y={rover.y:.3f}")
        
        # Calculate the actual global UTM coordinates
        actual_easting = rover.x + rover.navigator.utm_offset_x
        actual_northing = rover.y + rover.navigator.utm_offset_y
        
        # Convert back to lat/lon for verification
        lat, lon = converter.utm_to_latlon_coord(
            actual_easting, actual_northing,
            zone_number=45, zone_letter='N'  # Make sure to use the correct zone
        )
        print(f"🌐 Rover position (Lat/Lon): {lat:.6f}, {lon:.6f}")
        
        # Calculate difference from original coordinates
        original_lat = emlid_data['latitude']
        original_lon = emlid_data['longitude']
        lat_diff = abs(lat - original_lat)
        lon_diff = abs(lon - original_lon)
        print(f"📊 Conversion difference: Lat={lat_diff:.8f}, Lon={lon_diff:.8f}")
        
        if lat_diff < 0.0001 and lon_diff < 0.0001:
            print("✅ Conversion accuracy check passed")
        else:
            print("❌ Conversion accuracy check failed - differences too large")
    else:
        print("❌ Failed to process Emlid GPS data")
    
    # Cleanup
    logging_100mm.stop_gps_logger(rover)
    print("🧪 Test completed")

if __name__ == "__main__":
    try:
        # Uncomment to test Emlid integration separately
        # test_emlid_integration()
        
        # Run the main simulation
        run_simulation()
    except KeyboardInterrupt:
        print("\n\n🛑 Simulation terminated by user.")
    except Exception as e:
        print(f"\n❌ Simulation error: {e}")
        # For debugging:
        import traceback
        traceback.print_exc()