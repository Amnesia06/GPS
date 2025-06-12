import matplotlib.pyplot as plt
import datetime
import time
import serial
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
from gps_system_monitor import GPSSystemMonitor
import sys
import logging 
import os
import csv

# Ensure port is available before starting
global_serial_connection = None
# At the top of the file, after imports
direct_test_completed = False  # Add this flag



def direct_serial_test():
    """Test direct serial connection like in test.py and keep it open if successful"""
    global global_serial_connection, direct_test_completed
    
    # Only run the test once
    if direct_test_completed:
        print("Direct serial test already completed, reusing connection")
        return global_serial_connection is not None
    
    print("\n🔬 Testing direct serial connection to COM12...")
    
    # First ensure port is released
    comprehensive_port_release('COM12')
    
    try:
        # Use EXACTLY the same code as your working test.py
        ser = serial.Serial(
            port='COM12',
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
        print("✅ Connected to COM12")
        
        # Try to read some data
        print("📡 Reading data...")
        data_received = False
        for _ in range(10):
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(f"📊 Received: {line}")
                data_received = True
            time.sleep(0.1)
        
        if data_received:
            print("✅ Successfully received data from COM12")
            # IMPORTANT: Don't close the connection, save it for later use
            global_serial_connection = ser
            direct_test_completed = True  # Mark test as completed
            return True
        else:
            print("⚠️ Connected but no data received")
            ser.close()
            direct_test_completed = True  # Mark test as completed
            return False
            
    except Exception as e:
        print(f"❌ Direct serial test failed: {e}")
        direct_test_completed = True  # Mark test as completed
        return False



logging.basicConfig(level=logging.INFO,
                   format='%(asctime)s - %(levelname)s - %(message)s')

def global_error_handler(type, value, traceback):
    print(f"❌ Uncaught error: {value}")
    cleanup_resources()
    sys.__excepthook__(type, value, traceback)

sys.excepthook = global_error_handler

def ensure_port_available(port='COM12'):
    """Ensure the port is available before attempting connection"""
    print(f"🧹 Ensuring {port} is available...")
    
    # First try to close any existing connections
    try:
        import serial
        temp_ser = serial.Serial(port)
        temp_ser.close()
        print(f"✅ Closed existing connection to {port}")
    except:
        pass
    
    # Kill any Python processes that might be using serial ports
    try:
        import psutil
        import os
        current_pid = os.getpid()
        killed = False
        
        for proc in psutil.process_iter(['pid', 'name']):
            try:
                if proc.pid != current_pid and proc.name().lower() in ['python.exe', 'pythonw.exe']:
                    print(f"🔫 Terminating {proc.name()} (PID: {proc.pid})")
                    proc.kill()
                    killed = True
            except:
                pass
                
        if killed:
            time.sleep(2)  # Wait for processes to terminate
            
    except Exception as e:
        print(f"⚠️ Process cleanup warning: {e}")
    
    # Use mode command to reset port
    try:
        import os
        os.system(f"mode {port} BAUD=115200 PARITY=N DATA=8 STOP=1")
        print(f"✅ Reset {port} settings")
    except:
        pass
    
    time.sleep(1)  # Give OS time to release
    return True

def comprehensive_port_release(port='COM12'):
    """Comprehensive approach to release a COM port"""
    print(f"🔓 Attempting comprehensive release of {port}...")
    
    # 1. Try to close any existing connections
    try:
        import serial
        temp_ser = serial.Serial(port)
        temp_ser.close()
        print("✅ Closed existing connection")
    except:
        pass
    
    # 2. Reset port using mode command
    try:
        import os
        os.system(f"mode {port} BAUD=115200 PARITY=N DATA=8 STOP=1")
        print("✅ Reset port settings")
    except:
        pass
    
    # 3. Kill potential blocking processes
    try:
        import psutil
        import os
        current_pid = os.getpid()
        for proc in psutil.process_iter(['pid', 'name']):
            try:
                if proc.pid != current_pid and proc.name().lower() in [
                    'python.exe', 'pythonw.exe', 'reachview.exe', 
                    'putty.exe', 'terraterm.exe'
                ]:
                    print(f"🔫 Terminating {proc.name()} (PID: {proc.pid})")
                    proc.kill()
            except:
                pass
        print("✅ Cleaned up processes")
    except:
        pass
    
    # 4. Wait for OS to fully release the port
    import time
    time.sleep(3)
    print("✅ Port release complete")
    return True

def robust_emlid_connection(port='COM12'):
    """More robust Emlid connection method"""
    print(f"🔌 Attempting robust connection to {port}...")
    
    # First force release the port
    comprehensive_port_release(port)
    
    # Try with direct file access first
    try:
        import serial
        import time
        
        # Open with exclusive access and minimal settings
        ser = serial.Serial(
            port=port,
            baudrate=115200,
            timeout=1,
            write_timeout=1,
            exclusive=True
        )
        
        # Test if we can read data
        print("📡 Testing data reception...")
        time.sleep(1)
        
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            print(f"✅ Received {len(data)} bytes")
            ser.close()
            time.sleep(1)
            return True
        
        ser.close()
        time.sleep(1)
    except Exception as e:
        print(f"❌ Connection error: {e}")
    
    return False

# Call this before attempting to connect


def is_port_permanently_blocked(port='COM12'):
    """
    Performs comprehensive checks to determine if a port is permanently blocked
    and cannot be accessed programmatically.
    
    Returns:
        tuple: (blocked, reason)
    """
    import serial
    import subprocess
    import os
    
    print(f"🔍 Checking if {port} is permanently blocked...")
    
    # Check 1: Basic port existence
    try:
        ports = list(serial.tools.list_ports.comports())
        port_exists = any(p.device == port for p in ports)
        if not port_exists:
            return True, f"{port} does not exist in the system"
    except:
        pass
    
    # Check 2: Try with different access modes
    for mode in ['r', 'r+', 'w', 'w+']:
        try:
            # Try to open the port as a file (low-level)
            with open(f"\\\\.\\{port}", mode) as f:
                return False, "Port can be accessed at file level"
        except:
            pass
    
    # Check 3: Try with different serial settings
    for baudrate in [9600, 115200, 57600]:
        for timeout in [0.1, 1.0]:
            try:
                ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
                ser.close()
                return False, "Port can be accessed with serial library"
            except:
                pass
    
    # Check 4: Check if it's a system-reserved port
    try:
        result = subprocess.run(["powershell", "-Command", 
                               f"Get-WmiObject Win32_SerialPort | Where-Object {{$_.DeviceID -eq '{port}'}} | Select-Object Name,PNPDeviceID"],
                              capture_output=True, text=True)
        if "System" in result.stdout:
            return True, f"{port} is reserved by the system"
    except:
        pass
    
    return True, f"{port} appears to be permanently blocked"

# Add after the imports
def display_gps_info(emlid_data):
    """Display current GPS information"""
    print("\n=== GPS Status ===")
    print(f"Fix Type: {emlid_data.get('solution_status', 'Unknown')}")
    print(f"Satellites: {emlid_data.get('satellites', 0)}")
    print(f"HDOP: {emlid_data.get('hdop', 0.0):.2f}")
    print(f"Position: {emlid_data.get('latitude', 0.0):.8f}°N, "
          f"{emlid_data.get('longitude', 0.0):.8f}°E")
    print("================\n")

def degrees_to_cardinal_16(degrees):
    """Convert degrees to 16-point compass direction"""
    if degrees == 'N/A' or degrees is None:
        return 'N/A'
    
    # Normalize degrees to 0-360
    degrees = degrees % 360
    
    # 16-point compass with 22.5 degree intervals - full names
    directions = [
        'North', 'North-Northeast', 'Northeast', 'East-Northeast',
        'East', 'East-Southeast', 'Southeast', 'South-Southeast',
        'South', 'South-Southwest', 'Southwest', 'West-Southwest',
        'West', 'West-Northwest', 'Northwest', 'North-Northwest'
    ]
    
    # Calculate index (each direction covers 22.5 degrees)
    index = int((degrees + 11.25) / 22.5) % 16
    return directions[index]

def display_gps_status():
    """Display GPS status, current position, and navigation information every 100 milliseconds"""
    from datetime import datetime
    import csv
    import time
    import os
    
    # Create CSV header if needed - inline header creation
    headers = [
        'Timestamp',
        'GPS_Fix_Quality',
        'Satellites_Visible',
        'Satellites_Used',
        'HDOP',
        'PDOP', 
        'VDOP',
        'Latitude',
        'Longitude',
        'Altitude_m',
        'Lat_Error_m',
        'Lon_Error_m',
        'Alt_Error_m',
        'UTM_X',
        'UTM_Y',
        'Heading_Degrees',
        'Direction_16_Point'
    ]
    
    # Check if file exists and has content
    if not os.path.exists(csv_file) or os.path.getsize(csv_file) == 0:
        with open(csv_file, 'w', newline='') as csvfile:
            gps_writer = csv.writer(csvfile)
            gps_writer.writerow(headers)
            print(f"Created CSV header in {csv_file}")
    
    log_interval = 0.1  # Log every 0.1 second
    last_log_time = time.time()
    
    while True:
        try:
            current_time = time.time()
            
            # Always log to CSV every 0.1 seconds, even if no GPS connection
            if current_time - last_log_time >= log_interval:
                with open(csv_file, 'a', newline='') as csvfile:
                    gps_writer = csv.writer(csvfile)
                    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                    
                    # Check if rover and GPS are available
                    if rover and hasattr(rover, 'gps_reader') and rover.gps_reader:
                        position = rover.gps_reader.last_position
                        
                        if position:
                            gps_fix = position.get('fix_quality', 'Unknown')
                            satellites = position.get('satellites', 0)
                            satellites_used = len(position.get('satellites_used', []))
                            hdop = position.get('hdop', 99.9)
                            pdop = position.get('pdop', 99.9)
                            vdop = position.get('vdop', 99.9)
                            latitude = position.get('latitude', 0.0)
                            longitude = position.get('longitude', 0.0)
                            altitude = position.get('altitude', 0.0)
                            lat_error = position.get('lat_error', 0.0)
                            lon_error = position.get('lon_error', 0.0)
                            alt_error = position.get('alt_error', 0.0)
                            utm_x = rover.x
                            utm_y = rover.y
                            heading = position.get('heading', 'N/A')
                            direction_16 = degrees_to_cardinal_16(heading)
                        else:
                            # GPS reader exists but no position data
                            gps_fix = 'No Fix'
                            satellites = 0
                            satellites_used = 0
                            hdop = 'N/A'
                            pdop = 'N/A'
                            vdop = 'N/A'
                            latitude = 'N/A'
                            longitude = 'N/A'
                            altitude = 'N/A'
                            lat_error = 'N/A'
                            lon_error = 'N/A'
                            alt_error = 'N/A'
                            utm_x = 'N/A'
                            utm_y = 'N/A'
                            heading = 'N/A'
                            direction_16 = 'N/A'
                    else:
                        # No rover or GPS reader available
                        gps_fix = 'No Connection'
                        satellites = 'N/A'
                        satellites_used = 'N/A'
                        hdop = 'N/A'
                        pdop = 'N/A'
                        vdop = 'N/A'
                        latitude = 'N/A'
                        longitude = 'N/A'
                        altitude = 'N/A'
                        lat_error = 'N/A'
                        lon_error = 'N/A'
                        alt_error = 'N/A'
                        utm_x = 'N/A'
                        utm_y = 'N/A'
                        heading = 'N/A'
                        direction_16 = 'N/A'
                    
                    # Write row to CSV
                    gps_writer.writerow([
                        timestamp, gps_fix, satellites, satellites_used, hdop, pdop, vdop,
                        latitude, longitude, altitude, lat_error, lon_error, alt_error,
                        utm_x, utm_y, heading, direction_16
                    ])
                    
                    # Debug message - show more detailed info
                    status_msg = 'Data logged'
                    if rover and hasattr(rover, 'gps_reader') and rover.gps_reader:
                        if rover.gps_reader.last_position:
                            pos = rover.gps_reader.last_position
                            fix_quality = pos.get('fix_quality', 'Unknown')
                            sats_visible = pos.get('satellites', 0)
                            sats_used = len(pos.get('satellites_used', []))
                            pdop_val = pos.get('pdop', 99.9)
                            vdop_val = pos.get('vdop', 99.9)
                            status_msg += f' (GPS: {fix_quality}, Sats: {sats_visible}/{sats_used}, PDOP: {pdop_val:.1f}, VDOP: {vdop_val:.1f})'
                        else:
                            status_msg += ' (GPS No Fix)'
                    else:
                        status_msg += ' (GPS Disconnected)'
                    
                    print(f"CSV Log [{timestamp}]: {status_msg}")
                    
                last_log_time = current_time
            
            # Enhanced real-time display with better formatting
            print(f"\n=== ROVER STATUS [{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] ===")
            
            if rover and hasattr(rover, 'gps_reader') and rover.gps_reader:
                position = rover.gps_reader.last_position
                
                if position:
                    fix_quality = position.get('fix_quality', 'Unknown')
                    satellites_visible = position.get('satellites', 0)
                    satellites_used_list = position.get('satellites_used', [])
                    satellites_used_count = len(satellites_used_list)
                    hdop = position.get('hdop', 99.9)
                    pdop = position.get('pdop', 99.9)
                    vdop = position.get('vdop', 99.9)
                    
                    # Color coding for fix quality
                    if fix_quality == 'RTK Fixed':
                        fix_display = f"🟢 {fix_quality}"
                    elif fix_quality == 'RTK Float':
                        fix_display = f"🟡 {fix_quality}"
                    elif fix_quality in ['GPS', 'DGPS']:
                        fix_display = f"🔴 {fix_quality}"
                    else:
                        fix_display = f"⚫ {fix_quality}"
                    
                    print(f"GPS Fix: {fix_display}")
                    print(f"Satellites Visible: {satellites_visible}")
                    print(f"Satellites Used: {satellites_used_count}")
                    if satellites_used_list:
                        print(f"Satellite PRNs: {satellites_used_list}")
                    
                    # DOP values with quality indicators
                    print(f"HDOP: {hdop:.2f} {'🟢' if hdop < 2.0 else '🟡' if hdop < 5.0 else '🔴'}")
                    print(f"PDOP: {pdop:.2f} {'🟢' if pdop < 3.0 else '🟡' if pdop < 6.0 else '🔴'}")
                    print(f"VDOP: {vdop:.2f} {'🟢' if vdop < 3.0 else '🟡' if vdop < 6.0 else '🔴'}")
                    
                    print(f"Position (Lat/Lon): {position.get('latitude', 0.0):.8f}, {position.get('longitude', 0.0):.8f}")
                    print(f"Altitude: {position.get('altitude', 0.0):.3f}m")
                    
                    # Position errors
                    lat_err = position.get('lat_error', 0.0)
                    lon_err = position.get('lon_error', 0.0)
                    alt_err = position.get('alt_error', 0.0)
                    print(f"Position Errors - Lat: {lat_err:.3f}m, Lon: {lon_err:.3f}m, Alt: {alt_err:.3f}m")
                    
                    print(f"Current Position (UTM): {rover.x:.3f}, {rover.y:.3f}")
                    
                    if 'heading' in position and position['heading'] != 'N/A':
                        heading = position['heading']
                        direction_16 = degrees_to_cardinal_16(heading)
                        print(f"Heading: {heading:.1f}° ({direction_16})")
                    else:
                        print("Heading: N/A")
                        
                    # Additional RTK-specific info
                    if 'mode' in position:
                        print(f"GPS Mode: {position['mode']} (A=Auto, M=Manual)")
                    if 'fix_type' in position:
                        fix_type_map = {1: 'No Fix', 2: '2D Fix', 3: '3D Fix'}
                        print(f"Fix Type: {fix_type_map.get(position['fix_type'], 'Unknown')}")
                        
                else:
                    print("GPS Fix: No Fix Available")
                    print("Satellites Visible: 0")
                    print("Satellites Used: 0")
                    print("HDOP: N/A")
                    print("PDOP: N/A")  
                    print("VDOP: N/A")
                    print("Position (Lat/Lon): N/A")
                    print("Position Errors: N/A")
                    print("Current Position (UTM): N/A")
                    print("Heading: N/A")
            else:
                print("GPS Status: DISCONNECTED")
                print("GPS Fix: No Connection")
                print("Satellites Visible: N/A")
                print("Satellites Used: N/A")
                print("HDOP: N/A")
                print("PDOP: N/A")
                print("VDOP: N/A")
                print("Position (Lat/Lon): N/A")
                print("Position Errors: N/A")
                print("Current Position (UTM): N/A")
                print("Heading: N/A")

            # Navigation information
            if rover and hasattr(rover, 'navigator') and rover.navigator and rover.navigator.interpolated_path:
                if rover.navigator.current_waypoint_index < len(rover.navigator.interpolated_path):
                    next_wp = rover.navigator.interpolated_path[rover.navigator.current_waypoint_index]
                    heading = rover.navigator.calculate_heading((rover.x, rover.y), next_wp)
                    distance = rover.distance_to(next_wp[0], next_wp[1])
                    direction_16 = degrees_to_cardinal_16(heading)
                    print(f"Next Waypoint: {next_wp[0]:.3f}, {next_wp[1]:.3f}")
                    print(f"Heading to Waypoint: {heading:.1f}° ({direction_16})")
                    print(f"Distance to Waypoint: {distance:.3f}m")
                else:
                    print("Navigation complete")
            else:
                print("No navigation path set")
            
            print("====================\n")
            time.sleep(0.1)  # Update every 100 milliseconds
            
        except Exception as e:
            print(f"Status display error: {e}")
            # Still log error state to CSV
            try:
                with open(csv_file, 'a', newline='') as csvfile:
                    gps_writer = csv.writer(csvfile)
                    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                    gps_writer.writerow([
                        timestamp, 'ERROR', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A',
                        'N/A', 'N/A', 'N/A', 'N/A', 'N/A', 'N/A',
                        'N/A', 'N/A', 'N/A', 'N/A'
                    ])
            except:
                pass
            time.sleep(0.1)



# Add after the imports section
rover = None
ntrip_client = None
failsafe = None
gps_thread = None

def check_gps_status():
    """Check GPS quality and connection status"""
    global rover
    if not rover or not hasattr(rover, 'gps_reader'):
        return False
    
    position = rover.gps_reader.last_position
    if not position:
        return False
        
    satellites = position.get('satellites', 0)
    hdop = position.get('hdop', 99.9)
    fix_type = position.get('solution_status', 'Unknown')
    
    # Log GPS status
    print(f"\nGPS Status:")
    print(f"Fix Type: {fix_type}")
    print(f"Satellites: {satellites}")
    print(f"HDOP: {hdop:.1f}")
    
    # Check minimum requirements
    return (satellites >= 4 and 
            hdop < 5.0 and 
            fix_type in ['GPS', 'DGPS', 'RTK Fixed', 'RTK Float'])


def gps_reading_loop():
    """GPS data reading loop for real or simulated data"""
    while True:
        try:
            if hasattr(rover, 'gps_reader') and rover.gps_reader:  # ADDED CHECK
                if rover.gps_reader.simulate_gps:
                    fake = simulate_emlid_gps_reading()
                    logging_100mm.update_rover_position_from_emlid(rover, fake)
                    display_gps_info(fake)
                else:
                    # Real GPS data
                    if rover.gps_reader.last_position:
                        display_gps_info(rover.gps_reader.last_position)
            time.sleep(1.0)
        except Exception as e:
            print(f"GPS reading error: {e}")
            time.sleep(1.0)
def setup_ntrip(rover, emlid_reader):
    """Setup NTRIP connection and corrections"""
    try:
        NTRIP_CONFIG = {
            'host': 'your.ntrip.server.url',
            'port': 2101,
            'mountpoint': 'MOUNTPOINT',
            'user': 'your_username',
            'password': 'your_password'
        }
        
        ntrip_client = NTRIPClient(**NTRIP_CONFIG)
        
        # Test NTRIP connection
        if not ntrip_client.connect():
            raise Exception("Failed to connect to NTRIP server")
            
        # Start RTCM corrections thread
        def stream_rtcm():
            for rtcm_data in ntrip_client.get_corrections():
                if hasattr(rover, 'gps_reader'):
                    rover.gps_reader.send_rtcm_data(rtcm_data)
                    
        ntrip_thread = threading.Thread(target=stream_rtcm, daemon=True)
        ntrip_thread.start()
        
        return True
    except Exception as e:
        print(f"NTRIP setup failed: {e}")
        return False

def setup_gps_with_optional_ntrip(rover, use_ntrip=False):
    """Setup GPS with optional NTRIP - SIMPLIFIED VERSION based on working test script"""
    try:
        print("\n📡 Setting up Emlid GPS integration...")
        
        # Initialize Emlid reader with minimal configuration
        emlid_reader = EmlidGPSReader(port='COM12', baud_rate=115200, message_format='nmea')
        emlid_reader.simulate_gps = False
        
        # Register callback before connection
        update_rover_from_emlid(rover, emlid_reader)
        print("✅ Callback registered for Emlid GPS data")
        
        # Try the simple connection approach first (like your test script)
        print("\n🔌 Attempting to connect to Emlid GPS...")
        if hasattr(emlid_reader, 'connect_with_simple_approach'):
            connection_success = emlid_reader.connect_with_simple_approach()
        else:
            # Fallback to regular connect if the new method isn't available
            connection_success = emlid_reader.connect()
            
        if connection_success:
            print(f"✅ Connected to Emlid M2 on COM12")
            
            # Start reading with immediate verification
            if emlid_reader.start_reading():
                rover.gps_reader = emlid_reader
                print("📡 GPS data reading started")
                
                # Verify data reception
                print("🔍 Verifying data reception...")
                for _ in range(3):  # Check 3 times
                    time.sleep(1)
                    if emlid_reader.last_position:
                        print("✅ Live GPS data confirmed")
                        return True
                
                print("⚠️ No position data detected yet, but connection is active")
                return True
            else:
                print("❌ Failed to start GPS reading")
        else:
            print("❌ Connection failed")
        
        # If we get here, connection failed - clean up
        try:
            if hasattr(emlid_reader, 'stop_reading'):
                emlid_reader.stop_reading()
            if hasattr(emlid_reader, 'disconnect'):
                emlid_reader.disconnect()
        except:
            pass
        
        # Simulation fallback
        print("\n📡 GPS Connection Failed")
        print("Options:")
        print("1) Continue with GPS simulation")
        print("2) Abort")
        
        while True:
            try:
                choice = input("Select option (1/2): ").strip()
                if choice == '1':
                    print("\n🧪 Initializing GPS simulation...")
                    emlid_reader = EmlidGPSReader(port='COM12', message_format='nmea')
                    emlid_reader.simulate_gps = True
                    rover.gps_reader = emlid_reader
                    update_rover_from_emlid(rover, emlid_reader)
                    
                    def simulation_thread():
                        while True:
                            try:
                                fake_data = simulate_emlid_gps_reading()
                                for callback in emlid_reader.callbacks:
                                    try:
                                        callback(fake_data)
                                    except Exception as e:
                                        print(f"Callback error: {e}")
                                time.sleep(0.1)
                            except Exception as e:
                                print(f"Simulation error: {e}")
                                time.sleep(1)
                    
                    sim_thread = threading.Thread(target=simulation_thread, daemon=True)
                    sim_thread.start()
                    print("✅ GPS simulation active")
                    return False
                
                elif choice == '2':
                    print("\n❌ Setup aborted by user")
                    return False
                
                else:
                    print("Invalid choice - enter 1 or 2")
            
            except Exception as input_e:
                print(f"Input error: {input_e}")
    
    except Exception as e:
        print(f"\n❌ GPS setup error: {e}")
        import traceback
        traceback.print_exc()
        return False
debug = False
safety = SafetyModule()

def setup_gps_direct_approach(rover):
    """Setup GPS using the already open serial connection"""
    global global_serial_connection
    
    print("\n📡 Setting up GPS with direct serial approach...")
    
    # Check if we already have an open connection
    if not global_serial_connection or not global_serial_connection.is_open:
        print("❌ No open serial connection available")
        return False
    
    try:
        # Create a minimal EmlidGPSReader that just uses our working connection
        emlid_reader = EmlidGPSReader(port='COM12', baud_rate=115200)
        emlid_reader.serial_connection = global_serial_connection
        emlid_reader.simulate_gps = False
        
        # Register callback
        update_rover_from_emlid(rover, emlid_reader)
        
        # Create a custom reading thread
        def custom_reading_thread():
            error_count = 0
            max_errors = 10
            satellites_in_view = {}  # Use dict to avoid duplicates, key = PRN
            satellites_used = []     # Track satellites used in solution
            constellation_stats = {}  # Track stats per constellation
            
            while True:
                try:
                    if not global_serial_connection:
                        print("❌ No serial connection available")
                        time.sleep(1)
                        continue
                        
                    if not global_serial_connection.is_open:
                        print("❌ Serial connection is closed, attempting to reopen")
                        try:
                            global_serial_connection.open()
                            print("✅ Reopened serial connection")
                        except Exception as open_err:
                            print(f"❌ Failed to reopen connection: {open_err}")
                            time.sleep(1)
                            continue
                    
                    # Read NMEA data directly
                    try:
                        line = global_serial_connection.readline().decode('utf-8', errors='ignore').strip()
                        if line:
                            current_time = time.time()
                            if not hasattr(custom_reading_thread, 'last_print_time') or current_time - custom_reading_thread.last_print_time >= 2.0:
                                print(f"📡 NMEA: {line}")
                                custom_reading_thread.last_print_time = current_time
                            
                            # Parse ALL GSV messages - ENHANCED to catch more constellations
                            if 'GSV' in line and line.startswith('$'):
                                parts = line.split(',')
                                if len(parts) >= 4:
                                    try:
                                        constellation = line[:6]  # e.g., $GPGSV, $GNGSV, $GLGSV, etc.
                                        total_messages = int(parts[1]) if parts[1] else 0
                                        message_number = int(parts[2]) if parts[2] else 0
                                        total_sats_reported = int(parts[3]) if parts[3] else 0
                                        
                                        # Initialize constellation tracking
                                        if constellation not in constellation_stats:
                                            constellation_stats[constellation] = {
                                                'expected_messages': total_messages,
                                                'received_messages': 0,
                                                'satellites_count': 0
                                            }
                                        
                                        # If this is message 1, reset the constellation data
                                        if message_number == 1:
                                            constellation_stats[constellation] = {
                                                'expected_messages': total_messages,
                                                'received_messages': 0,
                                                'satellites_count': total_sats_reported
                                            }
                                            
                                            # Remove old satellites from this constellation
                                            constellation_prefixes = {
                                                '$GPGSV': range(1, 33),      # GPS: PRN 1-32
                                                '$GLGSV': range(65, 97),     # GLONASS: PRN 65-96 (sometimes 1-24)
                                                '$GAGSV': range(301, 337),   # Galileo: PRN 301-336 (sometimes 1-36)
                                                '$GBGSV': range(401, 438),   # BeiDou: PRN 401-437 (sometimes 1-37)
                                                '$GQGSV': range(201, 237),   # QZSS: PRN 201-237 (sometimes 1-10)
                                                '$GIGSV': range(501, 537),   # IRNSS: PRN 501-537
                                                '$GNGSV': range(1, 600)      # Mixed: could be any
                                            }
                                            
                                            # More flexible PRN removal - remove by constellation type
                                            if constellation in constellation_prefixes:
                                                prn_range = constellation_prefixes[constellation]
                                                # Remove satellites that might belong to this constellation
                                                satellites_in_view = {k: v for k, v in satellites_in_view.items() 
                                                                    if not (v.get('constellation') == constellation)}
                                        
                                        # Parse satellite info (up to 4 satellites per GSV message)
                                        satellites_in_this_message = 0
                                        for i in range(4, min(len(parts), 20), 4):
                                            if i + 3 < len(parts):
                                                sat_prn = parts[i].strip() if parts[i] else None
                                                elevation = parts[i + 1].strip() if parts[i + 1] else None
                                                azimuth = parts[i + 2].strip() if parts[i + 2] else None
                                                snr = parts[i + 3].split('*')[0].strip() if parts[i + 3] else None
                                                
                                                if sat_prn and sat_prn != '':
                                                    try:
                                                        prn = int(sat_prn)
                                                        # Create unique key combining constellation and PRN
                                                        sat_key = f"{constellation}_{prn}"
                                                        
                                                        sat_info = {
                                                            'prn': prn,
                                                            'constellation': constellation,
                                                            'elevation': int(elevation) if elevation and elevation != '' else 0,
                                                            'azimuth': int(azimuth) if azimuth and azimuth != '' else 0,
                                                            'snr': int(snr) if snr and snr != '' else 0,
                                                            'key': sat_key
                                                        }
                                                        satellites_in_view[sat_key] = sat_info
                                                        satellites_in_this_message += 1
                                                    except ValueError as ve:
                                                        print(f"⚠️ Error parsing satellite PRN '{sat_prn}': {ve}")
                                        
                                        # Update message tracking
                                        constellation_stats[constellation]['received_messages'] = message_number
                                        
                                        # Update position data with total satellite count
                                        if emlid_reader.last_position is None:
                                            emlid_reader.last_position = {}
                                        
                                        total_sats_visible = len(satellites_in_view)
                                        emlid_reader.last_position['satellites'] = total_sats_visible
                                        emlid_reader.last_position['satellites_in_view'] = list(satellites_in_view.values())
                                        
                                        # Detailed logging every few seconds
                                        if not hasattr(custom_reading_thread, 'last_detailed_log') or current_time - custom_reading_thread.last_detailed_log >= 3.0:
                                            print(f"🛰️ {constellation}: Msg {message_number}/{total_messages}, Reported: {total_sats_reported}, This msg: {satellites_in_this_message}")
                                            print(f"🛰️ Total satellites visible across all constellations: {total_sats_visible}")
                                            
                                            # Show breakdown by constellation
                                            constellation_breakdown = {}
                                            for sat_key, sat_info in satellites_in_view.items():
                                                const = sat_info['constellation']
                                                if const not in constellation_breakdown:
                                                    constellation_breakdown[const] = 0
                                                constellation_breakdown[const] += 1
                                            
                                            for const, count in constellation_breakdown.items():
                                                print(f"   {const}: {count} satellites")
                                            
                                            custom_reading_thread.last_detailed_log = current_time
                                        
                                    except Exception as gsv_e:
                                        print(f"GSV parsing error for {line}: {gsv_e}")
                            
                            # Parse $GPGGA or $GNGGA for position data
                            elif line.startswith(('$GPGGA', '$GNGGA')):
                                parts = line.split(',')
                                if len(parts) >= 15 and parts[6] != '0' and parts[2] and parts[4]:
                                    try:
                                        # Convert DDMM.MMMM to decimal degrees
                                        lat_raw = parts[2]
                                        lon_raw = parts[4]
                                        
                                        lat_deg = int(lat_raw[:2])
                                        lat_min = float(lat_raw[2:])
                                        latitude = lat_deg + lat_min / 60.0
                                        
                                        lon_deg = int(lon_raw[:3])
                                        lon_min = float(lon_raw[3:])
                                        longitude = lon_deg + lon_min / 60.0
                                        
                                        if parts[3] == 'S':
                                            latitude = -latitude
                                        if parts[5] == 'W':
                                            longitude = -longitude
                                        
                                        # Map fix quality to proper RTK status
                                        fix_quality_map = {
                                            '0': 'Invalid',
                                            '1': 'GPS',
                                            '2': 'DGPS',
                                            '3': 'PPS',
                                            '4': 'RTK Fixed',
                                            '5': 'RTK Float',
                                            '6': 'Estimated',
                                            '7': 'Manual',
                                            '8': 'Simulation'
                                        }
                                        
                                        position = {
                                            'latitude': latitude,
                                            'longitude': longitude,
                                            'altitude': float(parts[9]) if parts[9] else 0.0,
                                            'satellites': len(satellites_in_view),  # Total satellites visible
                                            'hdop': float(parts[8]) if parts[8] else 99.9,
                                            'fix_quality': fix_quality_map.get(parts[6], 'Unknown'),
                                            'solution_status': fix_quality_map.get(parts[6], 'Unknown')
                                        }
                                        
                                        if emlid_reader.last_position is None:
                                            emlid_reader.last_position = {}
                                        emlid_reader.last_position.update(position)
                                        emlid_reader.last_update_time = time.time()
                                        
                                        for callback in emlid_reader.callbacks:
                                            callback(emlid_reader.last_position)
                                    except Exception as parse_e:
                                        print(f"NMEA parsing error: {parse_e}")
                            
                            # Parse $GPRMC or $GNRMC for heading
                            elif line.startswith(('$GPRMC', '$GNRMC')):
                                parts = line.split(',')
                                if len(parts) >= 10 and parts[2] == 'A':
                                    try:
                                        cog = float(parts[8]) if parts[8] else 0.0
                                        if emlid_reader.last_position is None:
                                            emlid_reader.last_position = {}
                                        emlid_reader.last_position['heading'] = cog
                                        emlid_reader.last_update_time = time.time()
                                        for callback in emlid_reader.callbacks:
                                            callback(emlid_reader.last_position)
                                    except Exception as parse_e:
                                        print(f"$GPRMC parsing error: {parse_e}")
                            
                            # Parse ALL GSA messages for satellites used and DOP
                            elif 'GSA' in line and line.startswith('$'):
                                parts = line.split(',')
                                if len(parts) >= 18:
                                    try:
                                        constellation_gsa = line[:6]  # e.g., $GPGSA, $GNGSA, $GLGSA
                                        mode = parts[1]  # A = Auto, M = Manual
                                        fix_type = int(parts[2]) if parts[2] else 0  # 1=No fix, 2=2D, 3=3D
                                        
                                        # Extract satellites used (positions 3-14)
                                        current_satellites_used = []
                                        for i in range(3, 15):  # Positions 3-14 contain satellite PRNs
                                            if i < len(parts) and parts[i] and parts[i].strip():
                                                try:
                                                    sat_prn = int(parts[i].strip())
                                                    current_satellites_used.append(sat_prn)
                                                except ValueError:
                                                    pass
                                        
                                        # For GNGSA (multi-constellation), this gives us the total used
                                        if constellation_gsa == '$GNGSA':
                                            satellites_used = current_satellites_used
                                        else:
                                            # For single constellation GSA, add to the list
                                            for sat in current_satellites_used:
                                                if sat not in satellites_used:
                                                    satellites_used.append(sat)
                                        
                                        # Extract DOP values
                                        pdop = float(parts[15]) if len(parts) >= 16 and parts[15] else 99.9
                                        hdop = float(parts[16]) if len(parts) >= 17 and parts[16] else 99.9
                                        vdop_str = parts[17].split('*')[0] if len(parts) >= 18 and parts[17] else '99.9'
                                        vdop = float(vdop_str) if vdop_str else 99.9
                                        
                                        if emlid_reader.last_position is None:
                                            emlid_reader.last_position = {}
                                        
                                        emlid_reader.last_position.update({
                                            'mode': mode,
                                            'fix_type': fix_type,
                                            'satellites_used': satellites_used,
                                            'pdop': pdop,
                                            'hdop': hdop,
                                            'vdop': vdop
                                        })
                                        
                                        # Validation and detailed logging
                                        sats_visible = len(satellites_in_view)
                                        sats_used = len(satellites_used)
                                        
                                        if sats_visible < sats_used:
                                            print(f"⚠️ Warning: Satellites used ({sats_used}) > visible ({sats_visible})")
                                        
                                        # Enhanced logging for RTK analysis
                                        if not hasattr(custom_reading_thread, 'last_rtk_log') or current_time - custom_reading_thread.last_rtk_log >= 5.0:
                                            fix_quality = emlid_reader.last_position.get('fix_quality', 'Unknown')
                                            print(f"🛰️ RTK Status: {fix_quality}")
                                            print(f"🛰️ GSA ({constellation_gsa}): Mode={mode}, Fix={fix_type}")
                                            print(f"🛰️ Satellites: Visible={sats_visible}, Used={sats_used}")
                                            print(f"🛰️ DOP: PDOP={pdop:.1f}, HDOP={hdop:.1f}, VDOP={vdop:.1f}")
                                            
                                            # RTK quality assessment
                                            if fix_quality == 'RTK Fixed':
                                                print("🟢 Excellent RTK Fixed solution")
                                            elif fix_quality == 'RTK Float':
                                                print("🟡 Good RTK Float solution")
                                                if sats_used < 8:
                                                    print("   💡 Consider: More satellites could help achieve RTK Fixed")
                                            elif fix_quality in ['GPS', 'DGPS']:
                                                print("🔴 Basic GPS solution - RTK corrections may not be working")
                                                print("   💡 Check NTRIP connection and base station distance")
                                            
                                            # Satellite usage analysis
                                            if sats_visible >= 20 and sats_used < 10:
                                                print(f"   💡 Many satellites visible ({sats_visible}) but few used ({sats_used})")
                                                print("   💡 This is normal - receiver selects best satellites for solution")
                                            elif sats_used >= 12:
                                                print(f"   ✅ Good satellite usage: {sats_used} satellites")
                                            
                                            custom_reading_thread.last_rtk_log = current_time
                                        
                                        for callback in emlid_reader.callbacks:
                                            callback(emlid_reader.last_position)
                                    except Exception as parse_e:
                                        print(f"$GSA parsing error: {parse_e}")
                            
                            # Parse $GPGST or $GNGST for position errors
                            elif line.startswith(('$GPGST', '$GNGST')):
                                parts = line.split(',')
                                if len(parts) >= 9:
                                    try:
                                        lat_error = float(parts[6]) if parts[6] else 0.0
                                        lon_error = float(parts[7]) if parts[7] else 0.0
                                        alt_error_str = parts[8].split('*')[0] if parts[8] else '0.0'
                                        alt_error = float(alt_error_str) if alt_error_str else 0.0
                                        
                                        if emlid_reader.last_position is None:
                                            emlid_reader.last_position = {}
                                        emlid_reader.last_position.update({
                                            'lat_error': lat_error,
                                            'lon_error': lon_error,
                                            'alt_error': alt_error
                                        })
                                        for callback in emlid_reader.callbacks:
                                            callback(emlid_reader.last_position)
                                    except Exception as parse_e:
                                        print(f"$GST parsing error: {parse_e}")
                        
                        error_count = 0
                    except Exception as read_err:
                        error_count += 1
                        print(f"Read error ({error_count}/{max_errors}): {read_err}")
                        if error_count >= max_errors:
                            print("Too many read errors, resetting connection")
                            try:
                                global_serial_connection.close()
                                time.sleep(1)
                                global_serial_connection.open()
                                print("Connection reset complete")
                            except:
                                pass
                            error_count = 0
                        time.sleep(0.5)
                        continue
                        
                    time.sleep(0.05)  # Faster polling for more data
                    
                except Exception as e:
                    print(f"Reading thread error: {e}")
                    error_count += 1
                    if error_count >= max_errors:
                        print("Too many errors in reading thread, resetting")
                        error_count = 0
                    time.sleep(1.0)   
        
        # Start our custom reading thread
        reading_thread = threading.Thread(target=custom_reading_thread, daemon=True)
        reading_thread.start()
        emlid_reader.reading_thread = reading_thread
        
        # Assign to rover
        rover.gps_reader = emlid_reader
        
        print("✅ GPS setup complete with direct approach")
        return True 
        
    except Exception as e:
        print(f"❌ Direct GPS setup failed: {e}")
        return False





def enhanced_gps_status_monitor():
    """Enhanced GPS status monitor with health checks"""
    global rover
    last_health_report = 0

    while True:
        try:
            if rover and hasattr(rover, 'gps_reader') and rover.gps_reader:  # ADDED CHECK
                current_time = time.time()

                # Get health status
                health = rover.gps_reader.check_health()

                # Print detailed status every 10 seconds
                if current_time - last_health_report > 10:
                    print(f"\n=== GPS Health Report [{datetime.datetime.now().strftime('%H:%M:%S')}] ===")
                    print(f"Connected: {'✅' if health['connected'] else '❌'}")
                    print(f"Thread Alive: {'✅' if health['thread_alive'] else '❌'}")
                    print(f"Simulation Mode: {'🧪' if health['simulation_mode'] else '📡'}")
                    print(f"Last Update: {health['last_update']:.1f}s ago")

                    if rover.gps_reader.last_position:
                        pos = rover.gps_reader.last_position
                        print(f"Fix Quality: {pos.get('fix_quality', 'Unknown')}")
                        print(f"Satellites: {pos.get('satellites', 0)}")
                        print(f"HDOP: {pos.get('hdop', 99.9):.2f}")

                        # Quality indicators
                        if pos.get('fix_quality') == 'RTK Fixed':
                            print("🟢 Excellent RTK Fixed")
                        elif pos.get('fix_quality') == 'RTK Float':
                            print("🟡 Good RTK Float")
                        elif pos.get('fix_quality') in ['GPS', 'DGPS']:
                            print("🔴 Basic GPS Fix")
                        else:
                            print("⚫ Poor/No Fix")

                    print("=" * 50)
                    last_health_report = current_time

                # Check for problems
                if health['last_update'] > 10:  # No data for 10 seconds
                    print("⚠️ WARNING: No GPS data received for 10+ seconds")

                if not health['connected'] and not health['simulation_mode']:
                    print("⚠️ WARNING: GPS connection lost, attempting recovery...")
                    try:
                        rover.gps_reader.connect(retries=2, retry_delay=1)
                    except:
                        pass

            time.sleep(5)  # Update every 5 seconds

        except Exception as e:
            print(f"GPS monitor error: {e}")
            time.sleep(5)

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

def display_ntrip_status(ntrip_client):
        print("\n=== NTRIP Status ===")
        print(f"Connected: {ntrip_client.connected}")
        print("===================\n")
def handle_gps_error(error_message):
    """Handle GPS errors without disconnecting"""
    print(f"⚠️ GPS Error: {error_message}")
    print("Attempting recovery...")
    
    # Just reset the failsafe timers without disconnecting
    if hasattr(rover, 'failsafe'):
        rover.failsafe.last_gps_update = time.time()
        if hasattr(rover.failsafe, 'last_correction_update'):
            rover.failsafe.last_correction_update = time.time()
        
    return True  # Indicate successful recovery

def setup_gps_simple_approach(rover):
    """Setup GPS using the exact same approach as test.py"""
    print("\n📡 Setting up Emlid GPS with simple approach...")
    
    # First ensure all existing connections are closed
    try:
        from emlid_gps_integration import cleanup_all_gps_connections
        cleanup_all_gps_connections()
        time.sleep(2)  # Give OS time to release port
    except Exception as e:
        print(f"Cleanup error: {e}")
    
    # Force release the port
    force_release_com_port('COM12')
    
    # Check if port is permanently blocked
    blocked, reason = is_port_permanently_blocked('COM12')
    if blocked:
        print(f"❌ COM12 is permanently blocked: {reason}")
        print("   Cannot proceed with real GPS connection")
        return False
    
    try:
        # Create a simple serial connection like in test.py
        ser = serial.Serial(
            port='COM12',
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
        
        print("✅ Connected to COM12")
        
        # Initialize the EmlidGPSReader with this connection
        emlid_reader = EmlidGPSReader(port='COM12', baud_rate=115200)
        emlid_reader.serial_connection = ser  # Use our working connection
        
        # Register callback
        update_rover_from_emlid(rover, emlid_reader)
        
        # Start reading thread
        emlid_reader.start_reading()
        
        # Assign to rover
        rover.gps_reader = emlid_reader
        
        return True
    except Exception as e:
        print(f"❌ Simple GPS setup failed: {e}")
        return False
    
csv_dir = r'F:\GPS\task_2_waypoints'
csv_file = os.path.join(csv_dir, 'gps_status_log.csv')
if not os.path.exists(csv_dir):
    os.makedirs(csv_dir)
    print(f"✅ Created directory: {csv_dir}")
if not os.path.exists(csv_file):
    with open(csv_file, 'w', newline='') as csvfile:
        gps_writer = csv.writer(csvfile)
        gps_writer.writerow([
            'Timestamp', 'GPS Fix', 'Satellites', 'Satellites Used', 'HDOP', 'PDOP', 'VDOP',
            'Latitude', 'Longitude', 'Altitude', 'Lat Error', 'Lon Error', 'Alt Error',
            'UTM X', 'UTM Y', 'Heading'
        ])
def run_simulation():
    global rover, ntrip_client, failsafe, global_serial_connection 
    print("🧹 Initial cleanup of any existing GPS connections...")
    try:
        from emlid_gps_integration import cleanup_all_gps_connections
        cleanup_all_gps_connections()
    except Exception as e:
        print(f"Initial cleanup error: {e}")

    direct_serial_test()

    
        
    # Initialize gps_success variable with a default value
    gps_success = False

      # Only run the direct serial test if it hasn't been run yet
    if not direct_test_completed:
        direct_serial_test()
    
    def on_failsafe_triggered(reason):
        print(f"⚠️ Failsafe triggered: {reason.value}")
        rover.log_movement("stop")  # Stop the rover for safety

    # Update the on_recovery_attempt function
    def on_recovery_attempt(reason):
        print(f"🔄 Attempting recovery from {reason.value}")
        current_time = time.time()
        try:
            if reason == GPSFailsafeReason.GPS_STALE_DATA or reason == GPSFailsafeReason.GPS_DATA_LOSS:
                failsafe.last_gps_update = current_time
                # Don't disconnect/reconnect, just reset the timer
                print("Resetting GPS data timer without disconnecting")
                return True
            elif reason == GPSFailsafeReason.GPS_CORRECTION_STALE:
                failsafe.last_correction_update = current_time
                print("Resetting GPS correction timer without disconnecting")
                return True
            elif reason == GPSFailsafeReason.INTERNET_CONNECTION_LOST or reason == GPSFailsafeReason.INTERNET_CONNECTION_SLOW:
                failsafe.last_internet_check = current_time
                return True  # Continue without internet
            elif reason == GPSFailsafeReason.MODULE_COMMUNICATION_FAILURE:
                failsafe.last_module_comm = current_time
                return True
            return True
        except Exception as e:
            print(f"Recovery attempt failed: {e}")
            return False

   
    # Add periodic NTRIP status check

    print("🚜 Farm Rover Navigation Simulation 🚜")
    print("=====================================")
    sys.modules['__main__'].global_serial_connection = global_serial_connection
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
    
    # Inside run_simulation function, replace the GPS setup section with:

    # Enhanced GPS setup with comprehensive error handling
    print("\n📡 Setting up Emlid GPS integration...")
    # Initialize gps_success variable with a default value
    gps_success = False

    # Try our direct approach using the already open connection
    gps_success = setup_gps_direct_approach(rover)

    status_thread = threading.Thread(target=display_gps_status, daemon=True)
    status_thread.start()

    # If that fails, fall back to simulation
    if not gps_success:
        print("\n⚠️ GPS setup failed. Switching to simulation mode...")
        try:
            # Clean up any partial setup
            if hasattr(rover, 'gps_reader') and rover.gps_reader:
                rover.gps_reader.stop_reading()
                rover.gps_reader.disconnect()
            
            # Setup simulation
            emlid_reader = EmlidGPSReader(port='COM12', message_format='nmea')
            emlid_reader.simulate_gps = True
            rover.gps_reader = emlid_reader
            
            # Register callback for simulation
            update_rover_from_emlid(rover, emlid_reader)

            def gps_simulation_loop():
                while True:
                    try:
                        fake = simulate_emlid_gps_reading()
                        for callback in emlid_reader.callbacks:
                            try:
                                callback(fake)
                            except Exception as cb_e:
                                print(f"Simulation callback error: {cb_e}")
                        time.sleep(0.1)  # 10Hz simulation
                    except Exception as sim_e:
                        print(f"Simulation error: {sim_e}")
                        time.sleep(1.0)
            
            threading.Thread(target=gps_simulation_loop, daemon=True).start()
            print("🧪 GPS simulation started successfully")
            
        except Exception as sim_error:
            print(f"❌ Even simulation setup failed: {sim_error}")
            print("   This is a critical error - check your imports and dependencies")

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
    cleanup_resources()

def cleanup_resources():
    """Enhanced cleanup with better error handling"""
    global rover, ntrip_client, failsafe, gps_thread, global_serial_connection
    print("\nCleaning up resources...")

    try:
        if global_serial_connection and global_serial_connection.is_open:
            try:
                global_serial_connection.close()
                print("✅ Closed global serial connection")
            except Exception as e:
                print(f"⚠️ Error closing global serial connection: {e}")
            global_serial_connection = None
        # FIRST: Stop all GPS-related threads and connections
        if rover and hasattr(rover, 'gps_reader') and rover.gps_reader:  # ADDED CHECK
            print("Stopping GPS reader...")
            try:
                rover.gps_reader.stop_reading()  # Stop reading first
                time.sleep(1)  # Give it time
                rover.gps_reader.disconnect()   # Then disconnect
                time.sleep(1)  # Give OS time to release port
                print("✅ GPS disconnected successfully")
            except Exception as e:
                print(f"⚠️ GPS cleanup error: {e}")

        # Cleanup all GPS connections (using our new function)
        try:
            from emlid_gps_integration import cleanup_all_gps_connections
            cleanup_all_gps_connections()
        except Exception as e:
            print(f"⚠️ Global GPS cleanup error: {e}")

        if ntrip_client:
            print("Cleaning up NTRIP...")
            try:
                ntrip_client.cleanup()
            except Exception as e:
                print(f"⚠️ NTRIP cleanup error: {e}")
            ntrip_client = None

        if rover:
            print("Stopping GPS logger...")
            try:
                logging_100mm.stop_gps_logger(rover)
            except Exception as e:
                print(f"⚠️ Logger cleanup error: {e}")

        if failsafe:
            print("Stopping failsafe monitoring...")
            try:
                failsafe.stop_monitoring()
            except Exception as e:
                print(f"⚠️ Failsafe cleanup error: {e}")

        try:
            plt.close('all')
        except Exception as e:
            print(f"⚠️ Plot cleanup error: {e}")

        print("✅ Cleanup complete")
    except Exception as e:
        print(f"❌ Cleanup error: {e}")

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
        'hdop': hdop,                      # Horizontal dilution of precision
        'fix_quality': solution_status.upper(),  # Add fix_quality
        'altitude': random.uniform(200, 300),  # Add altitude
        'speed': random.uniform(0, 5),  # Add speed
        'heading': random.uniform(0, 360)  # Add heading
    }

def gps_status_monitor():
    """Monitor GPS status and quality"""
    global rover
    while True:
        try:
            if rover and hasattr(rover, 'gps_reader'):
                if rover.gps_reader.last_position:
                    fix_type = rover.gps_reader.last_position.get('solution_status', 'Unknown')
                    satellites = rover.gps_reader.last_position.get('satellites', 0)
                    hdop = rover.gps_reader.last_position.get('hdop', 0.0)
                    
                    if fix_type == 'fixed':
                        print("🟢 RTK Fixed")
                    elif fix_type == 'float':
                        print("🟡 RTK Float")
                    else:
                        print("🔴 No RTK")
                        
                    print(f"Satellites: {satellites}, HDOP: {hdop:.2f}")
            time.sleep(5)  # Update every 5 seconds
        except Exception as e:
            print(f"GPS status monitor error: {e}")
            time.sleep(1)

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
            zone_number=43, zone_letter='N'  # Make sure to use the correct zone
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
def force_release_com_port(port='COM12'):
    """Force release a COM port using mode command"""
    print(f"\n🔧 Forcing release of {port}...")
    try:
        # Try closing any existing connections
        try:
            temp_ser = serial.Serial(port)
            temp_ser.close()
        except:
            pass
            
        # Use mode command to reset port
        os.system(f"mode {port} BAUD=115200 PARITY=N DATA=8 STOP=1")
        time.sleep(2)  # Give OS time to release
        
        # Kill any Python processes that might be using serial ports
        import psutil
        current_pid = os.getpid()
        for proc in psutil.process_iter(['pid', 'name']):
            try:
                if proc.pid != current_pid and proc.name().lower() in ['python.exe', 'pythonw.exe']:
                    proc.kill()
            except:
                pass
                
        time.sleep(2)  # Wait for processes to terminate
        print("✅ Port cleanup complete")
        return True
    except Exception as e:
        print(f"⚠️ Port cleanup warning: {e}")
        return False


def test_com12_availability():
    """Test if COM12 is available"""
    print("🧪 Testing COM12 availability...")
    try:
        ser = serial.Serial(
            port='COM12',
            baudrate=115200,
            timeout=1,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE
        )
        print("✅ COM12 is available and opened successfully")
        ser.close()
    except serial.SerialException as e:
        print(f"❌ Failed to open COM12: {e}")
        if "Access is denied" in str(e):
            print("   🔍 COM12 is likely in use by another application or process.")
            print("   Please close other programs (e.g., ReachView, PuTTY) and try again.")
if __name__ == "__main__":
    try:
        print("🚜 Starting Farm Simulation...")
        
        # Force initial cleanup
        try:
            from emlid_gps_integration import cleanup_all_gps_connections
            cleanup_all_gps_connections()
            comprehensive_port_release('COM12')
            time.sleep(2)  # Give OS time to release ports
            print("✅ Initial cleanup complete")
        except Exception as e:
            print(f"⚠️ Cleanup warning: {e}")
        
        # First, try direct serial connection like in test.py
        real_gps = direct_serial_test()
        
        # Run simulation in simulation mode by default
        print("\n" + "=" * 50)
        if not real_gps:
            # Set simulation flag in GPS module
            import emlid_gps_integration
            emlid_gps_integration.simulate_gps = True
            print("🧪 Running in simulation mode")
        
        run_simulation()
        
    except KeyboardInterrupt:
        print("\n\n🛑 Simulation terminated by user")
    except Exception as e:
        print(f"\n❌ Simulation error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        cleanup_resources()