import time
from datetime import datetime
import csv
import os
import random
from threading import Thread, Event
from coordinate_converter import CoordinateConverter

class GPSLogger:
    def __init__(self, rover):
        self.rover = rover
        self.log_interval = 0.1  # 100 milliseconds
        self.max_attempts = 3    # Try 3 times before reporting an issue
        self.log_file_path = r'F:\GPS\task_2_waypoints\rover_log.csv'
        self.last_log_time = 0
        self.running = False
        self.stop_event = Event()
        self.logger_thread = None
        self.converter = CoordinateConverter()
        
        # Set run_id properly
        if not hasattr(self.rover, 'run_id'):
            self.rover.run_id = self.get_next_run_id()
            
        # Ensure log directory exists
        os.makedirs(os.path.dirname(self.log_file_path), exist_ok=True)
        
        # Define fieldnames consistently for use throughout the class
        self.fieldnames = ['timestamp', 'run_id', 'x_utm', 'y_utm', 'latitude', 'longitude', 
                          'heading', 'bearing', 'compass_heading', 'fix_quality', 
                          'satellite_count', 'deviation', 'data_age', 'status']
        
        # Write header if file doesn't exist
        if not os.path.exists(self.log_file_path) or os.path.getsize(self.log_file_path) == 0:
            with open(self.log_file_path, 'w', newline='') as csvfile:
                writer = csv.DictWriter(csvfile, fieldnames=self.fieldnames)
                writer.writeheader()
    
    def get_next_run_id(self):
        """Determine the next run ID based on existing data in the log file."""
        if not os.path.exists(self.log_file_path):
            return 1
        try:
            max_run_id = 0
            with open(self.log_file_path, 'r', newline='') as csvfile:
                reader = csv.DictReader(csvfile)
                for row in reader:
                    if 'run_id' in row and row['run_id']:
                        try:
                            run_id = int(row['run_id'])
                            max_run_id = max(max_run_id, run_id)
                        except (ValueError, TypeError):
                            pass
            return max_run_id + 1
        except Exception as e:
            print(f"Error determining run ID: {e}")
            return 1
    
    def process_emlid_gps_data(self, emlid_data):
        """Process real Emlid GPS data (not used in simulation)."""
        if not emlid_data or 'latitude' not in emlid_data or 'longitude' not in emlid_data:
            print("⚠️ Invalid or missing Emlid GPS data")
            return None, None
        try:
            easting, northing = self.converter.latlon_to_utm_coord(
                emlid_data['latitude'], emlid_data['longitude']
            )
            if easting is None or northing is None:
                print("⚠️ Failed to convert Emlid lat/lon to UTM")
                return None, None
                
            # Calculate local simulation coordinates by applying the offset
            utm_offset_x = 0
            utm_offset_y = 0
            if hasattr(self.rover, 'navigator') and hasattr(self.rover.navigator, 'utm_offset_x'):
                utm_offset_x = self.rover.navigator.utm_offset_x
                utm_offset_y = self.rover.navigator.utm_offset_y
                
            sim_x = easting - utm_offset_x
            sim_y = northing - utm_offset_y
            self.rover.set_position(sim_x, sim_y)
            
            return easting, northing
        except Exception as e:
            print(f"Error processing Emlid GPS data: {e}")
            return None, None
            
    def get_gps_data(self):
        """Get current GPS data from rover, ensuring UTM offsets are applied and valid lat/lon are computed."""
        # Get rover heading and calculate bearing
        heading = self.rover.heading if hasattr(self.rover, 'heading') else 0.0
        bearing = (90 - heading) % 360
        compass_heading = self.rover.get_compass_direction(heading) if hasattr(self.rover, 'get_compass_direction') else 'Unknown'
        
        # Get rover position
        x_local = self.rover.x if hasattr(self.rover, 'x') else 0.0
        y_local = self.rover.y if hasattr(self.rover, 'y') else 0.0
        
        # Set default UTM offsets (used if navigator is not available)
        utm_offset_x = 380000.0
        utm_offset_y = 2044880.0
        deviation = 0.0
        
        # Get UTM offsets from navigator if available
        if (hasattr(self.rover, 'navigator') and 
            hasattr(self.rover.navigator, 'utm_offset_x') and
            self.rover.navigator.utm_offset_x is not None and
            self.rover.navigator.utm_offset_y is not None):
            utm_offset_x = self.rover.navigator.utm_offset_x
            utm_offset_y = self.rover.navigator.utm_offset_y
            if hasattr(self.rover.navigator, 'calculate_deviation'):
                deviation = self.rover.navigator.calculate_deviation(x_local, y_local)
        
        # Calculate actual UTM coordinates
        x_utm = x_local + utm_offset_x
        y_utm = y_local + utm_offset_y
        
        # Convert UTM to lat/lon
        try:
            lat, lon = self.converter.utm_to_latlon_coord(
                x_utm, y_utm,
                zone_number=45, zone_letter='N'
            )
            
            # Validate lat/lon ranges
            if lat is None or lon is None or not (-90 <= lat <= 90) or not (-180 <= lon <= 180):
                raise ValueError(f"Invalid lat/lon values: {lat}, {lon}")
            
            # Check for RTK status from Emlid M2
            fix_quality = 'No Fix'
            if hasattr(self.rover, 'emlid_data') and self.rover.emlid_data:
                # Extract RTK fix status from Emlid data if available
                if 'rtk_status' in self.rover.emlid_data:
                    fix_quality = self.rover.emlid_data['rtk_status']
                elif 'fix_quality' in self.rover.emlid_data:
                    fix_quality = self.rover.emlid_data['fix_quality']
            else:
                # Determine RTK status based on NTRIP connection and other factors
                if hasattr(self.rover, 'ntrip_client') and self.rover.ntrip_client:
                    if hasattr(self.rover.ntrip_client, 'connected') and self.rover.ntrip_client.connected:
                        if hasattr(self.rover, 'failsafe') and self.rover.failsafe:
                            hdop = getattr(self.rover.failsafe, 'gps_hdop', 1.5)
                            if hdop < 0.2:
                                fix_quality = 'RTK Fixed'
                            elif hdop < 0.5:
                                fix_quality = 'RTK Float'
                            elif hdop < 1.0:
                                fix_quality = 'DGPS'
                            else:
                                fix_quality = 'GNSS'
                        else:
                            # Default to RTK Float when NTRIP connected but no detailed info
                            fix_quality = 'RTK Float'
                    else:
                        fix_quality = 'GNSS'  # No NTRIP connection
                else:
                    fix_quality = 'GNSS'  # Fallback for simulation
            
            satellite_count = 0
            # Try to get actual satellite count from emlid data or failsafe
            if hasattr(self.rover, 'emlid_data') and self.rover.emlid_data and 'satellites' in self.rover.emlid_data:
                satellite_count = self.rover.emlid_data['satellites']
            elif hasattr(self.rover, 'failsafe') and hasattr(self.rover.failsafe, 'gps_satellites'):
                satellite_count = self.rover.failsafe.gps_satellites
            else:
                satellite_count = random.randint(8, 12)  # Fallback for simulation
            
            status = 'OK'
        except Exception as e:
            print(f"⚠️ Error converting UTM to Lat/Lon: {e}")
            lat, lon = 0.0, 0.0
            fix_quality = 'No Fix'
            satellite_count = 0
            status = 'ERROR'

        # Create and return the data dictionary
        return {
            'timestamp': datetime.now().isoformat(),
            'run_id': self.rover.run_id,
            'x_utm': x_utm,
            'y_utm': y_utm, 
            'latitude': lat,
            'longitude': lon,
            'heading': heading,
            'bearing': bearing,
            'compass_heading': compass_heading,
            'fix_quality': fix_quality,
            'satellite_count': satellite_count,
            'deviation': deviation,
            'data_age': 0,
            'status': status
        }
    def log_data_once(self):
        """Log a single GPS data entry to the CSV file."""
        try:
            # Get GPS data
            data = self.get_gps_data()
            
            # Ensure all required fields have valid values
            for field in self.fieldnames:
                if field not in data or data[field] is None:
                    if field in ['x_utm', 'y_utm', 'latitude', 'longitude', 'heading', 
                            'bearing', 'deviation', 'data_age', 'satellite_count']:
                        data[field] = 0.0  # Default for numeric fields
                    elif field == 'status':
                        data[field] = 'ERROR'
                    elif field == 'fix_quality':
                        data[field] = 'No Fix'
                    elif field == 'run_id':
                        data[field] = self.rover.run_id
                    else:
                        data[field] = ""
            
            # Write to CSV file
            with open(self.log_file_path, 'a', newline='') as csvfile:
                writer = csv.DictWriter(csvfile, fieldnames=self.fieldnames)
                writer.writerow(data)
                
        except Exception as e:
            print(f"Error in log_data_once: {e}")
            # Try to write an error record if possible
            try:
                error_data = {field: "" for field in self.fieldnames}
                error_data.update({
                    'timestamp': datetime.now().isoformat(),
                    'run_id': getattr(self.rover, 'run_id', 0),
                    'status': f'ERROR: {str(e)[:50]}'
                })
                
                with open(self.log_file_path, 'a', newline='') as csvfile:
                    writer = csv.DictWriter(csvfile, fieldnames=self.fieldnames)
                    writer.writerow(error_data)
            except:
                pass  # If even error logging fails, just continue
        
    
    def logger_loop(self):
        """Main logger loop that runs in a separate thread."""
        while not self.stop_event.is_set():
            try:
                current_time = time.time()
                time_since_last_log = current_time - self.last_log_time
                
                # If it's time to log data
                if time_since_last_log >= self.log_interval:
                    self.log_data_once()
                    self.last_log_time = current_time
                
                # Sleep to maintain proper interval
                sleep_time = max(0.01, self.log_interval - (time.time() - current_time))
                time.sleep(sleep_time)
            except Exception as e:
                print(f"Error in logger loop: {e}")
                time.sleep(0.1)  # Sleep briefly to avoid tight error loops
    
    def start(self):
        """Start the GPS logging thread."""
        if self.running:
            print("GPS logger is already running")
            return
            
        print("📡 Starting GPS data logger (100ms interval)")
        self.stop_event.clear()
        self.running = True
        self.last_log_time = time.time()
        self.logger_thread = Thread(target=self.logger_loop, daemon=True)
        self.logger_thread.start()
    
    def stop(self):
        """Stop the GPS logging thread."""
        if not self.running:
            return
            
        print("📡 Stopping GPS data logger")
        self.stop_event.set()
        if self.logger_thread and self.logger_thread.is_alive():
            self.logger_thread.join(timeout=1.0)
        self.running = False

# Integration functions for the Rover class
def initialize_gps_logger(rover):
    """Initialize and start the GPS logger for the rover."""
    rover.gps_logger = GPSLogger(rover)
    rover.gps_logger.start()
    return rover.gps_logger

def stop_gps_logger(rover):
    """Stop the GPS logger if it's running."""
    if hasattr(rover, 'gps_logger') and rover.gps_logger.running:
        rover.gps_logger.stop()

def update_rover_position_from_emlid(rover, emlid_data):
    """
    Update rover position from Emlid GPS data.
    
    Args:
        rover: The rover instance
        emlid_data: Dictionary containing 'latitude', 'longitude', and RTK status from Emlid receiver
    
    Returns:
        bool: True if position was updated successfully, False otherwise
    """
    if not hasattr(rover, 'gps_logger'):
        print("⚠️ GPS logger not initialized")
        return False
        
    try:
        # Store the Emlid data in the rover for later use by GPS logger
        rover.emlid_data = emlid_data
        
        # Process RTK fix status if available
        if 'solution_status' in emlid_data:
            # Map Emlid's solution_status values to our standardized RTK status values
            status_mapping = {
                'fixed': 'RTK Fixed',
                'float': 'RTK Float',
                'single': 'GNSS',
                'dgps': 'DGPS',
                'none': 'No Fix',
                'autonomous': 'GNSS',
            }
            
            # Store fix quality with our standardized values
            emlid_data['rtk_status'] = status_mapping.get(emlid_data['solution_status'].lower(), 'GNSS')
            
        # Process the emlid GPS data
        easting, northing = rover.gps_logger.process_emlid_gps_data(emlid_data)
        if easting is not None and northing is not None:
            return True
        return False
    except Exception as e:
        print(f"Error updating rover position: {e}")
        return False