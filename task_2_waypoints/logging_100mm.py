import time
from datetime import datetime
import csv
import os
import random
from threading import Thread, Event

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
        
        # Ensure log directory exists
        os.makedirs(os.path.dirname(self.log_file_path), exist_ok=True)
        
        # Write header if file doesn't exist
        if not os.path.exists(self.log_file_path) or os.path.getsize(self.log_file_path) == 0:
            with open(self.log_file_path, 'w', newline='') as csvfile:
                fieldnames = ['timestamp', 'run_id', 'x', 'y', 'heading', 'bearing', 
                             'compass_heading', 'fix_quality', 'satellite_count', 
                             'deviation', 'data_age', 'status']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
        
        # If rover doesn't have a run_id, initialize it
        if not hasattr(self.rover, 'run_id'):
            self.rover.run_id = self.get_next_run_id()
    
    def get_next_run_id(self):
        """Determine the next run ID based on existing data in the log file."""
        if not os.path.exists(self.log_file_path):
            return 1
            
        try:
            # Read the existing file to find the highest run_id
            max_run_id = 0
            with open(self.log_file_path, 'r', newline='') as csvfile:
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
   # Modify the get_gps_data method in your logging_100mm.py file
    def get_gps_data(self):
        """Get current GPS data from rover. In a real system, this would 
        query the actual GPS hardware."""
        # Calculate standard compass bearing
        bearing = (90 - self.rover.heading) % 360  
        
        # Get compass direction
        compass_heading = self.rover.get_compass_direction(self.rover.heading)
        
        # Safely calculate deviation if the method exists
        deviation = 0
        if hasattr(self.rover, 'navigator'):
            navigator = self.rover.navigator
            if hasattr(navigator, 'calculate_deviation'):
                deviation = navigator.calculate_deviation(self.rover.x, self.rover.y)
            elif hasattr(navigator, 'calculate_path_deviation'):
                # Try an alternative method name if it exists
                deviation = navigator.calculate_path_deviation(self.rover.x, self.rover.y)
        
        return {
            'timestamp': datetime.now().isoformat(),
            'run_id': self.rover.run_id,
            'x': self.rover.x,
            'y': self.rover.y,
            'heading': self.rover.heading,
            'bearing': bearing,
            'compass_heading': compass_heading,
            'fix_quality': '3D Fix',  # Simulated fix quality
            'satellite_count': random.randint(8, 12),  # Simulated satellite count
            'deviation': deviation,
            'data_age': 0,  # Fresh data
            'status': 'OK'
        }
        
    def log_data_once(self):
        """Attempt to log GPS data once with retry mechanism."""
        attempts = 0
        data = None
        
        while attempts < self.max_attempts:
            try:
                data = self.get_gps_data()
                if data:
                    break  # Successfully got data
            except Exception as e:
                print(f"GPS data retrieval error (attempt {attempts+1}/{self.max_attempts}): {e}")
            
            attempts += 1
            time.sleep(0.1)  # Wait 100ms between attempts
        
        # If we still don't have data after all attempts
        if data is None:
            data = {
                'timestamp': datetime.now().isoformat(),
                'run_id': getattr(self.rover, 'run_id', 0),
                'x': getattr(self.rover, 'x', 0),
                'y': getattr(self.rover, 'y', 0),
                'heading': getattr(self.rover, 'heading', 0),
                'bearing': 0,
                'compass_heading': 'Unknown',
                'fix_quality': 'No Fix',
                'satellite_count': 0,
                'deviation': 0,
                'data_age': 300,  # 300ms old (stale data)
                'status': 'ERROR: GPS data unavailable after 3 attempts'
            }
            print("⚠️ WARNING: Failed to receive fresh GPS data after 3 attempts (300ms)")
        
        # Write data to log file
        try:
            with open(self.log_file_path, 'a', newline='') as csvfile:
                fieldnames = ['timestamp', 'run_id', 'x', 'y', 'heading', 'bearing', 
                             'compass_heading', 'fix_quality', 'satellite_count', 
                             'deviation', 'data_age', 'status']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writerow(data)
                self.last_log_time = time.time()
                return True
        except Exception as e:
            print(f"Error writing to GPS log: {e}")
            return False
    
    def logger_loop(self):
        """Main logger loop that runs in a separate thread."""
        while not self.stop_event.is_set():
            current_time = time.time()
            time_since_last_log = current_time - self.last_log_time
            
            # If it's time to log data
            if time_since_last_log >= self.log_interval:
                self.log_data_once()
            
            # Sleep a bit to prevent thrashing
            # Calculate sleep time to maintain proper interval
            sleep_time = max(0.01, self.log_interval - (time.time() - current_time))
            time.sleep(sleep_time)
    
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


# To integrate with the existing Rover class, add these methods:

def initialize_gps_logger(rover):
    """Initialize and start the GPS logger for the rover."""
    rover.gps_logger = GPSLogger(rover)
    rover.gps_logger.start()
    return rover.gps_logger

def stop_gps_logger(rover):
    """Stop the GPS logger if it's running."""
    if hasattr(rover, 'gps_logger') and rover.gps_logger.running:
        rover.gps_logger.stop()



