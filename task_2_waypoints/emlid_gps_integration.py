import serial
import time
import json
import threading
from coordinate_converter import CoordinateConverter

class EmlidGPSReader:
    """
    Class to handle reading GPS data from an Emlid Reach GNSS receiver.
    The Emlid receiver outputs NMEA or JSON data which this class parses
    and converts to UTM coordinates for the rover navigation system.
    """
    
    def __init__(self, port='/dev/ttyACM0', baud_rate=115200, message_format='json'):
        """
        Initialize the Emlid GPS reader.
        
        Args:
            port (str): Serial port where the Emlid receiver is connected
            baud_rate (int): Baud rate for serial communication
            message_format (str): Format of the GPS messages ('json' or 'nmea')
        """
        self.port = port
        self.baud_rate = baud_rate
        self.message_format = message_format
        self.serial_connection = None
        self.reading_thread = None
        self.stop_thread = threading.Event()
        self.converter = CoordinateConverter()
        self.last_position = None
        self.last_update_time = 0
        self.callbacks = []
        
    def connect(self):
        """
        Connect to the Emlid receiver through serial port.
        
        Returns:
            bool: True if connection successful, False otherwise
        """
        try:
            self.serial_connection = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=1
            )
            print(f"✅ Connected to Emlid receiver on {self.port}")
            return True
        except Exception as e:
            print(f"❌ Failed to connect to Emlid receiver: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from the Emlid receiver."""
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
            print("✅ Disconnected from Emlid receiver")
    
    def start_reading(self):
        """Start a thread to continuously read GPS data."""
        if not self.serial_connection or not self.serial_connection.is_open:
            success = self.connect()
            if not success:
                return False
                
        self.stop_thread.clear()
        self.reading_thread = threading.Thread(target=self._read_loop)
        self.reading_thread.daemon = True
        self.reading_thread.start()
        print("📡 Started GPS data reading thread")
        return True
    
    def stop_reading(self):
        """Stop the GPS reading thread."""
        if self.reading_thread and self.reading_thread.is_alive():
            self.stop_thread.set()
            self.reading_thread.join(timeout=2.0)
            print("📡 Stopped GPS data reading thread")
        
        # Disconnect from serial port
        self.disconnect()
    
    def register_callback(self, callback_func):
        """
        Register a callback function to be called when new GPS data is received.
        
        Args:
            callback_func: Function that takes a position dict as parameter
        """
        if callback_func not in self.callbacks:
            self.callbacks.append(callback_func)
    
    def unregister_callback(self, callback_func):
        """Remove a callback function from the list."""
        if callback_func in self.callbacks:
            self.callbacks.remove(callback_func)
    
    def _read_loop(self):
        """Main loop for reading GPS data from the serial port."""
        while not self.stop_thread.is_set():
            try:
                if self.message_format == 'json':
                    position = self._read_json()
                else:
                    position = self._read_nmea()
                    
                if position:
                    self.last_position = position
                    self.last_update_time = time.time()
                    
                    # Convert lat/lon to UTM
                    if 'latitude' in position and 'longitude' in position:
                        easting, northing = self.converter.latlon_to_utm_coord(
                            position['latitude'], position['longitude']
                        )
                        position['easting'] = easting
                        position['northing'] = northing
                    
                    # Call all registered callbacks with the position data
                    for callback in self.callbacks:
                        try:
                            callback(position)
                        except Exception as e:
                            print(f"Error in GPS callback: {e}")
            
            except Exception as e:
                print(f"Error reading GPS data: {e}")
                time.sleep(1)  # Wait before retrying
    
    def _read_json(self):
        """
        Read and parse JSON formatted GPS data from Emlid.
        
        Returns:
            dict: Parsed GPS position data or None if no valid data
        """
        if not self.serial_connection:
            return None
            
        try:
            line = self.serial_connection.readline().decode('utf-8').strip()
            if not line:
                return None
                
            data = json.loads(line)
            
            # Extract relevant GPS data
            if 'position' in data:
                position = {
                    'latitude': data['position'].get('lat', 0),
                    'longitude': data['position'].get('lon', 0),
                    'altitude': data['position'].get('height', 0),
                    'fix_quality': data.get('solution_status', 'Unknown'),
                    'satellites': data.get('satellites_used', 0),
                    'hdop': data.get('pdop', 0)  # Using PDOP if HDOP not available
                }
                return position
        except json.JSONDecodeError:
            pass
        except Exception as e:
            print(f"Error parsing JSON GPS data: {e}")
            
        return None
    
    def _read_nmea(self):
        """
        Read and parse NMEA formatted GPS data from Emlid.
        
        Returns:
            dict: Parsed GPS position data or None if no valid data
        """
        if not self.serial_connection:
            return None
            
        try:
            line = self.serial_connection.readline().decode('utf-8').strip()
            if not line or not line.startswith('$'):
                return None
                
            # Parse NMEA GGA sentence
            if line.startswith('$GPGGA') or line.startswith('$GNGGA'):
                parts = line.split(',')
                if len(parts) >= 15:
                    # Check if we have a valid fix
                    fix_quality = int(parts[6]) if parts[6] else 0
                    if fix_quality > 0:
                        # Extract latitude
                        lat = float(parts[2][:2]) + float(parts[2][2:]) / 60
                        if parts[3] == 'S':
                            lat = -lat
                            
                        # Extract longitude
                        lon = float(parts[4][:3]) + float(parts[4][3:]) / 60
                        if parts[5] == 'W':
                            lon = -lon
                            
                        position = {
                            'latitude': lat,
                            'longitude': lon,
                            'altitude': float(parts[9]) if parts[9] else 0,
                            'fix_quality': ['No Fix', '2D Fix', '3D Fix'][min(fix_quality, 2)],
                            'satellites': int(parts[7]) if parts[7] else 0,
                            'hdop': float(parts[8]) if parts[8] else 0
                        }
                        return position
        except Exception as e:
            print(f"Error parsing NMEA GPS data: {e}")
            
        return None
    
    def get_last_position(self):
        """
        Get the last received GPS position.
        
        Returns:
            dict: Last GPS position or None if no position has been received
        """
        return self.last_position


def update_rover_from_emlid(rover, emlid_reader):
    """
    Callback function to update rover position from Emlid GPS data.
    
    Args:
        rover: The rover instance
        emlid_reader: EmlidGPSReader instance
    """
    def on_gps_data(position):
        if position and 'easting' in position and 'northing' in position:
            # Update rover position with UTM coordinates
            rover.set_position(position['easting'], position['northing'])
            
            # Update failsafe module if available
            if hasattr(rover, 'failsafe'):
                rover.failsafe.update_gps_status(
                    has_fix=position['fix_quality'] != 'No Fix',
                    satellites=position['satellites'],
                    hdop=position['hdop']
                )
    
    # Register the callback with the Emlid reader
    emlid_reader.register_callback(on_gps_data)


def setup_emlid_integration(rover):
    """
    Set up Emlid GPS integration with the rover.
    
    Args:
        rover: The rover instance
        
    Returns:
        EmlidGPSReader: Configured GPS reader instance
    """
    # Initialize the GPS reader
    emlid_reader = EmlidGPSReader()
    
    # Configure rover to use Emlid GPS data
    update_rover_from_emlid(rover, emlid_reader)
    
    # Start reading GPS data
    success = emlid_reader.start_reading()
    if not success:
        print("⚠️ Failed to start Emlid GPS reader")
    
    return emlid_reader