import random
import serial
import time
import json
import threading
from coordinate_converter import CoordinateConverter
import os
from serial import SerialException
simulate_gps = False  # Set to True if you want to force simulation even if Emlid is connected

class EmlidGPSReader:
    """
    Class to handle reading GPS data from an Emlid Reach GNSS receiver.
    The Emlid receiver outputs NMEA or JSON data which this class parses
    and converts to UTM coordinates for the rover navigation system.
    """
    
    def __init__(self, port=None, baud_rate=115200, message_format='nmea'):
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
        self.port = port or self._autodetect_emlid_port()

    def _autodetect_emlid_port(self):
        # Common ports for Emlid M2 on Windows/Linux
        possible_ports = ['COM3', 'COM8', '/dev/ttyACM0', '/dev/ttyUSB0']
        for port in possible_ports:
            if os.path.exists(port):
                return port
        raise Exception("Emlid M2 not found on common ports!")

    def connect(self, retries=3, retry_delay=2):
        """
        Connect to the Emlid receiver with retry logic.
        
        Args:
            retries (int): Number of connection attempts (default: 3)
            retry_delay (float): Delay between attempts in seconds (default: 2)
        
        Returns:
            bool: True if connection successful, False otherwise
        """
        for attempt in range(1, retries + 1):
            try:
                self.serial_connection = serial.Serial(
                    port=self.port,
                    baudrate=self.baud_rate,
                    timeout=1
                )
                print(f"✅ Connected to Emlid receiver on {self.port} (Attempt {attempt}/{retries})")
                return True
            except (serial.SerialException, OSError) as e:
                retry_delay *= 1.5 
                if attempt < retries:
                    print(f"⚠️ Connection failed (Attempt {attempt}/{retries}): {e}")
                    print(f"Retrying in {retry_delay}s...")
                    time.sleep(retry_delay)
                else:
                    print(f"❌ Failed to connect after {retries} attempts")
                    return False
        return True
    
    def send_rtcm_data(self, rtcm_bytes: bytes):
        """
        Send raw RTCM correction bytes to the Emlid unit.
        """
        if self.serial_connection and self.serial_connection.is_open:
            try:
                self.serial_connection.write(rtcm_bytes)
            except Exception as e:
                print(f"⚠️ Failed to send RTCM data: {e}")
        
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
    
    def _simulate_gps_data(self):
        """Return simulated GPS data for testing."""
        return {
            'latitude': 28.6139,
            'longitude': 77.2090,
            'altitude': 216.0,
            'satellites': random.randint(8, 12),
            'hdop': random.uniform(0.8, 1.5),
            'fix_quality': random.choice(['GPS', 'DGPS', 'RTK Fixed', 'RTK Float'])
        }

    def _read_loop(self):
        """Main loop for reading GPS data from the serial port."""
        while not self.stop_thread.is_set():
            try:
                if simulate_gps:
                    position = self._simulate_gps_data()
                else:
                    if not self.serial_connection or not self.serial_connection.is_open:
                        print("⚠️ Serial connection lost, attempting to reconnect...")
                        if not self.connect():
                            time.sleep(1)
                            continue
                    
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
            
            except serial.SerialException as e:
                print(f"Serial error: {e}, retrying in 1s...")
                self.disconnect()
                time.sleep(1)
            except Exception as e:
                print(f"Error reading GPS data: {e}")
                time.sleep(1)
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
        """Parse NMEA sentences from Emlid M2 and return GPS data dictionary."""
        try:
            line = self.serial_connection.readline().decode('utf-8', errors='ignore').strip()
            
            # Parse GPGGA (Global Positioning System Fix Data)
            if line.startswith(('$GPGGA', '$GNGGA')):
                parts = line.split(',')
                if len(parts) >= 15:
                    return {
                        'latitude': self._nmea_to_decimal(parts[2], parts[3]),
                        'longitude': self._nmea_to_decimal(parts[4], parts[5]),
                        'altitude': float(parts[9]) if parts[9] else 0.0,
                        'satellites': int(parts[7]) if parts[7] else 0,
                        'hdop': float(parts[8]) if parts[8] else 99.9,
                        'fix_quality': {
                            '0': 'Invalid',
                            '1': 'GPS',
                            '2': 'DGPS',
                            '4': 'RTK Fixed',
                            '5': 'RTK Float'
                        }.get(parts[6], 'Unknown')
                    }
            
            # Parse GPRMC (Recommended Minimum Specific GNSS Data)
            elif line.startswith('$GPRMC'):
                parts = line.split(',')
                if len(parts) >= 10 and parts[2] == 'A':  # 'A' = Active/Valid fix
                    lat = self._nmea_to_decimal(parts[3], parts[4])
                    lon = self._nmea_to_decimal(parts[5], parts[6])
                    speed = float(parts[7]) * 0.514444 if parts[7] else 0.0
                    course = float(parts[8]) if parts[8] else 0.0
                    return {
                        'latitude': lat,
                        'longitude': lon,
                        'speed': speed,
                        'course': course,
                        'fix_quality': 'GPS',  # GPRMC does not provide RTK status
                        'timestamp': parts[1][:6]
                    }
                        
        except Exception as e:
            print(f"NMEA parsing error: {e}")
        return None

    def _nmea_to_decimal(self, nmea_coord, direction):
        """Convert NMEA coordinate (DDMM.MMMM) to decimal degrees."""
        if not nmea_coord or not direction:
            return 0.0
        
        try:
            degrees = float(nmea_coord[:2]) if len(nmea_coord) > 2 else 0.0
            minutes = float(nmea_coord[2:])
            decimal = degrees + (minutes / 60.0)
            
            if direction in ('S', 'W'):
                decimal *= -1
            return decimal
        except ValueError:
            return 0.0
    
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
                    has_fix = position['fix_quality'] in ['GPS', 'DGPS', 'RTK Fixed', 'RTK Float'],
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
    
    # Try connecting with 5 retries and 3-second delays
    success = emlid_reader.connect(retries=5, retry_delay=3)
    
    if success:
        print("Connected! Starting data collection...")
    else:
        print("Failed to connect. Check hardware and try again.")
    
    # Start reading GPS data
    success = emlid_reader.start_reading()
    if not success:
        print("⚠️ Failed to start Emlid GPS reader")
    
    return emlid_reader