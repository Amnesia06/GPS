import random
import serial
import time
import json
import threading
from coordinate_converter import CoordinateConverter
import os
from serial import SerialException
import logging
import datetime
simulate_gps = False  # Set to True if you want to force simulation even if Emlid is connected

class EmlidGPSReader:
    """
    Class to handle reading GPS data from an Emlid Reach GNSS receiver.
    The Emlid receiver outputs NMEA or JSON data which this class parses
    and converts to UTM coordinates for the rover navigation system.
    """
    
    def __init__(self, port='COM12', baud_rate=115200, message_format='nmea'):
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
        self.port = port or 'COM12'

    
    def validate_position(self, position):
        if not position:
            return False
        lat = position.get('latitude', 0)
        lon = position.get('longitude', 0)
        hdop = position.get('hdop', 99.9)
        satellites = position.get('satellites', 0)

        return (-90 <= lat <= 90 and 
                -180 <= lon <= 180 and 
                hdop < 2.5 and satellites >= 6
        )

    def __del__(self):
        """Destructor to ensure cleanup"""
        self.stop_reading()
        self.disconnect()

    def _autodetect_emlid_port(self):
        # Common ports for Emlid M2 on Windows/Linux
        possible_ports = ['COM12', 'COM11', '/dev/ttyACM0', '/dev/ttyUSB0']
        for port in possible_ports:
            if os.path.exists(port):
                print("Emlid is connected")
                return port
        raise Exception("Emlid M2 not found on common ports!")

    def connect(self, retries=10, retry_delay=2):
        for attempt in range(retries):
            try:
                if self.serial_connection and self.serial_connection.is_open:
                    self.serial_connection.close()
                self.serial_connection = serial.Serial(self.port, self.baud_rate, timeout=1)
                logging.info(f"Connected to {self.port}")
                return True
            except serial.SerialException as e:
                logging.warning(f"Connection attempt {attempt + 1}/{retries} failed: {e}")
                time.sleep(retry_delay)
                retry_delay = min(retry_delay * 2, 30)  # Cap at 30 seconds
        logging.error(f"Failed to connect after {retries} attempts")
        self.simulate_gps = False
        return False
    
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
        """Read GPS data at high frequency (10Hz)"""
        while not self.stop_thread.is_set():
            try:
                if self.simulate_gps:
                    position = self._simulate_gps_data()
                    time.sleep(0.1)
                else:
                    if not self.serial_connection or not self.serial_connection.is_open:
                        if not self.connect():
                            time.sleep(0.1)
                            continue
                                    
                    data = self._read_nmea() if self.message_format == 'nmea' else self._read_json()

                    if data:
                        position = data
                        print(f"\n=== GPS Update [{datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]}] ===")
                        print(f"UTM: [{data['easting']:.3f}, {data['northing']:.3f}]")
                        print(f"Fix: {position.get('fix_quality', 'Unknown')}")
                        print(f"Sats: {position.get('satellites', 0)}")
                        print(f"HDOP: {position.get('hdop', 0.0):.1f}")

                        # Update timestamps and notify callbacks
                        self.last_position = position.copy()
                        self.last_update_time = time.time()

                        for callback in self.callbacks:
                            try:
                                callback(position)
                            except Exception as cb_error:
                                logging.error(f"Callback error: {cb_error}")
                    
                    time.sleep(0.1)  # 10Hz update rate, moved inside 'else' block

            except Exception as e:
                logging.error(f"Read loop error: {e}")
                time.sleep(1)

    def check_health(self):
        """Check system health"""
        health = {
            'connected': bool(self.serial_connection and self.serial_connection.is_open),
            'last_update': time.time() - self.last_update_time,
            'fix_quality': self.last_position.get('fix_quality') if self.last_position else None,
            'satellites': self.last_position.get('satellites') if self.last_position else 0,
            'thread_alive': bool(self.reading_thread and self.reading_thread.is_alive())
        }
        return health
    
    def _validate_position_data(self, position):
        """Validate position data"""
        required_fields = ['latitude', 'longitude', 'altitude']
        if not all(field in position for field in required_fields):
            logging.warning("Missing required GPS fields")
            return False
            
        # Check for reasonable values
        if not (-90 <= position['latitude'] <= 90):
            logging.warning(f"Invalid latitude: {position['latitude']}")
            return False
        if not (-180 <= position['longitude'] <= 180):
            logging.warning(f"Invalid longitude: {position['longitude']}")
            return False
        if not (-1000 <= position['altitude'] <= 10000):
            logging.warning(f"Invalid altitude: {position['altitude']}")
            return False
            
        return True
    
    def _process_gga(self, parts):
        """Process NMEA GGA message"""
        try:
            position = {
                'latitude': self._nmea_to_decimal(parts[2], parts[3]),
                'longitude': self._nmea_to_decimal(parts[4], parts[5]),
                'satellites': int(parts[7]),
                'hdop': float(parts[8])
            }
            
            # Add validation before updating
            if self.validate_position(position):
                self.last_position = position
                return True
            return False
            
        except (ValueError, IndexError) as e:
            logging.error(f"GGA parsing error: {e}")
            return False

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
            if not line:
                print("⚠️ No NMEA data received")
                return None
            
            print(f"📜 NMEA sentence: {line}")
            
            # Parse GPGGA (Global Positioning System Fix Data)
            if line.startswith(('$GPGGA', '$GNGGA')):
                parts = line.split(',')
                if len(parts) >= 15 and parts[6] != '0' and parts[2] and parts[4]:  # Check fix and valid lat/lon
                    position = {
                        'latitude': self._nmea_to_decimal(parts[2], parts[3], is_longitude=False),
                        'longitude': self._nmea_to_decimal(parts[4], parts[5], is_longitude=True),
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
                    easting, northing, zone_number, zone_letter = self.converter.latlon_to_utm(
                        position['latitude'], 
                        position['longitude']
                    )
                    position.update({
                        'easting': easting,
                        'northing': northing,
                        'zone_number': zone_number,
                        'zone_letter': zone_letter
                    })



                    if position['latitude'] == 0.0 and position['longitude'] == 0.0:
                        print("⚠️ Invalid GPS fix: Zero coordinates")
                        return None
                    print(f"✅ Parsed GPGGA: {position}")
                    if self.validate_position(position):
                        self.last_position = position
                        return position
                    return None

            
            # Parse GPRMC (Recommended Minimum Specific GNSS Data)
            elif line.startswith('$GPRMC'):
                parts = line.split(',')
                if len(parts) >= 10 and parts[2] == 'A' and parts[3] and parts[5]:  # Valid fix and lat/lon
                    position = {
                        'latitude': self._nmea_to_decimal(parts[3], parts[4], is_longitude=False),
                        'longitude': self._nmea_to_decimal(parts[5], parts[6], is_longitude=True),
                        'speed': float(parts[7]) * 0.514444 if parts[7] else 0.0,
                        'course': float(parts[8]) if parts[8] else 0.0,
                        'fix_quality': 'GPS',
                        'timestamp': parts[1][:6] if parts[1] else ''
                    }
                    if position['latitude'] == 0.0 and position['longitude'] == 0.0:
                        print("⚠️ Invalid GPS fix: Zero coordinates")
                        return None
                    print(f"✅ Parsed GPRMC: {position}")
                    return position
                
            if position and self.validate_position(position):
                self.last_position = position

            # Ignore other sentences quietly
            return None

        except Exception as nmea_error:
            print(f"❌ NMEA parsing error: {nmea_error}")
            return None
    def _nmea_to_decimal(self, nmea_coord, direction, is_longitude=False):
        """Convert NMEA coordinate to decimal degrees."""
        try:
            if not nmea_coord:
                return None
                
            # Split degrees and minutes
            if is_longitude:
                degrees = float(nmea_coord[:3])
                minutes = float(nmea_coord[3:])
            else:
                degrees = float(nmea_coord[:2])
                minutes = float(nmea_coord[2:])
                
            # Calculate decimal degrees
            decimal = degrees + minutes/60.0
            
            # Apply direction
            if direction in ['S', 'W']:
                decimal = -decimal
                
            return decimal
            
        except Exception as e:
            logging.error(f"Error converting NMEA coordinate: {e}")
            return None
        except Exception as nmea_error:
            logging.error(f"NMEA parsing error: {nmea_error}")
            return None

    def get_last_position(self):
        """
        Get the last received GPS position.
        
        Returns:
            dict: Last GPS position or None if no position has been received
        """
        return self.last_position


def update_rover_from_emlid(rover, emlid_reader):
    """Callback function to update rover position from Emlid GPS data."""
    def on_gps_data(position):
        if not position or 'easting' not in position or 'northing' not in position:
            print("⚠️ Invalid GPS position data")
            return
        
        try:
            # Update rover position with UTM coordinates
            rover.set_position(position['easting'], position['northing'])
            print(f"🚜 Rover position updated: UTM X={position['easting']:.2f}, Y={position['northing']:.2f}")
            print(f"🌍 Corresponding Lat/Lon: {position['latitude']:.6f}, {position['longitude']:.6f}")
            
            # Update failsafe module if available
            if hasattr(rover, 'failsafe'):
                rover.failsafe.update_gps_status(
                    has_fix=position['fix_quality'] in ['GPS', 'DGPS', 'RTK Fixed', 'RTK Float'],
                    satellites=position['satellites'],
                    hdop=position['hdop']
                )
                print(f"🔒 Failsafe updated: Fix={position['fix_quality']}, Satellites={position['satellites']}, HDOP={position['hdop']:.2f}")
        
        except Exception as update_error:
            print(f"❌ Error updating rover position: {update_error}")

    # Register the callback with the Emlid reader
    try:
        emlid_reader.register_callback(on_gps_data)
        print("✅ Callback registered for Emlid GPS data")
    except Exception as callback_error:
        print(f"❌ Error registering callback: {callback_error}")

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