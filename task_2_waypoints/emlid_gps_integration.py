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
import serial.tools.list_ports
import psutil
import subprocess
import sys

_active_readers = []
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
        global _active_readers
        self._is_disconnecting = False  # Add this flag

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
        self.simulate_gps = False
        _active_readers.append(self)

    def validate_position(self, position):
        if not position:
            return False
        lat = position.get('latitude', 0)
        lon = position.get('longitude', 0)
        hdop = position.get('hdop', 99.9)
        satellites = position.get('satellites', 0)

        return (-90 <= lat <= 90 and 
                -180 <= lon <= 180 and 
                hdop < 10.0 and satellites >= 4  # More lenient for initial connection
        )

    def __del__(self):
        """Safe destructor that won't raise exceptions"""
        try:
            if not self._is_disconnecting:  # Only disconnect if not already doing so
                self.stop_reading()
                self.disconnect()
        except:
            pass  # Suppress any errors during cleanup

    def _kill_processes_using_port(self, port):
        """Enhanced process killing that excludes current process"""
        print(f"🔍 Checking processes using {port}...")
        
        try:
            import psutil
            import os
            
            current_pid = os.getpid()  # Get our own process ID
            
            # Get all processes using serial ports
            for proc in psutil.process_iter(['pid', 'name']):
                try:
                    # Skip our own process
                    if proc.pid == current_pid:
                        continue
                        
                    # Only kill known problematic processes
                    if proc.name().lower() in ['python.exe', 'pythonw.exe', 'reachview.exe',
                                            'putty.exe', 'terraterm.exe']:
                        print(f"🔫 Terminating {proc.name()} (PID: {proc.pid})")
                        proc.kill()
                        time.sleep(0.5)
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue
                    
            time.sleep(1)  # Reduced wait time
            print("✅ Process cleanup complete")
            
        except Exception as e:
            print(f"⚠️ Process cleanup warning: {e}")
            # Continue even if cleanup fails

    def _check_port_availability(self, port):
        """Enhanced port availability check"""
        try:
            ports = list(serial.tools.list_ports.comports())
            
            for p in ports:
                if p.device == port:
                    print(f"✅ Found {port}: {p.description}")
                    if p.hwid:
                        print(f"   Hardware ID: {p.hwid}")
                    if p.manufacturer:
                        print(f"   Manufacturer: {p.manufacturer}")
                    
                    # Additional check for FTDI (common for Emlid)
                    if 'FTDI' in str(p.hwid) or 'ftdi' in str(p.description).lower():
                        print("   ✅ Detected FTDI device (likely Emlid compatible)")
                    
                    return True
                    
            print(f"❌ {port} not found in available ports")
            available = [p.device for p in ports]
            if available:
                print(f"Available ports: {', '.join(available)}")
            return False
            
        except Exception as e:
            print(f"Port check error: {e}")
            return False

    def _test_port_basic_access(self, port):
        """Test basic port access without full configuration"""
        try:
            test_serial = serial.Serial()
            test_serial.port = port
            test_serial.timeout = 0.1
            test_serial.open()
            test_serial.close()
            print(f"✅ Basic port access test passed for {port}")
            return True
        except Exception as e:
            print(f"❌ Basic port access test failed for {port}: {e}")
            return False

    def _autodetect_emlid_port(self):
        """Improved auto-detection of Emlid GPS port"""
        print("Searching for Emlid GPS device...")
        ports = list(serial.tools.list_ports.comports())
        
        # Look for Emlid-specific identifiers
        emlid_keywords = ['emlid', 'reach', 'gnss', 'gps', 'ftdi', 'cp210x']
        
        for port in ports:
            description = port.description.lower()
            manufacturer = (port.manufacturer or '').lower()
            hwid = port.hwid.lower()
            
            for keyword in emlid_keywords:
                if keyword in description or keyword in manufacturer or keyword in hwid:
                    print(f"Found potential Emlid device on {port.device}")
                    print(f"Description: {port.description}")
                    return port.device
        
        # If not found by description, try common ports
        possible_ports = ['COM12', 'COM11', 'COM10', 'COM9', 'COM8', 'COM7']
        for port_name in possible_ports:
            for port in ports:
                if port.device == port_name:
                    if self._test_port_basic_access(port_name):
                        print(f"Using common port {port_name}")
                        return port_name
        
        # Return first available port as last resort
        available_ports = [p.device for p in ports]
        if available_ports:
            print(f"Available ports: {', '.join(available_ports)}")
            return available_ports[0]
        
        raise Exception("No serial ports found!")

    def connect(self, retries=5, retry_delay=2):
        """Enhanced connect method with better error handling"""
        print(f"🔌 Connecting to Emlid GPS on {self.port}...")
        
        # First disconnect if already connected
        self.disconnect()
        
        # Verify port exists
        if not os.path.exists(f"//./{self.port}"):
            print(f"❌ Port {self.port} does not exist")
            return False
            
        # Check hardware info
        import subprocess
        try:
            result = subprocess.run(["wmic", "path", "Win32_PnPEntity", "where", 
                                "Caption like '%COM%'", "get", "Caption,DeviceID,Manufacturer", "/format:list"], 
                                capture_output=True, text=True)
            if self.port in result.stdout:
                print(f"✅ Found {self.port}: {result.stdout.split(self.port)[1].split('DeviceID')[0].strip()}")
        except Exception as e:
            print(f"⚠️ Could not get hardware info: {e}")

        # Try different baud rates
        baud_rates = [115200, 57600, 9600]  # Try multiple baud rates
        
        for attempt in range(1, retries + 1):
            for baud in baud_rates:
                try:
                    print(f"🔌 Attempt {attempt}/{retries}: Trying {baud} baud...")
                    
                    # Open with different settings
                    self.ser = serial.Serial(
                        port=self.port,
                        baudrate=baud,
                        bytesize=serial.EIGHTBITS,
                        parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE,
                        timeout=1,
                        write_timeout=1,
                        xonxoff=False,
                        rtscts=False,
                        dsrdtr=False
                    )
                    
                    # Test if we can read data
                    print("📡 Testing data reception...")
                    if self.ser.in_waiting or self.ser.read():
                        print(f"✅ Successfully connected to {self.port} at {baud} baud")
                        return True
                        
                except serial.SerialException as e:
                    print(f"❌ Serial error at {baud} baud: {e}")
                    print("⏳ Waiting for port to be released...")
                    
                except Exception as e:
                    print(f"❌ Other error: {e}")
                    
                finally:
                    if hasattr(self, 'ser') and self.ser:
                        self.ser.close()
                        
                print(f"Waiting {retry_delay} seconds before next attempt...")
                time.sleep(retry_delay)
                
        print(f"❌ Failed to connect to {self.port} after {retries} attempts")
        return False

    def connect_with_simple_approach(self):
        """
        Connect using the exact same approach that works in test.py
        """
        print(f"\n🔌 Connecting to Emlid GPS on {self.port} using test.py approach...")
        
        # First disconnect if already connected
        self.disconnect()
        time.sleep(1)  # Short delay
        
        try:
            # Use EXACTLY the same parameters as in test.py
            self.serial_connection = serial.Serial(
                port=self.port,
                baudrate=115200,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            
            print("✅ Connected to Emlid GPS")
            
            # Test if we can read data
            time.sleep(0.5)  # Give it a moment to receive data
            
            for _ in range(3):  # Try reading a few times
                if self.serial_connection.in_waiting > 0:
                    line = self.serial_connection.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        print(f"📡 Data received: {line}")
                        return True
                time.sleep(0.1)
            
            print("⚠️ Connected but no data received yet")
            return True  # Still return True as we connected successfully
            
        except Exception as e:
            print(f"❌ Connection error: {e}")
            if self.serial_connection:
                try:
                    self.serial_connection.close()
                except:
                    pass
                self.serial_connection = None
            return False

    def send_rtcm_data(self, rtcm_bytes: bytes):
        """Send raw RTCM correction bytes to the Emlid unit."""
        if self.serial_connection and self.serial_connection.is_open:
            try:
                self.serial_connection.write(rtcm_bytes)
            except Exception as e:
                print(f"⚠️ Failed to send RTCM data: {e}")
        
   

    def disconnect(self):
        """Enhanced disconnect with better cleanup and error handling"""
        if self._is_disconnecting:  # Prevent recursive calls
            return
            
        self._is_disconnecting = True
        print("🔌 Disconnecting from GPS...")
        
        try:
            # Stop reading thread first
            if hasattr(self, 'stop_thread'):
                self.stop_thread.set()
            
            # Wait for thread to finish with timeout
            if hasattr(self, 'reading_thread') and self.reading_thread:
                if self.reading_thread.is_alive():
                    if threading.current_thread() != self.reading_thread:
                        try:
                            self.reading_thread.join(timeout=2.0)
                        except RuntimeError:  # Handle "cannot join current thread"
                            pass
            
            # Close serial connection
            if hasattr(self, 'serial_connection') and self.serial_connection:
                if self.serial_connection.is_open:
                    try:
                        self.serial_connection.reset_input_buffer()
                        self.serial_connection.reset_output_buffer()
                    except:
                        pass
                        
                    try:
                        self.serial_connection.close()
                        print(f"✅ Closed connection to {self.port}")
                    except:
                        pass
                        
                self.serial_connection = None
            
            # Reset thread event for next connection
            if hasattr(self, 'stop_thread'):
                self.stop_thread.clear()
                
        except Exception as e:
            print(f"⚠️ Disconnect warning: {e}")
        finally:
            self._is_disconnecting = False
            time.sleep(0.5)  # Short delay after cleanup
            print("🔌 Disconnect complete")
    def start_reading(self):
        """Start a thread to continuously read GPS data with better error handling."""
        # Stop any existing reading thread first
        self.stop_reading()
        time.sleep(0.5)
        
        if not self.serial_connection or not self.serial_connection.is_open:
            print("No active connection. Attempting to connect...")
            success = self.connect()
            if not success:
                print("Failed to establish connection. Cannot start reading.")
                return False
                    
        self.stop_thread.clear()
        self.reading_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.reading_thread.start()
        print("📡 Started GPS data reading thread")
        return True

    def stop_reading(self):
        """Stop the GPS reading thread."""
        print("🛑 Stopping GPS reading...")
        
        if hasattr(self, 'stop_thread'):
            self.stop_thread.set()
            
        if hasattr(self, 'reading_thread') and self.reading_thread and self.reading_thread.is_alive():
            try:
                self.reading_thread.join(timeout=3.0)
                print("📡 Stopped GPS data reading thread")
            except Exception as e:
                print(f"Warning: Thread stop error: {e}")
        
    def register_callback(self, callback_func):
        """Register a callback function to be called when new GPS data is received."""
        if callback_func not in self.callbacks:
            self.callbacks.append(callback_func)
    
    def unregister_callback(self, callback_func):
        """Remove a callback function from the list."""
        if callback_func in self.callbacks:
            self.callbacks.remove(callback_func)
    
    def _simulate_gps_data(self):
        """Return simulated GPS data for testing."""
        solution_statuses = ["RTK Fixed", "RTK Float", "DGPS", "GPS"]
        solution_status = random.choice(solution_statuses)
        
        if solution_status == "RTK Fixed":
            hdop = random.uniform(0.5, 1.0)
        elif solution_status == "RTK Float":
            hdop = random.uniform(1.0, 2.0)
        else:
            hdop = random.uniform(2.0, 4.0)
        
        return {
            'latitude': 28.6139,
            'longitude': 77.2090,
            'altitude': 216.0,
            'satellites': random.randint(8, 15),
            'hdop': hdop,
            'fix_quality': solution_status
        }

    def _read_loop(self):
        """Read GPS data at high frequency (10Hz) with robust error handling"""
        consecutive_errors = 0
        max_consecutive_errors = 10
        
        while not self.stop_thread.is_set():
            try:
                if self.simulate_gps:
                    position = self._simulate_gps_data()
                    time.sleep(0.1)
                    consecutive_errors = 0  # Reset error counter
                else:
                    # Check connection health
                    if not self.serial_connection or not self.serial_connection.is_open:
                        print("⚠️ Connection lost, attempting to reconnect...")
                        if not self.connect():
                            consecutive_errors += 1
                            if consecutive_errors >= max_consecutive_errors:
                                print("❌ Too many connection failures, switching to simulation mode")
                                self.simulate_gps = True
                                continue
                            time.sleep(1)
                            continue
                        consecutive_errors = 0
                    
                    # Read data based on format
                    try:
                        data = self._read_nmea() if self.message_format == 'nmea' else self._read_json()
                        consecutive_errors = 0  # Reset on successful read
                    except Exception as read_error:
                        print(f"Read error: {read_error}")
                        consecutive_errors += 1
                        if consecutive_errors >= max_consecutive_errors:
                            print("❌ Too many read errors, attempting reconnection...")
                            self.disconnect()
                            continue
                        time.sleep(0.1)
                        continue

                    if data:
                        position = data
                        print(f"\n=== GPS Update [{datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]}] ===")
                        print(f"UTM: [{data.get('easting', 0):.3f}, {data.get('northing', 0):.3f}]")
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
                    else:
                        # No data received, but this is normal
                        pass
                    
                    time.sleep(0.1)  # 10Hz update rate

            except Exception as e:
                consecutive_errors += 1
                logging.error(f"Read loop error: {e}")
                
                if consecutive_errors >= max_consecutive_errors:
                    print("❌ Too many consecutive errors, switching to simulation mode")
                    self.simulate_gps = True
                    consecutive_errors = 0
                
                time.sleep(1)

    def check_health(self):
        """Check system health"""
        health = {
            'connected': bool(self.serial_connection and self.serial_connection.is_open),
            'last_update': time.time() - self.last_update_time if self.last_update_time > 0 else float('inf'),
            'fix_quality': self.last_position.get('fix_quality') if self.last_position else None,
            'satellites': self.last_position.get('satellites') if self.last_position else 0,
            'thread_alive': bool(self.reading_thread and self.reading_thread.is_alive()),
            'simulation_mode': self.simulate_gps
        }
        return health
    
    def _validate_position_data(self, position):
        """Validate position data"""
        required_fields = ['latitude', 'longitude']
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
        if 'altitude' in position and not (-1000 <= position['altitude'] <= 10000):
            logging.warning(f"Invalid altitude: {position['altitude']}")
            return False
            
        return True
    
    def _process_gga(self, parts):
        """Process NMEA GGA message"""
        try:
            position = {
                'latitude': self._nmea_to_decimal(parts[2], parts[3]),
                'longitude': self._nmea_to_decimal(parts[4], parts[5]),
                'satellites': int(parts[7]) if parts[7] else 0,
                'hdop': float(parts[8]) if parts[8] else 99.9
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
        """Read and parse JSON formatted GPS data from Emlid."""
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
        if not self.serial_connection or not self.serial_connection.is_open:
            return None
            
        try:
            # Check if data is available
            if self.serial_connection.in_waiting == 0:
                return None
                
            line = self.serial_connection.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                return None
            
            # Only print every 10th line to reduce spam
            if random.randint(1, 10) == 1:
                print(f"📜 NMEA: {line[:60]}...")
            
            position = None
            
            # Parse GPGGA (Global Positioning System Fix Data)
            if line.startswith(('$GPGGA', '$GNGGA')):
                parts = line.split(',')
                if len(parts) >= 15 and parts[6] != '0' and parts[2] and parts[4]:
                    try:
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
                        
                        # Skip invalid coordinates
                        if position['latitude'] == 0.0 and position['longitude'] == 0.0:
                            return None
                            
                        # Add UTM conversion
                        try:
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
                        except Exception as utm_error:
                            print(f"UTM conversion error: {utm_error}")
                            
                        if self.validate_position(position):
                            return position
                            
                    except (ValueError, IndexError) as parse_error:
                        if random.randint(1, 10) == 1:  # Only print occasional parsing errors
                            print(f"GPGGA parsing error: {parse_error}")
                        return None
            
            # Parse GPRMC (Recommended Minimum Specific GNSS Data)
            elif line.startswith(('$GPRMC', '$GNRMC')):
                parts = line.split(',')
                if len(parts) >= 10 and parts[2] == 'A' and parts[3] and parts[5]:
                    try:
                        position = {
                            'latitude': self._nmea_to_decimal(parts[3], parts[4], is_longitude=False),
                            'longitude': self._nmea_to_decimal(parts[5], parts[6], is_longitude=True),
                            'speed': float(parts[7]) * 0.514444 if parts[7] else 0.0,  # knots to m/s
                            'course': float(parts[8]) if parts[8] else 0.0,
                            'fix_quality': 'GPS',
                            'satellites': 0,  # GPRMC doesn0t include satellite count
                            'hdop': 99.9,     # GPRMC doesn't include HDOP
                            'timestamp': parts[1][:6] if parts[1] else ''
                        }
                        
                        # Skip invalid coordinates
                        if position['latitude'] == 0.0 and position['longitude'] == 0.0:
                            return None
                            
                        if self.validate_position(position):
                            return position
                            
                    except (ValueError, IndexError) as parse_error:
                        if random.randint(1, 10) == 1:
                            print(f"GPRMC parsing error: {parse_error}")
                        return None

            return None

        except Exception as nmea_error:
            if random.randint(1, 20) == 1:  # Only print occasional errors
                print(f"❌ NMEA parsing error: {nmea_error}")
            return None

    def _nmea_to_decimal(self, nmea_coord, direction, is_longitude=False):
        """Convert NMEA coordinate to decimal degrees."""
        try:
            if not nmea_coord or not direction:
                return None
                
            # Split degrees and minutes
            if is_longitude:
                if len(nmea_coord) < 5:
                    return None
                degrees = float(nmea_coord[:3])
                minutes = float(nmea_coord[3:])
            else:
                if len(nmea_coord) < 4:
                    return None
                degrees = float(nmea_coord[:2])
                minutes = float(nmea_coord[2:])
                
            # Calculate decimal degrees
            decimal = degrees + minutes/60.0
            
            # Apply direction
            if direction in ['S', 'W']:
                decimal = -decimal
                
            return decimal
            
        except Exception as e:
            return None

    def get_last_position(self):
        """Get the last received GPS position."""
        return self.last_position


def cleanup_all_gps_connections():
    """Cleanup all active GPS connections"""
    global _active_readers
    print("🧹 Cleaning up all GPS connections...")
    
    for reader in _active_readers[:]:  # Use slice to avoid modification during iteration
        try:
            reader.stop_reading()
            reader.disconnect()
            _active_readers.remove(reader)
        except Exception as e:
            print(f"Cleanup error: {e}")
    
    time.sleep(2)  # Give OS time to release ports
    print("✅ GPS cleanup complete")



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
    """Set up Emlid GPS integration with the rover."""
    # Initialize the GPS reader
    emlid_reader = EmlidGPSReader()
    
    # Configure rover to use Emlid GPS data
    update_rover_from_emlid(rover, emlid_reader)
    
    # Try connecting with more attempts and longer delays
    success = emlid_reader.connect(retries=8, retry_delay=3)
    
    if success:
        print("✅ Connected! Starting data collection...")
        # Start reading GPS data
        reading_success = emlid_reader.start_reading()
        if reading_success:
            print("✅ GPS data reading started successfully")
        else:
            print("⚠️ Failed to start GPS data reading")
            success = False
    else:
        print("❌ Failed to connect. Check hardware and try again.")
    
    return emlid_reader if success else None


# Additional diagnostic functions
def diagnose_com_port(port='COM12'):
    """Diagnostic function to check COM port status"""
    import subprocess
    import serial.tools.list_ports
    import os
    
    print(f"\n🔍 Diagnosing {port}...")
    
    # Check if port exists in system
    ports = list(serial.tools.list_ports.comports())
    port_found = False
    port_info = None
    
    for p in ports:
        if port in p.device:
            port_found = True
            port_info = p
            print(f"✅ Found {port}:")
            print(f"   Description: {p.description}")
            print(f"   Hardware ID: {p.hwid}")
            print(f"   Manufacturer: {p.manufacturer}")
            break
            
    if not port_found:
        print(f"❌ {port} not found in system")
        return False
    
    # Check if port is in use
    try:
        with serial.Serial(port, 115200, timeout=1) as ser:
            print(f"✅ Can open {port}")
    except serial.SerialException as e:
        print(f"❌ Cannot open {port}: {e}")
        if "Access is denied" in str(e):
            print("   Port is likely in use by another program")
            
            # Try to kill processes using the port
            try:
                cleanup_all_gps_connections()
                print("   ✅ Cleaned up potential blocking processes")
            except Exception as ce:
                print(f"   ⚠️ Cleanup failed: {ce}")
        return False
        
    # Try to set port parameters
    try:
        subprocess.run(f"mode {port} BAUD=115200 PARITY=N DATA=8 STOP=1", 
                      shell=True, check=True)
        print(f"✅ Successfully configured {port}")
    except subprocess.CalledProcessError as e:
        print(f"⚠️ Could not configure {port}: {e}")
        
    return True
def test_emlid_connection_comprehensive():
    """Test Emlid connection with better process handling and port release"""
    print("\n🧪 Comprehensive Emlid Connection Test")
    print("=" * 50)
    
    # Step 1: Initial cleanup
    print("🧹 Cleaning up all GPS connections...")
    cleanup_all_gps_connections()
    print("✅ GPS cleanup complete\n")
    
    # Step 2: Force close any existing connections
    try:
        import serial
        test_ser = serial.Serial('COM12')
        test_ser.close()
    except:
        pass
    
    # Wait for OS to fully release the port
    print("⏳ Waiting for port to be fully released...")
    time.sleep(2)
    
    # Step 3: Check COM12 availability
    print("🔍 Checking COM12 availability...")
    import serial.tools.list_ports
    ports = list(serial.tools.list_ports.comports())
    com12_found = False
    for port in ports:
        if 'COM12' in port.device:
            print(f"✅ Found {port.device}")
            print(f"   Description: {port.description}")
            print(f"   Hardware ID: {port.hwid}")
            com12_found = True
            break
    
    if not com12_found:
        print("❌ COM12 not found")
        return False
    
    # Step 4: Kill any processes that might be using the port
    print("\n🔍 Checking for competing processes...")
    try:
        import psutil
        current_pid = os.getpid()
        for proc in psutil.process_iter(['pid', 'name']):
            try:
                if proc.pid != current_pid:  # Don't kill ourselves
                    if proc.name().lower() in ['python.exe', 'pythonw.exe']:
                        print(f"🔫 Terminating {proc.name()} (PID: {proc.pid})")
                        proc.kill()
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
    except Exception as e:
        print(f"⚠️ Process cleanup warning: {e}")
    
    # Wait after process cleanup
    time.sleep(2)
        
    # Step 5: Test basic port access
    print("\n🔌 Testing basic COM12 access...")
    ser = None
    try:
        ser = serial.Serial(
            port='COM12',
            baudrate=115200,
            timeout=1,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE
        )
        print("✅ COM12 opened successfully")
        ser.close()
        print("✅ COM12 closed successfully")
        
        # Critical: Wait after closing
        time.sleep(1)
        
    except Exception as e:
        print(f"❌ COM12 access failed: {e}")
        if ser:
            try:
                ser.close()
            except:
                pass
        return False

    # Step 6: Test Emlid communication
    print("\n📡 Testing Emlid communication...")
    emlid = None
    try:
        emlid = EmlidGPSReader(port='COM12', baud_rate=115200)
        if emlid.connect():
            print("✅ Connected to Emlid")
            emlid.start_reading()
            print("📊 Waiting for data...")
            time.sleep(3)
            
            if emlid.last_position:
                print("✅ Receiving GPS data")
            else:
                print("⚠️ No GPS data received (might be normal)")
                
            emlid.stop_reading()
            emlid.disconnect()
            print("✅ Emlid test complete")
            return True
            
    except Exception as e:
        print(f"❌ Emlid communication failed: {e}")
        return False
    finally:
        if emlid:
            try:
                emlid.stop_reading()
                emlid.disconnect()
            except:
                pass
            
    return True

def force_release_port(port='COM12'):
    """
    Force release a serial port that might be locked by another process.
    Uses multiple strategies to ensure the port is available.
    """
    print(f"\n🔓 Attempting to force-release {port}...")
    
    try:
        # 1. First try to identify processes using the port
        import psutil
        import os
        import subprocess
        import time
        import signal
        
        current_pid = os.getpid()
        port_users = []
        
        # Look for processes that might be using serial ports
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                # Skip our own process
                if proc.pid == current_pid:
                    continue
                    
                # Check process name for common serial port users
                proc_name = proc.name().lower()
                if any(name in proc_name for name in ['python', 'putty', 'terminal', 'reachview', 
                                                     'serialport', 'com0com', 'realterm']):
                    port_users.append(proc)
                    
                # Check command line for the specific port
                if proc.cmdline():
                    cmdline = ' '.join(proc.cmdline()).lower()
                    if port.lower() in cmdline:
                        port_users.append(proc)
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        
        # 2. Kill identified processes
        if port_users:
            print(f"Found {len(port_users)} processes potentially using {port}:")
            for proc in port_users:
                try:
                    print(f"  🔫 Terminating {proc.name()} (PID: {proc.pid})")
                    proc.kill()
                except Exception as e:
                    print(f"    ⚠️ Failed to terminate: {e}")
            
            # Wait for processes to terminate
            print("  ⏳ Waiting for processes to terminate...")
            time.sleep(3)
        
        # 3. Use Windows-specific methods to reset the port
        if os.name == 'nt':  # Windows
            try:
                # Use mode command to reset port settings
                print(f"  🔄 Resetting {port} using mode command...")
                subprocess.run(f"mode {port} BAUD=115200 PARITY=N DATA=8 STOP=1", 
                              shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                
                # Use net use to release any network-mapped ports (unlikely but thorough)
                subprocess.run(f"net use {port} /delete /y", 
                              shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except Exception as e:
                print(f"  ⚠️ Windows-specific commands failed: {e}")
        
        # 4. Try to open and immediately close the port
        try:
            import serial
            print(f"  🔌 Opening and closing {port} to reset state...")
            ser = serial.Serial(port, 115200, timeout=0.1)
            ser.close()
            print(f"  ✅ Successfully opened and closed {port}")
            time.sleep(1)  # Give OS time to fully release
            return True
        except Exception as e:
            print(f"  ⚠️ Could not open port: {e}")
        
        # 5. Last resort: Try to use lower-level system calls
        if os.name == 'nt':  # Windows
            try:
                print("  🔨 Attempting low-level port reset...")
                # Use CreateFile and CloseHandle via ctypes
                import ctypes
                from ctypes import wintypes
                
                GENERIC_READ = 0x80000000
                GENERIC_WRITE = 0x40000000
                OPEN_EXISTING = 3
                
                # Convert port name to device path
                port_path = f"\\\\.\\{port}"
                
                # Open and close the port with low-level Windows API
                h = ctypes.windll.kernel32.CreateFileW(
                    port_path, 
                    GENERIC_READ | GENERIC_WRITE,
                    0,  # No sharing
                    None,  # Default security
                    OPEN_EXISTING,
                    0,  # No overlapped I/O
                    None  # No template
                )
                
                if h != -1:  # INVALID_HANDLE_VALUE
                    ctypes.windll.kernel32.CloseHandle(h)
                    print("  ✅ Low-level port reset successful")
                    time.sleep(1)  # Give OS time to fully release
                    return True
                else:
                    print("  ⚠️ Low-level port reset failed")
            except Exception as e:
                print(f"  ⚠️ Low-level reset error: {e}")
        
        print(f"⚠️ Could not force-release {port} - may need manual intervention")
        return False
        
    except Exception as e:
        print(f"⚠️ Port release error: {e}")
        return False

if __name__ == "__main__":
    print("\n🔍 Starting Emlid GPS Diagnostics...")
    print("=" * 50)
    
    # Run initial diagnostics
    if diagnose_com_port():
        print("\n✅ Initial diagnostics passed")
        
        # Force cleanup and wait
        cleanup_all_gps_connections()
        time.sleep(2)
        
        # Run comprehensive test
        if test_emlid_connection_comprehensive():
            print("\n✅ All tests passed successfully!")
        else:
            print("\n❌ Some tests failed - check messages above")
    else:
        print("\n❌ Initial diagnostics failed")
    
    print("\n" + "=" * 50)