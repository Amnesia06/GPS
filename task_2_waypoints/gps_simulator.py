import serial
import time
from datetime import datetime
import math
import os
import csv

class RTKGPSRover:
    def __init__(self, port='COM8', baudrate=115200, log_data=True, log_path='gps_logs'):
        # Configure the serial connection
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=1
        )
        
        # Data storage
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0
        self.fix_quality = 0
        self.satellites = 0
        self.hdop = 0.0
        self.speed = 0.0  # in knots
        self.course = 0.0  # in degrees
        self.fix_time = ""
        self.log_data = log_data
        
        # Quality indicator strings
        self.fix_quality_str = {
            0: "Invalid",
            1: "GPS Fix",
            2: "DGPS Fix",
            4: "RTK Fixed",
            5: "RTK Float",
            6: "Estimated (DR) Fix"
        }
        
        # Logging setup
        if log_data:
            self.log_path = log_path
            if not os.path.exists(log_path):
                os.makedirs(log_path)
            
            # Create a new log file with timestamp
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.log_file = os.path.join(log_path, f"gps_log_{timestamp}.csv")
            
            # Create and initialize the CSV file
            with open(self.log_file, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['Timestamp', 'Latitude', 'Longitude', 'Altitude', 'Fix_Quality', 
                                'Satellites', 'HDOP', 'Speed_knots', 'Speed_kmh', 'Course',
                                'Fix_Time'])
        
        print(f"Connected to: {self.ser.port}")
        print(f"RTK GPS Rover initialized. {'Logging enabled.' if log_data else 'Logging disabled.'}")

    def parse_gga(self, message):
        """Parse GGA message (Global Positioning System Fix Data)"""
        parts = message.split(',')
        if len(parts) < 15:
            return False
        
        # Extract time
        if parts[1]:
            hours = parts[1][0:2]
            minutes = parts[1][2:4]
            seconds = parts[1][4:]
            self.fix_time = f"{hours}:{minutes}:{seconds}"
        
        # Extract latitude and longitude if available
        if parts[2] and parts[4]:
            # Extract latitude
            lat_deg = float(parts[2][:2])
            lat_min = float(parts[2][2:])
            self.latitude = lat_deg + (lat_min / 60.0)
            if parts[3] == 'S':
                self.latitude *= -1

            # Extract longitude
            lon_deg = float(parts[4][:3])
            lon_min = float(parts[4][3:])
            self.longitude = lon_deg + (lon_min / 60.0)
            if parts[5] == 'W':
                self.longitude *= -1
        
        # Extract other data
        if parts[6]:
            self.fix_quality = int(parts[6])
        
        if parts[7]:
            self.satellites = int(parts[7])
        
        if parts[8]:
            self.hdop = float(parts[8])
        
        if parts[9]:
            self.altitude = float(parts[9])
        
        return True

    def parse_rmc(self, message):
        """Parse RMC message (Recommended Minimum Navigation Information)"""
        parts = message.split(',')
        if len(parts) < 10:
            return False
        
        # Extract speed and course
        if parts[7]:
            self.speed = float(parts[7])  # Speed over ground in knots
        
        if parts[8]:
            self.course = float(parts[8])  # Course over ground in degrees
        
        return True

    def parse_nmea(self, line):
        """Parse different NMEA sentences"""
        if line.startswith('$GNGGA') or line.startswith('$GPGGA'):
            return self.parse_gga(line)
        elif line.startswith('$GNRMC') or line.startswith('$GPRMC'):
            return self.parse_rmc(line)
        return False

    def log_to_csv(self):
        """Log current GPS data to CSV file"""
        if not self.log_data:
            return
            
        with open(self.log_file, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            writer.writerow([
                timestamp,
                f"{self.latitude:.8f}",
                f"{self.longitude:.8f}",
                f"{self.altitude:.2f}",
                self.fix_quality,
                self.satellites,
                f"{self.hdop:.1f}",
                f"{self.speed:.2f}",
                f"{self.speed * 1.852:.2f}",  # Convert knots to km/h
                f"{self.course:.1f}",
                self.fix_time
            ])

    def calculate_distance(self, lat1, lon1, lat2, lon2):
        """Calculate distance between two coordinates using Haversine formula"""
        # Radius of the Earth in meters
        R = 6371000.0
        
        # Convert coordinates from degrees to radians
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)
        
        # Differences
        dlat = lat2_rad - lat1_rad
        dlon = lon2_rad - lon1_rad
        
        # Haversine formula
        a = math.sin(dlat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        distance = R * c
        
        return distance

    def run(self):
        """Main loop to continuously read and process GPS data"""
        last_lat = self.latitude
        last_lon = self.longitude
        last_time = datetime.now()
        
        try:
            while True:
                # Flush the input buffer to get fresh data
                self.ser.reset_input_buffer()
                
                # Read line from serial port
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
                
                # Parse NMEA sentence if valid
                if line and line[0] == '$' and '*' in line:
                    if self.parse_nmea(line):
                        # Get current timestamp
                        current_time = datetime.now()
                        timestamp = current_time.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                        
                        # Calculate time delta for speed estimation
                        time_delta = (current_time - last_time).total_seconds()
                        
                        # Calculate speed based on position change (as a cross-check)
                        if time_delta > 0 and last_lat != 0 and self.latitude != 0:
                            dist = self.calculate_distance(last_lat, last_lon, self.latitude, self.longitude)
                            calc_speed_ms = dist / time_delta
                            calc_speed_kmh = calc_speed_ms * 3.6
                        else:
                            calc_speed_kmh = 0
                        
                        # Print status
                        if self.fix_quality > 0:
                            quality_text = self.fix_quality_str.get(self.fix_quality, "Unknown")
                            print(f"[{timestamp}] Pos: {self.latitude:.8f}, {self.longitude:.8f} | "
                                  f"Alt: {self.altitude:.2f}m | Fix: {quality_text} | "
                                  f"Sat: {self.satellites} | HDOP: {self.hdop:.1f} | "
                                  f"Speed: {self.speed * 1.852:.2f} km/h | Course: {self.course:.1f}°")
                        else:
                            print(f"[{timestamp}] Waiting for GPS fix...")
                        
                        # Log data if enabled
                        self.log_to_csv()
                        
                        # Update last position for next iteration
                        last_lat = self.latitude
                        last_lon = self.longitude
                        last_time = current_time
                
                time.sleep(0.05)  # 50ms delay
                
        except KeyboardInterrupt:
            print("Exiting...")
            self.ser.close()
            print(f"GPS data logged to: {self.log_file if self.log_data else 'Logging disabled'}")

if __name__ == "__main__":
    # Create RTK GPS rover instance
    # Change the port name if needed
    rover = RTKGPSRover(port='COM8', baudrate=115200, log_data=True)
    
    # Run the main loop
    rover.run()