"""
GNSS Rover Failsafe Module
--------------------------
This module implements the failsafe monitoring and recovery mechanisms for the rover
when dealing with GNSS-related issues. It can read data from an M2 GNSS receiver
or operate in simulation mode when the receiver is not present.

If failsafe conditions are not properly handled, the rover will enter sleep mode
to prevent dangerous operation.
"""

import enum
import time
import threading
import random
import math
import logging
import serial
import json
import os
from typing import Callable, Dict, List, Optional, Tuple, Union
from datetime import datetime
import pynmea2
from enum import Enum

# Configure logging
import logging

csv_formatter = logging.Formatter('%(asctime)s,%(name)s,%(levelname)s,%(message)s')

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s,%(name)s,%(levelname)s,%(message)s',  # Applies only to basicConfig
    handlers=[
        logging.FileHandler(r"F:\GPS\task_2_waypoints\rover_failsafe.csv"),
        logging.StreamHandler()
    ]
)

# Apply formatter explicitly to ensure correct CSV output
for handler in logging.getLogger("rover_failsafe").handlers:
    handler.setFormatter(csv_formatter)

logger = logging.getLogger("rover_failsafe")


# Enum for GPS failsafe reasons
class GPSFailsafeReason(enum.Enum):
    GPS_STALE_DATA = "GPS data stale (>300ms without NMEA fix)"
    GPS_CORRECTION_STALE = "GPS corrections stale (>1000ms old sustained for >5000ms)"
    GPS_DATA_LOSS = "Serial data loss (>300ms without NMEA on COM port)"
    GPS_FIX_INSTABILITY = "Fix-status instability (≥3 FIX→FLOAT drops in 30000ms)"
    GPS_PERSISTENT_DRIFT = "Persistent drift (≥6 drifts >3cm in 20000ms)"
    GPS_POSITION_JUMP = "Position jump (>30cm or >0.5m/s implied speed)"
    GPS_HIGH_DOP = "High DOP (PDOP >3.0 for >5000ms)"
    GPS_WEAK_CONSTELLATION = "Weak constellation (<6 satellites for >5000ms)"
    GPS_MULTIPATH = "Signal multipath (C/N₀ drop >10dB-Hz for >5000ms)"
    RTK_FIX_LOST = "RTK fix lost (>10000ms without RTK-FIX)"
    INTERNET_CONNECTION_SLOW = "Internet Connection Slow"
    INTERNET_CONNECTION_LOST = "Complete internet loss (no corrections >5000ms AND no cell >10000ms)"
    MODULE_COMMUNICATION_FAILURE = "Module communication failure"
    UNKNOWN = "Unknown GPS failure"
    PATH_DEVIATION = "Path deviation (>5cm distance or >5° heading from planned path)"
    LOW_NTRIP_DATA_RATE = "Low NTRIP data rate (<2.4 kbps for >30000ms)"

# Enum for drift severity levels
class DriftSeverity(enum.Enum):
    NONE = 0
    LOW = 1
    MEDIUM = 2
    HIGH = 3

# Enum for drift action responses
class DriftAction(enum.Enum):
    CONTINUE = "Continue operation"
    SLOW_DOWN = "Reduce speed"
    PAUSE = "Pause movement"
    STOP = "Stop and wait for recovery"
    SLEEP = "Enter sleep mode"

class FailsafeModule:
    """
    Implements GNSS failsafe monitoring and handling based on the specified thresholds.
    Can work with a real M2 GNSS receiver or in simulation mode.
    """
    
    def __init__(self, port: str = "COM8", baud_rate: int = 115200, simulation_mode: bool = True, rover=None):
        self.snr_values = {}
        # Initialize timestamps

        self.last_gps_update = time.time()
        self.last_internet_check = time.time()
        self.last_module_comm = time.time()
        self.last_position_check = time.time()
        self.last_correction_update = time.time() 
        
        self.monitoring = False
        self._on_failsafe = None
        self._on_recovery = None


        # Serial connection parameters
        self.port = port
        self.baud_rate = baud_rate
        self.serial_conn = None
        self.simulation_mode = simulation_mode
        
        # GPS status
        self.has_fix = False
        self.fix_type = "NONE"  # NONE, FLOAT, FIX
        self.satellites = 0
        self.hdop = 99.9
        self.pdop = 99.9
        self.vdop = 99.9
        self.snr_values = {}  # Satellite PRN -> SNR value
        self.position = (0.0, 0.0)  # (latitude, longitude)
        self.altitude = 0.0
        
        # Internet and corrections status
        self.internet_connected = False
        self.internet_latency = 999.0
        self.last_correction = time.time() - 1000  # Start with old corrections
        self.correction_age = 999.0
        self.ntrip_data_rate = 0.0  # Data rate in kbps
        self.ntrip_data_rate_check_time = time.time()
        
        # Failsafe monitoring
        self.monitoring_thread = None
        self.stop_thread = False
        self.active_failsafe = None
        self.in_recovery = False
        self.in_sleep_mode = False
        self.sleep_start_time = 0
        self.sleep_duration = 0
        
        # Statistics for monitoring
        self.fix_drops = []  # List of timestamps when FIX->FLOAT drops occurred
        self.position_history = []  # List of (timestamp, position) tuples
        self.drift_events = []  # List of (timestamp, drift_size) tuples
        
        # Callbacks
        self.on_failsafe_triggered = None
        self.on_recovery_attempt = None
        self.on_rover_wakeup = None
        
        # Dependencies
        self.safety_module = None
        self.in_failsafe_mode = False 

        # Add path tracking fields
        self.planned_path = []  # List of waypoints
        self.planned_heading = 0.0  # Planned heading in degrees
        self.current_heading = 0.0  # Current heading in degrees
        self.path_deviation_events = []  # List of (timestamp, distance_dev, heading_dev) tuples
        self.last_waypoint = None  # Last successfully reached waypoint
        

        


        # Configure simulation parameters for testing
        self.simulation_params = {
            "path_deviation_prob": 0.01,  # Probability of path deviation
            "stale_data_prob": 0.005,  # Probability of stale data per check
            "correction_stale_prob": 0.005,  # Probability of stale corrections
            "data_loss_prob": 0.005,  # Probability of serial data loss
            "fix_drop_prob": 0.01,  # Probability of FIX->FLOAT drop
            "drift_prob": 0.01,  # Probability of position drift
            "jump_prob": 0.005,  # Probability of position jump
            "high_dop_prob": 0.01,  # Probability of high DOP
            "weak_constellation_prob": 0.01,  # Probability of weak constellation
            "multipath_prob": 0.01,  # Probability of signal multipath
            "rtk_loss_prob": 0.01,  # Probability of RTK fix loss
            "internet_latency_prob": 0.01,  # Probability of high internet latency
            "internet_loss_prob": 0.005,  # Probability of complete internet loss
            "low_ntrip_data_rate_prob": 0.005,  # Probability of low NTRIP data rate
        }
        
        # Updated thresholds based on requirements
        self.thresholds = {
            "gps_stale_data": 0.3,  # 300ms without NMEA fix
            "gps_correction_stale_age": 0.3,  # 300ms correction age
            "gps_correction_stale_duration": 2.0,  # 5000ms duration
            "gps_data_loss": 0.3,  # 300ms without NMEA on COM port
            "fix_instability_count": 3,  # 3 FIX->FLOAT drops
            "fix_instability_window": 30.0,  # 30000ms window
            "persistent_drift_count": 6,  # 6 drifts
            "persistent_drift_size": 0.03,  # 3cm drift
            "persistent_drift_window": 20.0,  # 20000ms window
            "position_jump_distance": 0.3,  # 30cm jump
            "position_jump_speed": 0.5,  # 0.5m/s implied speed
            "high_dop_threshold": 3.0,  # PDOP > 3.0
            "high_dop_duration": 5.0,  # 5000ms duration
            "weak_constellation_count": 6,  # <6 satellites
            "weak_constellation_duration": 5.0,  # 5000ms duration
            "multipath_drop": 10.0,  # 10dB-Hz drop
            "multipath_duration": 5.0,  # 5000ms duration
            "rtk_fix_lost_duration": 10.0,  # 10000ms without RTK-FIX
            "ntrip_latency_threshold": 1.0,  # 1000ms correction age
            "ntrip_latency_duration": 10.0,  # 10000ms duration
            "internet_loss_corrections": 5.0,  # 5000ms without corrections
            "internet_loss_cellular": 10.0,  # 10000ms without cellular
            "path_deviation_distance": 0.05,  # 5cm distance deviation
            "path_deviation_heading": 5.0,  # 5° heading deviation
            "low_ntrip_data_rate": 2.4,  # 2.4 kbps
            "low_ntrip_data_rate_duration": 30.0,  # 30000ms duration
        }
        
        # Recovery times based on requirements
        self.recovery_times = {
            GPSFailsafeReason.GPS_STALE_DATA: 10,  # 10s to reconnect GNSS
            GPSFailsafeReason.GPS_CORRECTION_STALE: 20,  # 20s to reconnect NTRIP
            GPSFailsafeReason.GPS_DATA_LOSS: 15,  # 15s to reconnect GNSS
            GPSFailsafeReason.GPS_FIX_INSTABILITY: 30,  # 30s to monitor
            GPSFailsafeReason.GPS_PERSISTENT_DRIFT: 20,  # 20s to monitor
            GPSFailsafeReason.GPS_POSITION_JUMP: 10,  # 10s to stop and check
            GPSFailsafeReason.GPS_HIGH_DOP: 15,  # 15s to wait
            GPSFailsafeReason.GPS_WEAK_CONSTELLATION: 20,  # 20s to wait
            GPSFailsafeReason.GPS_MULTIPATH: 5,  # 5s before sleep
            GPSFailsafeReason.RTK_FIX_LOST: 30,  # 30s to monitor
            GPSFailsafeReason.INTERNET_CONNECTION_SLOW: 20,  # 20s to reconnect
            GPSFailsafeReason.INTERNET_CONNECTION_LOST: 30,  # 30s to reconnect
            GPSFailsafeReason.MODULE_COMMUNICATION_FAILURE: 15,  # 15s to reconnect
            GPSFailsafeReason.PATH_DEVIATION: 20,  # 20s to adjust path
            GPSFailsafeReason.LOW_NTRIP_DATA_RATE: 30,  # 30s to reconnect NTRIP
            GPSFailsafeReason.UNKNOWN: 10  # 10s default
        }
        
        # Sleep duration in seconds (5 minutes)
        self.default_sleep_duration = 300

        # ADD: Store rover reference for real GPS data
        self.rover = rover
        self.use_rover_gps = rover is not None and hasattr(rover, 'gps_reader')

        logger.info("FailsafeModule initialized in %s mode with %s GPS source", 
                "simulation" if simulation_mode else "hardware",
                "rover" if self.use_rover_gps else "direct serial")

        

    def set_safety_module(self, safety_module):
        """Set the safety module reference"""
        self.safety_module = safety_module

    def set_callbacks(self, on_failsafe_triggered: Callable = None, 
                     on_recovery_attempt: Callable = None,
                     on_rover_wakeup: Callable = None):
        """Set callback functions"""
        self.on_failsafe_triggered = on_failsafe_triggered
        self.on_recovery_attempt = on_recovery_attempt
        self.on_rover_wakeup = on_rover_wakeup

    def connect_to_gnss(self) -> bool:
        """
        Connect to M2 GNSS receiver through serial port
        Returns True if connection successful, False otherwise
        """
        if self.simulation_mode:
            logger.info("Running in simulation mode, no physical GNSS connection required")
            return True
            
        try:
            self.serial_conn = serial.Serial(self.port, self.baud_rate, timeout=1)
            logger.info(f"Connected to GNSS receiver on {self.port} at {self.baud_rate} baud")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to GNSS receiver: {e}")
            return False

    def read_gnss_data(self) -> Dict:
        """
        Read data from M2 GNSS receiver or rover GPS reader
        Returns a dictionary with parsed GNSS data
        """
        # Priority 1: Use rover's GPS reader if available (real Emlid data)
        if (hasattr(self, 'use_rover_gps') and self.use_rover_gps and 
            self.rover and hasattr(self.rover, 'gps_reader') and 
            self.rover.gps_reader and self.rover.gps_reader.last_position):
            try:
                return self._convert_rover_gps_data(self.rover.gps_reader.last_position)
            except Exception as e:
                logger.error(f"Error reading rover GPS data: {e}")
                # Fall through to other methods

        # Priority 2: Simulation mode
        if self.simulation_mode:
            return self._generate_simulated_gnss_data()
            
        # Priority 3: Direct serial connection (fallback)
        if not self.serial_conn or not self.serial_conn.is_open:
            if not self.connect_to_gnss():
                return {}
                
        try:
            # Read data from serial port
            line = self.serial_conn.readline().decode('ascii', errors='replace').strip()
            if not line:
                return {}
                
            # Parse NMEA sentence
            try:
                msg = pynmea2.parse(line)
                data = self._parse_nmea_message(msg)
                self.last_gps_update = time.time()
                return data
            except pynmea2.ParseError:
                return {}
        except Exception as e:
            logger.error(f"Error reading GNSS data: {e}")
            return {}


    def _convert_rover_gps_data(self, rover_position: Dict) -> Dict:
        """Convert rover GPS data format to failsafe expected format"""
        data = {}
        
        try:
            # Map rover GPS data to failsafe format
            data["timestamp"] = time.time()
            
            # Fix quality mapping
            fix_quality = rover_position.get('fix_quality', 'Unknown')
            if fix_quality == 'RTK Fixed':
                data["fix_quality"] = "FIX"
            elif fix_quality == 'RTK Float':
                data["fix_quality"] = "FLOAT"
            elif fix_quality in ['GPS', 'DGPS']:
                data["fix_quality"] = "GPS"
            else:
                data["fix_quality"] = "NONE"
            
            # Basic GPS data
            data["satellites"] = rover_position.get('satellites', 0)
            data["hdop"] = rover_position.get('hdop', 99.9)
            data["pdop"] = rover_position.get('pdop', 99.9)
            data["vdop"] = rover_position.get('vdop', 99.9)
            
            # Position data
            # Position data
            lat = rover_position.get('latitude', 0.0)
            lon = rover_position.get('longitude', 0.0)
            if lat != 0.0 or lon != 0.0:  # Changed from 'and' to 'or'
                data["position"] = (lat, lon)
            else:
                data["position"] = self.position  # Use existing position as fallback

            
            data["altitude"] = rover_position.get('altitude', 0.0)
            
            # Speed and heading
            data["speed"] = rover_position.get('speed', 0.0)
            data["heading"] = rover_position.get('heading', 0.0)
            
            # Correction age (if available)
            data["correction_age"] = rover_position.get('age_of_corrections', 0.5)
            
            # SNR data (if available)
            satellites_in_view = rover_position.get('satellites_in_view', [])
            snr_values = {}
            for sat in satellites_in_view:
                prn = sat.get('prn', '')
                snr = sat.get('snr', 0)
                if prn and snr > 0:
                    snr_values[prn] = snr
            data["snr_values"] = snr_values
            
            # NTRIP data rate (default to good value for real GPS)
            data["ntrip_data_rate"] = 5.0  # Assume good data rate for real GPS
            
            logger.debug(f"Converted rover GPS data: fix={data['fix_quality']}, sats={data['satellites']}")
            
        except Exception as e:
            logger.error(f"Error converting rover GPS data: {e}")
            # Return minimal data to avoid complete failure
            data = {
                "timestamp": time.time(),
                "fix_quality": "NONE",
                "satellites": 0,
                "hdop": 99.9,
                "pdop": 99.9
            }
        
        return data
    
    def _update_status_from_gnss_data(self, gnss_data: Dict) -> None:
        """Update internal status from GNSS data"""
        try:
            if "fix_quality" in gnss_data:
                self.update_gps_status(
                    has_fix=(gnss_data["fix_quality"] != "NONE"),
                    fix_type=gnss_data["fix_quality"],
                    satellites=gnss_data.get("satellites", self.satellites),
                    hdop=gnss_data.get("hdop", self.hdop),
                    pdop=gnss_data.get("pdop", self.pdop),
                    position=gnss_data.get("position", self.position),
                    altitude=gnss_data.get("altitude", self.altitude)
                )
            
            if "correction_age" in gnss_data:
                self.update_correction_status(age=gnss_data["correction_age"])
                
            if "ntrip_data_rate" in gnss_data:
                self.update_ntrip_data_rate(data_rate=gnss_data["ntrip_data_rate"])
                
            self.update_module_communication()
            
        except Exception as e:
            logger.error(f"Error updating status from GNSS data: {e}")


    def _parse_nmea_message(self, msg) -> Dict:
        """Parse different types of NMEA messages and extract relevant data"""
        data = {}

        try:
            if isinstance(msg, pynmea2.GGA):
                # Global Positioning System Fix Data
                data["fix_type"] = int(msg.gps_qual)
                data["satellites"] = int(msg.num_sats) if msg.num_sats else 0
                data["hdop"] = float(msg.horizontal_dil) if msg.horizontal_dil else 99.9
                data["position"] = (float(msg.latitude), float(msg.longitude))
                data["altitude"] = float(msg.altitude) if msg.altitude else 0.0
                data["correction_age"] = float(msg.age_gps_data) if msg.age_gps_data else 999.0

                # Update fix quality based on GGA quality indicator
                if data["fix_type"] == 4:
                    data["fix_quality"] = "FIX"    # RTK fixed solution
                elif data["fix_type"] == 5:
                    data["fix_quality"] = "FLOAT"  # RTK float solution
                elif data["fix_type"] > 0:
                    data["fix_quality"] = "GPS"    # Standard GPS fix
                else:
                    data["fix_quality"] = "NONE"   # No fix

            elif isinstance(msg, pynmea2.GSA):
                # GPS DOP and active satellites
                data["pdop"] = float(msg.pdop) if msg.pdop else 99.9
                data["hdop"] = float(msg.hdop) if msg.hdop else 99.9
                data["vdop"] = float(msg.vdop) if msg.vdop else 99.9
                data["fix_type_3d"] = int(msg.mode_fix_type) if msg.mode_fix_type else 0

            elif isinstance(msg, pynmea2.GSV):
                # Satellites in view
                if msg.msg_num == 1:  # First message in sequence
                    self.snr_values = {}  # Reset SNR values
                current_time = time.time()
                for sat_index in range(4):  # Each GSV message contains up to 4 satellites
                    sat_num_attr = f"sv_prn_{sat_index + 1}"
                    snr_attr = f"snr_{sat_index + 1}"

                    if hasattr(msg, sat_num_attr) and hasattr(msg, snr_attr):
                        sat_num = getattr(msg, sat_num_attr)
                        snr = getattr(msg, snr_attr)

                        if sat_num and snr:
                            if sat_num not in self.snr_values:
                                self.snr_values[sat_num] = []
                            self.snr_values[sat_num].append((current_time, float(snr)))

                data["snr_values"] = self.snr_values.copy()

            elif isinstance(msg, pynmea2.VTG):
                # Track made good and ground speed
                data["speed"] = float(msg.spd_over_grnd_kmph) / 3.6  # km/h → m/s
                data["track"] = float(msg.true_track) if msg.true_track else 0.0

            elif isinstance(msg, pynmea2.RMC):
                # Recommended minimum specific GPS/Transit data
                if msg.status == 'A':  # A=active, V=void
                    data["position"] = (float(msg.latitude), float(msg.longitude))
                    data["speed"] = float(msg.spd_over_grnd) * 0.514444  # knots → m/s
                    data["track"] = float(msg.true_course) if msg.true_course else 0.0
                    data["timestamp"] = (
                        msg.datetime.timestamp() if msg.datetime else time.time()
                    )

        except Exception as e:
            logger.error(f"Error parsing NMEA message: {e}")

        return data

    def _generate_simulated_gnss_data(self) -> Dict:
        """Generate simulated GNSS data for testing"""
        current_time = time.time()
        data = {}
        
        # Base values
        data["timestamp"] = current_time
        
        # Simulate realistic values with occasional failures based on simulation parameters
        # Add heading information
        if hasattr(self, 'current_heading') and not hasattr(self, 'sim_heading'):
            self.sim_heading = random.uniform(0, 360)
        
        # Simulate heading
        if self.active_failsafe == GPSFailsafeReason.PATH_DEVIATION:
            # Introduce a significant heading deviation
            planned_heading = getattr(self, 'planned_heading', 0)
            heading_deviation = random.uniform(6, 20)  # More than 5 degrees
            if random.random() < 0.5:
                data["heading"] = (planned_heading + heading_deviation) % 360
            else:
                data["heading"] = (planned_heading - heading_deviation) % 360
        else:
            # Normal small heading variations
            if hasattr(self, 'sim_heading'):
                data["heading"] = (self.sim_heading + random.uniform(-3, 3)) % 360
                self.sim_heading = data["heading"]  # Update for next time
            else:
                data["heading"] = random.uniform(0, 360)
        # Fix type
        if self.active_failsafe == GPSFailsafeReason.RTK_FIX_LOST:
            data["fix_quality"] = "FLOAT"
        elif self.active_failsafe == GPSFailsafeReason.GPS_FIX_INSTABILITY:
            # Occasionally switch between FIX and FLOAT
            data["fix_quality"] = "FLOAT" if random.random() < 0.5 else "FIX"
        else:
            # Normally provide RTK fix with occasional drops based on probability
            if random.random() < self.simulation_params["fix_drop_prob"]:
                data["fix_quality"] = "FLOAT"
            else:
                data["fix_quality"] = "FIX"
        
        # Satellites
        if self.active_failsafe == GPSFailsafeReason.GPS_WEAK_CONSTELLATION:
            data["satellites"] = random.randint(3, 5)  # Below threshold of 6
        else:
            data["satellites"] = random.randint(8, 16)  # Normal range
        
        # DOP values
        if self.active_failsafe == GPSFailsafeReason.GPS_HIGH_DOP:
            data["pdop"] = random.uniform(3.5, 6.0)  # Above threshold of 3.0
        else:
            data["pdop"] = random.uniform(1.2, 2.5)  # Normal range
        
        data["hdop"] = data["pdop"] * random.uniform(0.6, 0.8)
        data["vdop"] = data["pdop"] * random.uniform(0.7, 0.9)
        
        # SNR values
        snr_values = {}
        for i in range(1, data["satellites"] + 1):
            base_snr = random.uniform(35, 50)  # Normal range
            if self.active_failsafe == GPSFailsafeReason.GPS_MULTIPATH and random.random() < 0.7:
                base_snr -= random.uniform(12, 20)  # Drop by more than 10 dB-Hz
            snr_values[f"G{i}"] = base_snr
        data["snr_values"] = snr_values
        
        # Position
        if not hasattr(self, 'sim_position') or not self.sim_position:
            self.sim_position = (random.uniform(40.0, 41.0), random.uniform(-74.0, -73.0))
            
        # Handle position drift and jumps
        if self.active_failsafe == GPSFailsafeReason.GPS_POSITION_JUMP:
            # Large jump
            jump_size = random.uniform(0.4, 1.0)
            jump_angle = random.uniform(0, math.pi * 2)
            delta_lat = jump_size * math.cos(jump_angle) / 111111
            delta_lon = jump_size * math.sin(jump_angle) / (111111 * math.cos(math.radians(self.sim_position[0])))
            data["position"] = (self.sim_position[0] + delta_lat, self.sim_position[1] + delta_lon)
        elif self.active_failsafe == GPSFailsafeReason.GPS_PERSISTENT_DRIFT:
            # Persistent small drifts
            drift_size = random.uniform(0.03, 0.10)  # 3-10 cm
            drift_angle = random.uniform(0, math.pi * 2)
            delta_lat = drift_size * math.cos(drift_angle) / 111111
            delta_lon = drift_size * math.sin(drift_angle) / (111111 * math.cos(math.radians(self.sim_position[0])))
            data["position"] = (self.sim_position[0] + delta_lat, self.sim_position[1] + delta_lon)
        else:
            # Normal small variations
            delta_lat = random.uniform(-0.01, 0.01) / 111111  # ~1cm
            delta_lon = random.uniform(-0.01, 0.01) / (111111 * math.cos(math.radians(self.sim_position[0])))
            data["position"] = (self.sim_position[0] + delta_lat, self.sim_position[1] + delta_lon)
            
        self.sim_position = data["position"]  # Update simulated position for next time
        
        # Altitude
        data["altitude"] = random.uniform(100.0, 101.0)
        
        # Speed and heading
        data["speed"] = random.uniform(0.1, 0.5)  # m/s
        data["track"] = random.uniform(0, 360)  # degrees
        
        # Correction age
        if self.active_failsafe == GPSFailsafeReason.GPS_CORRECTION_STALE:
            data["correction_age"] = random.uniform(1.5, 10.0)  # Above threshold (1000ms)
        elif self.active_failsafe == GPSFailsafeReason.INTERNET_CONNECTION_SLOW:
            data["correction_age"] = random.uniform(1.5, 6.0)  # Above threshold (1000ms)
        else:
            data["correction_age"] = random.uniform(0.1, 0.8)  # Normal range
        
        # NTRIP data rate
        if self.active_failsafe == GPSFailsafeReason.LOW_NTRIP_DATA_RATE:
            data["ntrip_data_rate"] = random.uniform(0.5, 2.0)  # Below threshold (2.4 kbps)
        else:
            data["ntrip_data_rate"] = random.uniform(3.0, 10.0)  # Normal range
            
        return data

    def update_path_info(self, planned_path: List[Tuple[float, float]] = None, 
                        planned_heading: float = None,
                        current_heading: float = None,
                        last_waypoint: Tuple[float, float] = None) -> None:
        """Update the current path planning information"""
        if planned_path is not None:
            self.planned_path = planned_path
            
        if planned_heading is not None:
            self.planned_heading = planned_heading
            
        if current_heading is not None:
            self.current_heading = current_heading
            
        if last_waypoint is not None:
            self.last_waypoint = last_waypoint

    def update_gps_status(self, has_fix: bool = None, fix_type: str = None, 
                          satellites: int = None, hdop: float = None, 
                          pdop: float = None, position: Tuple[float, float] = None,
                          altitude: float = None) -> None:
        """Update the current GPS status with new values"""
        current_time = time.time()
        
        # Only update values that are provided
        if has_fix is not None:
            self.has_fix = has_fix
            
        if fix_type is not None:
            # If changing from FIX to FLOAT, record as a drop event
            if self.fix_type == "FIX" and fix_type == "FLOAT":
                self.fix_drops.append(current_time)
                # Clean up old drops (more than 30 seconds old)
                self.fix_drops = [t for t in self.fix_drops if current_time - t <= self.thresholds["fix_instability_window"]]
                
            self.fix_type = fix_type
            
        if satellites is not None:
            self.satellites = satellites
            
        if hdop is not None:
            self.hdop = hdop
            
        if pdop is not None:
            self.pdop = pdop
            
        if position is not None:
            # Record position for drift/jump detection
            self.position_history.append((current_time, position))
            # Clean up old positions (more than 30 seconds old)
            self.position_history = [(t, p) for t, p in self.position_history 
                                   if current_time - t <= 30]
            
            # Check for jumps or drifts if we have previous positions
            if len(self.position_history) > 1:
                prev_time, prev_pos = self.position_history[-2]
                dist = self._calculate_distance(prev_pos, position)
                time_diff = current_time - prev_time
                
                # Only check if time difference is reasonable
                if time_diff > 0:
                    # Detect jumps (>30cm or implied speed >0.5m/s)
                    if dist > self.thresholds["position_jump_distance"] or (dist / time_diff) > self.thresholds["position_jump_speed"]:
                        logger.warning(f"Position jump detected: {dist:.2f}m in {time_diff:.2f}s")
                    
                    # Detect drifts (>3cm)
                    if dist > self.thresholds["persistent_drift_size"]:
                        self.drift_events.append((current_time, dist))
                        # Clean up old drift events (more than 20 seconds old)
                        self.drift_events = [(t, d) for t, d in self.drift_events 
                                           if current_time - t <= self.thresholds["persistent_drift_window"]]
            
            self.position = position
            
        if altitude is not None:
            self.altitude = altitude
            
        self.last_gps_update = current_time

    def update_internet_status(self, connected: bool = None, latency: float = None) -> None:
        """Update the current internet connection status"""
        current_time = time.time()
        
        if connected is not None:
            self.internet_connected = connected
            
        if latency is not None:
            self.internet_latency = latency
            
        self.last_internet_check = current_time

    def update_correction_status(self, age: float = None) -> None:
        """Update the current correction status"""
        current_time = time.time()
        
        if age is not None:
            self.correction_age = age
            self.last_correction = current_time - age
            self.last_correction_update = current_time  

    def update_ntrip_data_rate(self, data_rate: float = None) -> None:
        """Update the current NTRIP data rate in kbps"""
        current_time = time.time()
        
        if data_rate is not None:
            self.ntrip_data_rate = data_rate
            self.ntrip_data_rate_check_time = current_time

    def update_module_communication(self) -> None:
        """Update the timestamp for last successful module communication"""
        self.last_module_comm = time.time()

    def _calculate_distance(self, pos1: Tuple[float, float], pos2: Tuple[float, float]) -> float:
        """
        Calculate the distance between two positions in meters
        Uses Haversine formula for accurate earth distance
        """
        # For simplicity in simulation, use a flat-earth approximation
        # 1 degree latitude = ~111,111 meters
        # 1 degree longitude = ~111,111 * cos(latitude) meters
        
        lat1, lon1 = pos1
        lat2, lon2 = pos2
        
        # Convert to radians
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        lon1_rad = math.radians(lon1)
        lon2_rad = math.radians(lon2)
        
        # Haversine formula
        dlon = lon2_rad - lon1_rad
        dlat = lat2_rad - lat1_rad
        a = (math.sin(dlat/2)**2) + math.cos(lat1_rad) * math.cos(lat2_rad) * (math.sin(dlon/2)**2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        distance = 6371000 * c  # Earth radius in meters
        
        return distance

    def get_failsafe_reason(self) -> Optional[GPSFailsafeReason]:
        """
        Check all failsafe conditions and return the reason if any is triggered
        Returns None if all is well
        """
        current_time = time.time()
        
        # 1. GPS data stale (>300ms without any NMEA fix)
        if current_time - self.last_gps_update > self.thresholds["gps_stale_data"]:
            return GPSFailsafeReason.GPS_STALE_DATA
            
        # 2. GPS corrections stale (>1000ms old sustained for >5000ms)
        if self.correction_age > self.thresholds["gps_correction_stale_age"] and current_time - self.last_correction > self.thresholds["gps_correction_stale_duration"]:
            return GPSFailsafeReason.GPS_CORRECTION_STALE
            
        # 3. Serial data loss (>300ms without NMEA on COM port)
        if current_time - self.last_module_comm > self.thresholds["gps_data_loss"]:
            return GPSFailsafeReason.GPS_DATA_LOSS
            
        # 4. Fix-status instability (≥3 FIX→FLOAT drops in 30000ms)
        if len(self.fix_drops) >= self.thresholds["fix_instability_count"]:
            return GPSFailsafeReason.GPS_FIX_INSTABILITY
            
        # 5. Persistent drift (≥6 drifts >3cm in 20000ms)
        if len(self.drift_events) >= self.thresholds["persistent_drift_count"]:
            return GPSFailsafeReason.GPS_PERSISTENT_DRIFT
            
        # 6. Position jump (>30cm or implied speed >0.5m/s)
        # This is checked during position updates
        if len(self.position_history) >= 2:
            last_time, last_pos = self.position_history[-1]
            prev_time, prev_pos = self.position_history[-2]
            time_diff = last_time - prev_time
            if time_diff > 0:
                dist = self._calculate_distance(prev_pos, last_pos)
                if dist > self.thresholds["position_jump_distance"] or (dist / time_diff) > self.thresholds["position_jump_speed"]:
                    return GPSFailsafeReason.GPS_POSITION_JUMP
                    
        # 7. High DOP (PDOP >3.0 for >5000ms)
        if self.pdop > self.thresholds["high_dop_threshold"] and current_time - self.last_position_check > self.thresholds["high_dop_duration"]:
            return GPSFailsafeReason.GPS_HIGH_DOP
            
        # 8. Weak constellation (<6 satellites for >5000ms)
        if self.satellites < self.thresholds["weak_constellation_count"] and current_time - self.last_position_check > self.thresholds["weak_constellation_duration"]:
            return GPSFailsafeReason.GPS_WEAK_CONSTELLATION
            
        # 9. Signal multipath (C/N₀ drop >10dB-Hz for >5000ms)
        # Check for multipath failsafe
        for prn in list(self.snr_values.keys()):
            # Clean up entries older than 60 seconds to limit memory usage
            self.snr_values[prn] = [(t, snr) for t, snr in self.snr_values[prn] if current_time - t <= 60]
            if not self.snr_values[prn]:  # Remove empty lists
                del self.snr_values[prn]
                continue

            snr_history = self.snr_values[prn]
            if len(snr_history) < 2:  # Need sufficient history to compare
                continue

            # Calculate baseline SNR (average over last 30 seconds)
            baseline_snrs = [snr for t, snr in snr_history if current_time - t <= 30]
            if not baseline_snrs:
                continue
            baseline_avg = sum(baseline_snrs) / len(baseline_snrs)

            # Get recent SNR values (last 5 seconds)
            recent_snrs = [snr for t, snr in snr_history if current_time - t <= self.thresholds["multipath_duration"]]
            # Ensure enough data points (e.g., at least 3) and check if all are below baseline - 10
            if len(recent_snrs) >= 3 and all(snr < baseline_avg - self.thresholds["multipath_drop"] for snr in recent_snrs):
                return GPSFailsafeReason.GPS_MULTIPATH
        
        # 10. RTK fix lost (>10000ms without RTK-FIX)
        if self.fix_type != "FIX" and current_time - self.last_position_check > self.thresholds["rtk_fix_lost_duration"]:
            return GPSFailsafeReason.RTK_FIX_LOST
            
        # 11. High NTRIP latency (avg correction age >1000ms for >10000ms)
        if self.correction_age > self.thresholds["ntrip_latency_threshold"] and current_time - self.last_internet_check > self.thresholds["ntrip_latency_duration"]:
            return GPSFailsafeReason.INTERNET_CONNECTION_SLOW
            
        # 12. Complete internet loss (no corrections >5000ms AND no cell >10000ms)
        if not self.internet_connected and current_time - self.last_correction > self.thresholds["internet_loss_corrections"] and current_time - self.last_internet_check > self.thresholds["internet_loss_cellular"]:
            return GPSFailsafeReason.INTERNET_CONNECTION_LOST
        
        # 13. Low NTRIP data rate (<2.4 kbps for >30000ms)
        if self.ntrip_data_rate < self.thresholds["low_ntrip_data_rate"] and current_time - self.ntrip_data_rate_check_time > self.thresholds["low_ntrip_data_rate_duration"]:
            return GPSFailsafeReason.LOW_NTRIP_DATA_RATE
            
        # 14. Check for path deviation (if we have a planned path)
        if self.planned_path and len(self.planned_path) > 0:
            deviation_detected, distance_dev, heading_dev = self.check_path_deviation(
                self.position, self.current_heading, self.planned_path, self.planned_heading)
            
            if deviation_detected:
                # Record this deviation event
                self.path_deviation_events.append((current_time, distance_dev, heading_dev))
                # Clean up old events (more than 15 seconds old)
                self.path_deviation_events = [(t, d, h) for t, d, h in self.path_deviation_events 
                                            if current_time - t <= 15]
                
                # If we have multiple deviation events in a short time, trigger failsafe
                if len(self.path_deviation_events) >= 3:  # At least 3 events
                    return GPSFailsafeReason.PATH_DEVIATION
            
        # No failsafe triggered
        return None

    def start_monitoring(self) -> None:
        """Start the failsafe monitoring thread"""
        if self.monitoring_thread and self.monitoring_thread.is_alive():
            logger.warning("Monitoring thread is already running")
            return
            
        self.stop_thread = False
        self.monitoring_thread = threading.Thread(target=self._monitoring_loop, daemon=True)
        self.monitoring_thread.start()
        logger.info("Failsafe monitoring started")

    def stop_monitoring(self) -> None:
        """Stop the failsafe monitoring thread"""
        self.stop_thread = True
        if self.monitoring_thread:
            self.monitoring_thread.join(timeout=2)
        logger.info("Failsafe monitoring stopped")

    def _monitoring_loop(self) -> None:
        """Main monitoring loop that runs in a separate thread"""
        check_interval = 0.2  # Check every 200ms
        last_record_time = time.time()
        record_interval = 5.0  # Record status every 5 seconds
        
        while not self.stop_thread:
            try:
                current_time = time.time()
                
                # Priority 1: Use rover GPS data if available (REAL EMLID DATA)
                if self.use_rover_gps and self.rover and hasattr(self.rover, 'gps_reader') and self.rover.gps_reader:
                    try:
                        # Get real GPS data from rover
                        gnss_data = self.read_gnss_data()
                        if gnss_data:
                            self._update_status_from_gnss_data(gnss_data)
                            logger.debug("Updated failsafe status from real rover GPS data")
                        else:
                            logger.warning("No GPS data available from rover")
                            # Set GPS as unavailable
                            self.update_gps_status(has_fix=False, satellites=0, hdop=99.9)
                    except Exception as e:
                        logger.error(f"Error reading rover GPS data: {e}")
                        self.update_gps_status(has_fix=False, satellites=0, hdop=99.9)
                
                # Priority 2: Simulation mode
                elif self.simulation_mode:
                    # Inject simulated failures occasionally
                    if not self.active_failsafe:
                        self._inject_simulated_failure()
                    
                    # Read simulated data
                    gnss_data = self.read_gnss_data()
                    if gnss_data:
                        self._update_status_from_gnss_data(gnss_data)
                
                # Priority 3: Direct serial connection (fallback)
                else:
                    try:
                        gnss_data = self.read_gnss_data()
                        if gnss_data:
                            self._update_status_from_gnss_data(gnss_data)
                        else:
                            logger.warning("No GPS data from direct serial connection")
                            self.update_gps_status(has_fix=False, satellites=0, hdop=99.9)
                    except Exception as e:
                        logger.error(f"Error reading direct serial GPS data: {e}")
                        self.update_gps_status(has_fix=False, satellites=0, hdop=99.9)
                
                # Check for failsafe conditions (regardless of data source)
                if not self.in_recovery:
                    reason = self.get_failsafe_reason()
                    if reason:
                        logger.warning(f"Failsafe condition detected: {reason}")
                        self._handle_failsafe(reason)
                    else:
                        # Reset active failsafe if conditions are good
                        if self.active_failsafe:
                            logger.info("Failsafe conditions cleared")
                            self.active_failsafe = None
                
                # Handle recovery attempts
                if self.in_recovery:
                    self._handle_recovery()
                
                # Log status periodically
                if current_time - last_record_time >= record_interval:
                    self._log_status()
                    last_record_time = current_time
                    
                    # Debug log for real GPS monitoring
                    if self.use_rover_gps:
                        logger.info(f"Rover GPS Status: Fix={self.has_fix}, Sats={self.satellites}, HDOP={self.hdop:.1f}")


                        
                
                # Update last position check time
                self.last_position_check = current_time
                    
                # Sleep for a bit
                time.sleep(check_interval)
                
            except Exception as e:
                logger.error(f"Critical error in monitoring loop: {e}")
                # On critical error, assume GPS is down
                self.update_gps_status(has_fix=False, satellites=0, hdop=99.9)
                time.sleep(1)  # Wait longer on critical error
        
        logger.info("Monitoring loop stopped")


    def _inject_simulated_failure(self) -> None:
        """Inject a simulated failure for testing purposes"""
        # Only run this occasionally
        if random.random() > 0.01:  # 1% chance per check (5 times per second)
            return
            
        # Choose a random failure type weighted by probability
        failure_types = [
            (GPSFailsafeReason.GPS_STALE_DATA, self.simulation_params["stale_data_prob"]),
            (GPSFailsafeReason.GPS_CORRECTION_STALE, self.simulation_params["correction_stale_prob"]),
            (GPSFailsafeReason.GPS_DATA_LOSS, self.simulation_params["data_loss_prob"]),
            (GPSFailsafeReason.GPS_FIX_INSTABILITY, self.simulation_params["fix_drop_prob"]),
            (GPSFailsafeReason.GPS_PERSISTENT_DRIFT, self.simulation_params["drift_prob"]),
            (GPSFailsafeReason.GPS_POSITION_JUMP, self.simulation_params["jump_prob"]),
            (GPSFailsafeReason.GPS_HIGH_DOP, self.simulation_params["high_dop_prob"]),
            (GPSFailsafeReason.GPS_WEAK_CONSTELLATION, self.simulation_params["weak_constellation_prob"]),
            (GPSFailsafeReason.GPS_MULTIPATH, self.simulation_params["multipath_prob"]),
            (GPSFailsafeReason.RTK_FIX_LOST, self.simulation_params["rtk_loss_prob"]),
            (GPSFailsafeReason.INTERNET_CONNECTION_SLOW, self.simulation_params["internet_latency_prob"]),
            (GPSFailsafeReason.INTERNET_CONNECTION_LOST, self.simulation_params["internet_loss_prob"]),
            (GPSFailsafeReason.PATH_DEVIATION, self.simulation_params["path_deviation_prob"]),
            (GPSFailsafeReason.LOW_NTRIP_DATA_RATE, self.simulation_params["low_ntrip_data_rate_prob"]),
        ]
        
        # Calculate total probability for normalization
        total_prob = sum(prob for _, prob in failure_types)
        if total_prob <= 0:
            return
            
        # Normalize probabilities
        normalized_probs = [prob/total_prob for _, prob in failure_types]
        
        # Choose a failure type based on probability
        chosen_failure = random.choices(
            [failure for failure, _ in failure_types],
            weights=normalized_probs,
            k=1
        )[0]
        
        # Set the active failsafe
                # Set the active failsafe
        logger.info(f"Injecting simulated failure: {chosen_failure.value}")
        self.active_failsafe = chosen_failure
        
        # Make the failure persist for a realistic time
        # It will be cleared when recovery is successful
        # The _handle_failsafe method will be called on the next monitoring cycle

    def _handle_failsafe(self, reason: GPSFailsafeReason) -> None:
        """Handle a detected failsafe condition"""
        if self.active_failsafe == reason:
            # Already handling this failsafe
            return
            
        logger.warning(f"GNSS Failsafe triggered: {reason.value}")
        self.active_failsafe = reason
        
        # Call the callback if registered
        if self.on_failsafe_triggered:
            try:
                self.on_failsafe_triggered(reason)
            except Exception as e:
                logger.error(f"Error in failsafe callback: {e}")
        
        # Start recovery process
        self.start_recovery()

    def start_recovery(self) -> None:
        """Start the failsafe recovery process"""
        if self.in_recovery:
            logger.warning("Already in recovery mode, ignoring new recovery request")
            return
            
        self.in_recovery = True
        logger.info(f"Starting recovery for: {self.active_failsafe.value}")
        
        # Call the recovery callback if registered
        if self.on_recovery_attempt:
            try:
                self.on_recovery_attempt(self.active_failsafe)
            except Exception as e:
                logger.error(f"Error in recovery callback: {e}")
        
        # Start recovery thread
        recovery_thread = threading.Thread(target=self._recovery_process, daemon=True)
        recovery_thread.start()

    def _recovery_process(self) -> None:
        """
        The recovery process runs in a separate thread.
        It tries different recovery strategies depending on the failsafe reason.
        """
        try:
            # Get recovery time for this failure type
            recovery_time = self.recovery_times.get(self.active_failsafe, 15)
            
            # Strategy depends on the failure type
            if self.active_failsafe == GPSFailsafeReason.GPS_STALE_DATA:
                self._attempt_reconnect_gnss()
            elif self.active_failsafe == GPSFailsafeReason.GPS_DATA_LOSS:
                self._attempt_reconnect_gnss()
            elif self.active_failsafe == GPSFailsafeReason.MODULE_COMMUNICATION_FAILURE:
                self._attempt_reconnect_gnss()
            elif self.active_failsafe in [GPSFailsafeReason.INTERNET_CONNECTION_SLOW, 
                                         GPSFailsafeReason.INTERNET_CONNECTION_LOST]:
                self._attempt_reconnect_internet()
            elif self.active_failsafe in [GPSFailsafeReason.GPS_CORRECTION_STALE, 
                                         GPSFailsafeReason.LOW_NTRIP_DATA_RATE]:
                self._attempt_reconnect_ntrip()
            elif self.active_failsafe == GPSFailsafeReason.PATH_DEVIATION:
                self._attempt_path_recovery()
            
            # Wait for conditions to improve up to recovery_time seconds
            start_time = time.time()
            while time.time() - start_time < recovery_time:
                # Check if the failsafe condition is still active
                current_reason = self.get_failsafe_reason()
                
                if current_reason is None or current_reason != self.active_failsafe:
                    # Condition is resolved or changed
                    logger.info(f"Recovery successful for: {self.active_failsafe.value}")
                    self.active_failsafe = None
                    self.in_recovery = False
                    return
                    
                # Sleep a bit before checking again
                time.sleep(1)
                
            # If we get here, recovery failed within the time limit
            logger.error(f"Recovery failed for: {self.active_failsafe.value}, entering sleep mode")
            self.enter_sleep_mode()
            
        except Exception as e:
            logger.error(f"Error in recovery process: {e}")
            self.enter_sleep_mode()  # Safety measure
        finally:
            self.in_recovery = False

    def _attempt_reconnect_gnss(self) -> None:
        """Attempt to reconnect to the GNSS receiver"""
        logger.info("Attempting to reconnect to GNSS receiver")
        
        if self.simulation_mode:
            # In simulation mode, just wait for the simulated failure to clear
            logger.info("Simulation mode: Waiting for GNSS reconnection")
            return
            
        # Close existing connection if any
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.close()
            except Exception as e:
                logger.error(f"Error closing serial connection: {e}")
                
        # Try to reconnect
        try:
            self.serial_conn = serial.Serial(self.port, self.baud_rate, timeout=1)
            logger.info(f"Reconnected to GNSS receiver on {self.port}")
        except Exception as e:
            logger.error(f"Failed to reconnect to GNSS receiver: {e}")

    def _attempt_reconnect_internet(self) -> None:
        """Attempt to reconnect to the internet"""
        logger.info("Attempting to reconnect to internet")
        
        if self.simulation_mode:
            # In simulation mode, just wait for the simulated failure to clear
            logger.info("Simulation mode: Waiting for internet reconnection")
            return
            
        # In a real implementation, this would attempt to reconnect the cellular modem
        # or other internet connection mechanism

    def _attempt_reconnect_ntrip(self) -> None:
        """Attempt to reconnect to the NTRIP server"""
        logger.info("Attempting to reconnect to NTRIP server")
        
        if self.simulation_mode:
            # In simulation mode, just wait for the simulated failure to clear
            logger.info("Simulation mode: Waiting for NTRIP reconnection")
            return
            
        # In a real implementation, this would attempt to reconnect to the NTRIP server

    def _attempt_path_recovery(self) -> None:
        """Attempt to recover from path deviation"""
        logger.info("Attempting to recover from path deviation")
        
        if self.simulation_mode:
            # In simulation mode, just wait for the simulated failure to clear
            logger.info("Simulation mode: Adjusting path")
            return
            
        # In a real implementation, this would:
        # 1. Alert the operator
        # 2. Attempt to adjust the path
        # 3. If unsuccessful, return to the last waypoint
        if self.last_waypoint:
            logger.info(f"Returning to last waypoint: {self.last_waypoint}")
            # Code to navigate back to last waypoint would go here

    def enter_sleep_mode(self, sleep_duration: int = None) -> None:
        """
        Enter sleep mode for a specified duration (default 5 minutes).
        In sleep mode, the rover will stop all operations to prevent dangerous behavior.
        """
        if self.in_sleep_mode:
            logger.warning("Already in sleep mode, ignoring new sleep request")
            return
        
        # Use the default sleep duration if none specified
        if sleep_duration is None:
            sleep_duration = self.default_sleep_duration
            
        logger.warning(f"Entering sleep mode for {sleep_duration} seconds")
        self.in_sleep_mode = True
        self.sleep_start_time = time.time()
        self.sleep_duration = sleep_duration
        
        # Signal to safety module if available
        if self.safety_module:
            try:
                self.safety_module.emergency_stop("GNSS Failsafe: " + self.active_failsafe.value)
            except Exception as e:
                logger.error(f"Error signaling safety module: {e}")
        
        # Start the sleep thread
        sleep_thread = threading.Thread(target=self._sleep_process, daemon=True)
        sleep_thread.start()

    def _sleep_process(self) -> None:
        """Process that runs during sleep mode"""
        try:
            # Sleep for the specified duration
            time.sleep(self.sleep_duration)
            
            # Wake up
            self.wake_up()
            
        except Exception as e:
            logger.error(f"Error in sleep process: {e}")
            # Try to wake up anyway
            self.wake_up()

    def wake_up(self) -> None:
        """Wake up from sleep mode"""
        if not self.in_sleep_mode:
            logger.warning("Not in sleep mode, ignoring wake up request")
            return
            
        logger.info("Waking up from sleep mode")
        self.in_sleep_mode = False
        self.active_failsafe = None
        
        # Reset all status counters and histories
        self.fix_drops = []
        self.position_history = []
        self.drift_events = []
        self.path_deviation_events = []
        
        # Signal to safety module if available
        if self.safety_module:
            try:
                self.safety_module.clear_emergency("GNSS Failsafe wake up")
            except Exception as e:
                logger.error(f"Error signaling safety module: {e}")
        
        # Call the wake up callback if registered
        if self.on_rover_wakeup:
            try:
                self.on_rover_wakeup()
            except Exception as e:
                logger.error(f"Error in wake up callback: {e}")

    def get_drift_severity(self) -> DriftSeverity:
        """
        Get the current drift severity level based on drift events
        Returns a DriftSeverity enum value
        """
        if not self.drift_events:
            return DriftSeverity.NONE
            
        # Calculate average drift size
        avg_drift = sum(d for _, d in self.drift_events) / len(self.drift_events)
        
        # Determine severity based on average drift and number of events
        if avg_drift < 0.05 and len(self.drift_events) < 3:
            return DriftSeverity.LOW
        elif avg_drift < 0.10 and len(self.drift_events) < 5:
            return DriftSeverity.MEDIUM
        else:
            return DriftSeverity.HIGH

    def get_recommended_action(self) -> DriftAction:
        """
        Get the recommended action based on current GPS status
        Returns a DriftAction enum value
        """
        # If already in sleep mode, recommend sleep
        if self.in_sleep_mode:
            return DriftAction.SLEEP
            
        # If an active failsafe is being handled, check if recovery is in progress
        if self.active_failsafe:
            if self.in_recovery:
                return DriftAction.PAUSE
            else:
                return DriftAction.STOP
                
        # Otherwise, check drift severity
        drift_severity = self.get_drift_severity()
        
        if drift_severity == DriftSeverity.NONE:
            return DriftAction.CONTINUE
        elif drift_severity == DriftSeverity.LOW:
            return DriftAction.CONTINUE
        elif drift_severity == DriftSeverity.MEDIUM:
            return DriftAction.SLOW_DOWN
        else:  # HIGH
            return DriftAction.PAUSE

    def _log_status(self) -> None:
        """Log the current status for monitoring"""
        status = {
            "timestamp": datetime.now().isoformat(),
            "has_fix": self.has_fix,
            "fix_type": self.fix_type,
            "satellites": self.satellites,
            "pdop": self.pdop,
            "hdop": self.hdop,
            "position": self.position,
            "internet_connected": self.internet_connected,
            "correction_age": self.correction_age,
            "ntrip_data_rate": self.ntrip_data_rate,
            "active_failsafe": self.active_failsafe.value if self.active_failsafe else None,
            "in_recovery": self.in_recovery,
            "in_sleep_mode": self.in_sleep_mode,
            "drift_severity": self.get_drift_severity().value,
            "recommended_action": self.get_recommended_action().value
        }
        
        logger.info(f"GNSS Status: {json.dumps(status, default=str)}")
        
        # In a real implementation, this data might be sent to a monitoring system
        
    def get_status_report(self) -> Dict:
        """
        Generate a detailed status report for external systems
        Returns a dictionary with all relevant status information
        """
        current_time = time.time()
        
        status = {
            "timestamp": datetime.now().isoformat(),
            "gps": {
                "has_fix": self.has_fix,
                "fix_type": self.fix_type,
                "satellites": self.satellites,
                "pdop": self.pdop,
                "hdop": self.hdop,
                "vdop": self.vdop,
                "position": self.position,
                "altitude": self.altitude,
                "last_update": self.last_gps_update,
                "time_since_update": current_time - self.last_gps_update
            },
            "internet": {
                "connected": self.internet_connected,
                "latency": self.internet_latency,
                "last_check": self.last_internet_check,
                "time_since_check": current_time - self.last_internet_check
            },
            "corrections": {
                "age": self.correction_age,
                "last_update": self.last_correction,
                "time_since_update": current_time - self.last_correction,
                "ntrip_data_rate": self.ntrip_data_rate
            },
            "failsafe": {
                "active": self.active_failsafe.value if self.active_failsafe else None,
                "in_recovery": self.in_recovery,
                "in_sleep_mode": self.in_sleep_mode,
                "sleep_start_time": self.sleep_start_time,
                "sleep_duration": self.sleep_duration,
                "time_in_sleep": current_time - self.sleep_start_time if self.in_sleep_mode else 0
            },
            "drift": {
                "severity": self.get_drift_severity().value,
                "recommended_action": self.get_recommended_action().value,
                "recent_events": len(self.drift_events),
                "fix_drops": len(self.fix_drops)
            },
            "path": {
                "deviation_events": len(self.path_deviation_events),
                "has_planned_path": len(self.planned_path) > 0,
                "last_waypoint": self.last_waypoint
            }
        }
        
        return status

    def check_path_deviation(self, current_position: Tuple[float, float], 
                         current_heading: float, 
                         planned_path: List[Tuple[float, float]],
                         planned_heading: float) -> Tuple[bool, float, float]:
        """
        Check if the rover has deviated from its planned path.
        
        Args:
                        current_position: Current (latitude, longitude) position
            current_heading: Current heading in degrees
            planned_path: List of (latitude, longitude) waypoints for the planned path
            planned_heading: Planned heading in degrees
            
        Returns:
            Tuple of (deviation_detected, distance_deviation, heading_deviation)
        """
        # Find the closest point on the planned path
        min_distance = float('inf')
        closest_index = 0
        
        for i, waypoint in enumerate(planned_path):
            distance = self._calculate_distance(current_position, waypoint)
            if distance < min_distance:
                min_distance = distance
                closest_index = i
        
        # Calculate the heading of the planned path at this point
        if closest_index < len(planned_path) - 1:
            next_point = planned_path[closest_index + 1]
            planned_segment_heading = self._calculate_heading(
                planned_path[closest_index], next_point)
        else:
            # Use the provided planned heading if we're at the last waypoint
            planned_segment_heading = planned_heading
        
        # Calculate heading deviation (normalize to -180 to 180 degrees)
        heading_diff = current_heading - planned_segment_heading
        while heading_diff > 180:
            heading_diff -= 360
        while heading_diff < -180:
            heading_diff += 360
        heading_deviation = abs(heading_diff)
        
        # Check against thresholds
        distance_deviation = min_distance
        deviation_detected = (distance_deviation > self.thresholds["path_deviation_distance"] or  # 5 cm threshold
                            heading_deviation > self.thresholds["path_deviation_heading"])       # 5 degrees threshold
        
        return deviation_detected, distance_deviation, heading_deviation

    def _calculate_heading(self, point1: Tuple[float, float], point2: Tuple[float, float]) -> float:
        """
        Calculate the heading from point1 to point2 in degrees from north.
        
        Args:
            point1: Starting position (latitude, longitude)
            point2: Ending position (latitude, longitude)
            
        Returns:
            Heading in degrees (0-360)
        """
        lat1, lon1 = math.radians(point1[0]), math.radians(point1[1])
        lat2, lon2 = math.radians(point2[0]), math.radians(point2[1])
        
        # Calculate heading using Great Circle formula
        dlon = lon2 - lon1
        x = math.sin(dlon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        
        heading = math.degrees(math.atan2(x, y))
        # Normalize to 0-360 degrees
        return (heading + 360) % 360


class SimulationExample:
    """
    Example usage of the FailsafeModule in simulation mode
    """
    
    def __init__(self):
        self.failsafe = FailsafeModule(simulation_mode=True)
        self.failsafe.set_callbacks(
            on_failsafe_triggered=self.on_failsafe,
            on_recovery_attempt=self.on_recovery,
            on_rover_wakeup=self.on_wakeup
        )
        
    def on_failsafe(self, reason):
        print(f"FAILSAFE TRIGGERED: {reason.value}")
        
    def on_recovery(self, reason):
        print(f"RECOVERY STARTED: {reason.value}")
        
    def on_wakeup(self):
        print("ROVER WAKING UP")
        
    def run_simulation(self, duration=300):
        """Run a simulation for the specified duration in seconds"""
        print(f"Starting GNSS failsafe simulation for {duration} seconds")
        
        # Start monitoring
        self.failsafe.start_monitoring()
        
        try:
            start_time = time.time()
            while time.time() - start_time < duration:
                # Generate some simulated GNSS data
                if not self.failsafe.in_sleep_mode:
                    gnss_data = self.failsafe._generate_simulated_gnss_data()
                    
                    # Update status based on simulated data
                    if "fix_quality" in gnss_data:
                        self.failsafe.update_gps_status(
                            has_fix=(gnss_data["fix_quality"] != "NONE"),
                            fix_type=gnss_data["fix_quality"],
                            satellites=gnss_data.get("satellites", self.failsafe.satellites),
                            hdop=gnss_data.get("hdop", self.failsafe.hdop),
                            pdop=gnss_data.get("pdop", self.failsafe.pdop),
                            position=gnss_data.get("position", self.failsafe.position),
                            altitude=gnss_data.get("altitude", self.failsafe.altitude)
                        )
                    
                    if "correction_age" in gnss_data:
                        self.failsafe.update_correction_status(age=gnss_data["correction_age"])
                        
                    if "ntrip_data_rate" in gnss_data:
                        self.failsafe.update_ntrip_data_rate(data_rate=gnss_data["ntrip_data_rate"])
                        
                    # Simulate random internet status
                    if random.random() < 0.01:  # 1% chance to change internet status
                        self.failsafe.update_internet_status(
                            connected=(random.random() < 0.9),  # 90% chance of being connected
                            latency=random.uniform(50, 200)
                        )
                    
                    self.failsafe.update_module_communication()
                
                # Print status occasionally
                if int(time.time()) % 5 == 0:
                    status = self.failsafe.get_status_report()
                    action = self.failsafe.get_recommended_action()
                    failsafe_status = "ACTIVE" if self.failsafe.active_failsafe else "NORMAL"
                    print(f"Time: {int(time.time() - start_time)}s | "
                          f"Fix: {status['gps']['fix_type']} | "
                          f"Sats: {status['gps']['satellites']} | "
                          f"Status: {failsafe_status} | "
                          f"Action: {action.value}")
                
                time.sleep(0.2)
                
        except KeyboardInterrupt:
            print("Simulation interrupted")
        finally:
            # Stop monitoring
            self.failsafe.stop_monitoring()
            print("Simulation ended")


if __name__ == "__main__":
    """Run a sample simulation when the module is executed directly"""
    sim = SimulationExample()
    sim.run_simulation(duration=120)  # Run for 2 minutes


