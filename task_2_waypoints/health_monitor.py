import time
import random
from datetime import datetime

class GPSData:
    def __init__(self):
        self.fix_type = 'No Fix'
        self.satellites = 0
        self.hdop = float('inf')
        self.last_update_time = time.time()
        self.position = (0.0, 0.0)
        self.baseline_length = None
        self.solution_status = None
        self.temperature = None

    def update(self, fix_type, satellites, hdop, position, baseline_length=None, solution_status=None):
        self.fix_type = fix_type
        self.satellites = satellites
        self.hdop = hdop
        self.position = position
        self.last_update_time = time.time()
        if baseline_length is not None:
            self.baseline_length = baseline_length
        if solution_status is not None:
            self.solution_status = solution_status

class HealthMonitor:
    def __init__(self, gps_data, max_ttff=60, min_satellites=6, max_hdop=2.0, max_data_age=0.5, max_speed=5.0, rtk_max_baseline=20000, use_rtk=False):
        self.gps_data = gps_data
        self.max_ttff = max_ttff
        self.min_satellites = min_satellites
        self.max_hdop = max_hdop
        self.max_data_age = max_data_age
        self.max_speed = max_speed
        self.rtk_max_baseline = rtk_max_baseline
        self.use_rtk = use_rtk
        self.start_time = None
        self.first_fix_time = None
        self.previous_position = None
        self.previous_time = None

    def start(self):
        self.start_time = time.time()
        self.first_fix_time = None
        self.previous_position = None
        self.previous_time = None

    def check_health(self):
        current_time = time.time()
        alerts = []
        if self.gps_data.temperature is not None and (self.gps_data.temperature < -40 or self.gps_data.temperature > 85):
            alerts.append(f"Temperature out of range: {self.gps_data.temperature}°C")
        # Check TTFF
        if self.first_fix_time is None and self.gps_data.fix_type in ['3D Fix', 'DGPS', 'RTK Fixed']:
            self.first_fix_time = current_time
            ttff = self.first_fix_time - self.start_time
            if ttff > self.max_ttff:
                alerts.append(f"TTFF exceeded: {ttff:.2f}s > {self.max_ttff}s")

        # Check fix type
        if self.gps_data.fix_type not in ['3D Fix', 'DGPS', 'RTK Fixed']:
            alerts.append(f"Invalid fix type: {self.gps_data.fix_type}")

        # Check number of satellites
        if self.gps_data.satellites < self.min_satellites:
            alerts.append(f"Insufficient satellites: {self.gps_data.satellites} < {self.min_satellites}")

        # Check HDOP
        if self.gps_data.hdop > self.max_hdop:
            alerts.append(f"High HDOP: {self.gps_data.hdop} > {self.max_hdop}")

        # Check data age
        data_age = current_time - self.gps_data.last_update_time
        if data_age > self.max_data_age:
            alerts.append(f"Stale data: {data_age:.2f}s > {self.max_data_age}s")

        # Check position jump
        if self.previous_position is not None and self.previous_time is not None:
            dt = current_time - self.previous_time
            if dt > 0:
                distance = ((self.gps_data.position[0] - self.previous_position[0])**2 + (self.gps_data.position[1] - self.previous_position[1])**2)**0.5
                speed = distance / dt
                if speed > self.max_speed:
                    alerts.append(f"Position jump detected: speed {speed:.2f} m/s > {self.max_speed} m/s")

        # Update previous position and time
        self.previous_position = self.gps_data.position
        self.previous_time = current_time

        # RTK checks if applicable
        if self.use_rtk:
            if self.gps_data.baseline_length and self.gps_data.baseline_length > self.rtk_max_baseline:
                alerts.append(f"Baseline too long: {self.gps_data.baseline_length} m > {self.rtk_max_baseline} m")
            if self.gps_data.solution_status != 'FIXED':
                alerts.append(f"RTK solution not FIXED: {self.gps_data.solution_status}")

        return alerts