import math
import random
import numpy as np

class SafetyModule:
    """
    A module to handle drift and no-go zone safety checks for farm rover navigation.
    Combines logic from both drift recovery and geofence enforcement.
    """
    def __init__(self):
        # Drift configuration
        self.turn_rate_per_cm = 20           # degrees per cm of turn capability
        self.distance_per_step = 0.2         # cm traveled per simulation step
        self.lookahead_steps = 5             # baseline lookahead for drift recovery
        self.drift_probability = 0.05        # probability of drift occurring per check
        
        # No-go zone configuration
        self.no_go_zones = []                # list of no-go zones [(x0, y0, x1, y1), ...]
        self.geofence = None                 # farm boundary polygon vertices
        self.violations_history = []         # track violation incidents
        self.drift_history = []              # track drift incidents
        
    def set_geofence(self, vertices):
        """Set the farm boundary as a polygon"""
        self.geofence = vertices
        
    def add_no_go_zone(self, x0, y0, x1, y1):
        """Add a rectangular no-go zone"""
        self.no_go_zones.append((x0, y0, x1, y1))
    
    def clear_no_go_zones(self):
        """Clear all no-go zones"""
        self.no_go_zones = []
    
    def is_in_no_go_zone(self, pos):
        """Check if position is in any no-go zone"""
        for zone in self.no_go_zones:
            x0, y0, x1, y1 = zone
            if x0 <= pos[0] <= x1 and y0 <= pos[1] <= y1:
                return True
        return False
    
    def is_outside_geofence(self, pos):
        """Check if position is outside the geofence"""
        if not self.geofence:
            return False
            
        # Ray casting algorithm for point-in-polygon test
        x, y = pos
        inside = False
        n = len(self.geofence)
        
        p1x, p1y = self.geofence[0]
        for i in range(1, n + 1):
            p2x, p2y = self.geofence[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
            
        return not inside
    
    def check_safety(self, pos, heading, path):
        """
        Check if current position and heading are safe.
        Returns:
        - safety_status: 'safe', 'drift', 'no-go', or 'outside'
        - recovery_data: Data needed for recovery (if applicable)
        """
        # Check for no-go zone
        if self.is_in_no_go_zone(pos):
            self.violations_history.append(('no-go', pos.copy(), heading))
            return 'no-go', {'violation_type': 'no-go', 'pos': pos.copy(), 'heading': heading}
            
        # Check for outside geofence
        if self.is_outside_geofence(pos):
            self.violations_history.append(('outside', pos.copy(), heading))
            return 'outside', {'violation_type': 'outside', 'pos': pos.copy(), 'heading': heading}
            
        # Check for potential drift (random occurrence)
        if path and random.random() < self.drift_probability:
            # Find closest point on path for reference
            closest_idx, closest_point = self.find_closest_point_on_path(pos, path)
            
            # Only drift if we have enough path ahead
            if closest_idx < len(path) - self.lookahead_steps:
                self.drift_history.append(('drift', pos.copy(), heading))
                
                # Calculate drift parameters
                drift_angle = 45 if random.choice([True, False]) else -45
                lookahead = self.lookahead_steps
                
                # Calculate recovery parameters
                trigger_idx = closest_idx
                end_idx = min(trigger_idx + lookahead, len(path) - 1)
                
                # Drift direction calculation
                dx = path[end_idx][0] - path[trigger_idx][0]
                dy = path[end_idx][1] - path[trigger_idx][1]
                mag = math.hypot(dx, dy)
                ux, uy = (dx/mag, dy/mag) if mag != 0 else (1.0, 0.0)
                
                # Rotate drift direction
                rad = math.radians(drift_angle)
                c, s = math.cos(rad), math.sin(rad)
                rx = ux * c + uy * s
                ry = -ux * s + uy * c
                
                # Calculate recovery index
                turn_dist = abs(drift_angle) / self.turn_rate_per_cm
                extra_skip = int(turn_dist / self.distance_per_step)
                recovery_idx = min(trigger_idx + lookahead + extra_skip, len(path) - 1)
                recovery_target = path[recovery_idx]
                
                # Return drift data
                drift_data = {
                    'trigger_idx': trigger_idx,
                    'drift_angle': drift_angle,
                    'drift_vector': (rx, ry),
                    'recovery_idx': recovery_idx,
                    'recovery_target': recovery_target,
                    'path': path
                }
                return 'drift', drift_data
            
        # If all checks pass, we're safe
        return 'safe', None
    
    def handle_drift(self, pos, heading, drift_data):
        """
        Handle a drift event by simulating drift and computing recovery path.
        Returns:
        - updated_pos: Position after drift or during recovery
        - updated_heading: Heading after drift or during recovery
        - status: 'drifting', 'recovering', or 'recovered'
        - recovery_path: Path to follow for recovery
        """
        # Extract drift parameters
        drift_vector = drift_data['drift_vector']
        recovery_target = drift_data['recovery_target']
        drift_len = 1.0
        drift_steps = 20
        
        # Calculate how far to drift in each step
        drift_delta = (drift_vector[0] * drift_len/drift_steps, 
                      drift_vector[1] * drift_len/drift_steps)
        
        # Apply drift to position
        pos[0] += drift_delta[0]
        pos[1] += drift_delta[1]
        
        # Determine if we need to start recovery
        drift_steps_done = drift_data.get('drift_steps_done', 0) + 1
        drift_data['drift_steps_done'] = drift_steps_done
        
        if drift_steps_done >= drift_steps:
            # Calculate recovery heading
            desired = self.head(pos, recovery_target)
            dh = self.diff_h(heading, desired)
            
            # Apply heading correction
            turn_step = self.turn_rate_per_cm * self.distance_per_step
            if abs(dh) > 5:
                heading += turn_step if dh > 0 else -turn_step
                return pos, heading, 'recovering', drift_data
            else:
                # Move toward recovery point
                pos[0] += self.distance_per_step * math.cos(math.radians(heading))
                pos[1] += self.distance_per_step * math.sin(math.radians(heading))
                
                # Check if we've recovered
                d = self.dist(pos, recovery_target)
                if d < 0.5:
                    return recovery_target, heading, 'recovered', None
                return pos, heading, 'recovering', drift_data
        else:
            return pos, heading, 'drifting', drift_data
    
    def handle_no_go_violation(self, pos, heading, violation_data):
        """
        Handle a no-go zone violation by backing away from the zone.
        Returns:
        - updated_pos: Position after backing away
        - updated_heading: Heading after backing away
        - status: 'backing', 'redirecting', or 'recovered'
        """
        # Back up in the opposite direction of current heading
        backup_heading = (heading + 180) % 360
        
        # Back up by a small amount
        backup_dist = self.distance_per_step * 2  # Back up by twice the normal step size
        new_pos = [
            pos[0] + backup_dist * math.cos(math.radians(backup_heading)),
            pos[1] + backup_dist * math.sin(math.radians(backup_heading))
        ]
        
        # Check if we're still in a no-go zone
        if self.is_in_no_go_zone(new_pos) or self.is_outside_geofence(new_pos):
            # Try a different direction if backing up doesn't work
            for angle_offset in [45, -45, 90, -90]:
                try_heading = (backup_heading + angle_offset) % 360
                try_pos = [
                    pos[0] + backup_dist * math.cos(math.radians(try_heading)),
                    pos[1] + backup_dist * math.sin(math.radians(try_heading))
                ]
                if not self.is_in_no_go_zone(try_pos) and not self.is_outside_geofence(try_pos):
                    return try_pos, try_heading, 'redirecting'
            
            # If all attempts fail, just back up a tiny bit and hope for the best
            return [
                pos[0] + 0.1 * math.cos(math.radians(backup_heading)),
                pos[1] + 0.1 * math.sin(math.radians(backup_heading))
            ], backup_heading, 'backing'
        else:
            # Successfully backed out
            return new_pos, backup_heading, 'recovered'
    
    def find_closest_point_on_path(self, pos, path):
        """Find the closest point on a path to the given position"""
        min_dist = float('inf')
        min_idx = 0
        min_point = None
        
        for i, point in enumerate(path):
            d = self.dist(pos, point)
            if d < min_dist:
                min_dist = d
                min_idx = i
                min_point = point
                
        return min_idx, min_point
    
    # Utility functions
    def dist(self, a, b):
        """Calculate Euclidean distance between points a and b"""
        return math.hypot(b[0] - a[0], b[1] - a[1])
    
    def head(self, a, b):
        """Calculate heading from point a to point b in degrees"""
        return math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])) % 360
    
    def diff_h(self, c, t):
        """Calculate the minimal angle difference between headings c and t"""
        return (t - c + 540) % 360 - 180
    

# At the end of your file, add:
safety = SafetyModule()