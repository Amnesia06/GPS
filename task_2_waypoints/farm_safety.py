import math
import random

class SafetyModule:
    """
    A module to handle drift and enforce no-go rules in farm rover navigation.
    No-go now covers:
      1) Revisiting a waypoint (in the user-defined waypoint list).
      2) Exiting the farm boundary.
    Drift logic is unchanged.
    """
    def __init__(self, revisit_threshold=0.2):
        # Drift configuration
        self.turn_rate_per_cm = 20           # degrees per cm of turn capability
        self.distance_per_step = 0.2         # cm traveled per simulation step
        self.lookahead_steps = 5             # baseline lookahead for drift recovery
        self.drift_probability = 0.05        # probability of drift occurring per check

        # Geofence
        self.geofence = None                 # farm boundary polygon vertices

        # Waypoint tracking for revisit prevention
        self.waypoints = []                  # list of (x,y) waypoints
        self.visited_wp_indices = set()      # indices of waypoints already visited
        self.revisit_threshold = revisit_threshold

        # History logs
        self.violations_history = []         # track no-go incidents
        self.drift_history = []              # track drift incidents

    def set_geofence(self, vertices):
        """Set the farm boundary as a polygon"""
        self.geofence = vertices

    def set_waypoints(self, waypoints):
        """Provide the ordered list of navigation waypoints"""
        self.waypoints = waypoints
        self.visited_wp_indices.clear()

    def is_outside_geofence(self, pos):
        """Check if a position is outside the farm boundary polygon"""
        if not self.geofence:
            return False
        x, y = pos
        inside = False
        n = len(self.geofence)
        p1x, p1y = self.geofence[0]
        for i in range(1, n+1):
            p2x, p2y = self.geofence[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y) and x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
            p1x, p1y = p2x, p2y
        return not inside

    def check_safety(self, pos, heading, path):
        """
        Check if a planned move is safe.
        Returns:
          - status: 'safe', 'drift', or 'no-go'
          - data: info for recovery or violation handling
        """
        # Candidate next position
        next_pos = path[-1]

        # 1) No-go: revisiting an already visited waypoint
        for idx, wp in enumerate(self.waypoints):
            if math.hypot(next_pos[0]-wp[0], next_pos[1]-wp[1]) <= self.revisit_threshold:
                if idx in self.visited_wp_indices:
                    self.violations_history.append(('no-go-revisit', next_pos, heading))
                    return 'no-go', {
                        'violation_type': 'revisit',
                        'pos': next_pos,
                        'heading': heading
                    }
                else:
                    # mark this waypoint as visited now
                    self.visited_wp_indices.add(idx)
                    break

        # 2) No-go: exiting farm boundary
        if self.is_outside_geofence(next_pos):
            self.violations_history.append(('no-go-boundary', next_pos, heading))
            return 'no-go', {
                'violation_type': 'boundary',
                'pos': next_pos,
                'heading': heading
            }

        # 3) Potential drift
        if path and random.random() < self.drift_probability:
            closest_idx, _ = self.find_closest_point_on_path(pos, path)
            if closest_idx < len(path) - self.lookahead_steps:
                self.drift_history.append(('drift', pos.copy(), heading))
                drift_angle = 45 if random.choice([True, False]) else -45
                trigger_idx = closest_idx
                end_idx = min(trigger_idx + self.lookahead_steps, len(path)-1)
                dx = path[end_idx][0] - path[trigger_idx][0]
                dy = path[end_idx][1] - path[trigger_idx][1]
                mag = math.hypot(dx, dy)
                ux, uy = (dx/mag, dy/mag) if mag else (1.0, 0.0)
                rad = math.radians(drift_angle)
                c, s = math.cos(rad), math.sin(rad)
                rx = ux*c + uy*s
                ry = -ux*s + uy*c
                turn_dist = abs(drift_angle) / self.turn_rate_per_cm
                extra_skip = int(turn_dist / self.distance_per_step)
                recovery_idx = min(trigger_idx + self.lookahead_steps + extra_skip, len(path)-1)
                recovery_target = path[recovery_idx]
                return 'drift', {
                    'trigger_idx': trigger_idx,
                    'drift_angle': drift_angle,
                    'drift_vector': (rx, ry),
                    'recovery_idx': recovery_idx,
                    'recovery_target': recovery_target,
                    'path': path
                }

        # 4) Safe to proceed
        return 'safe', None

    def handle_drift(self, pos, heading, drift_data):
        """(Unchanged) Simulate drift and guide recovery."""
        # ... existing drift handler code ...
        raise NotImplementedError

    def handle_no_go_violation(self, pos, heading, violation_data):
        """(Unchanged) Back away from forbidden position."""
        # ... existing no-go handler code ...
        raise NotImplementedError

    def find_closest_point_on_path(self, pos, path):
        """Find the path index closest to pos."""
        min_dist, min_idx = float('inf'), 0
        for i, p in enumerate(path):
            d = math.hypot(p[0]-pos[0], p[1]-pos[1])
            if d < min_dist:
                min_dist, min_idx = d, i
        return min_idx, path[min_idx]

    def diff_h(self, c, t):
        return (t - c + 540) % 360 - 180
