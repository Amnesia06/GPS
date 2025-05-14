from sleep_mode import FailsafeModule
import time
import threading
import math
import random
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.path import Path
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import matplotlib.colors as mcolors
from enum import Enum

class GPSFailsafeReason(Enum):
    GPS_DATA_LOSS = "GPS data loss"
    GPS_STALE_DATA = "GPS stale data"
    INTERNET_CONNECTION_LOST = "Internet connection lost"
    INTERNET_CONNECTION_SLOW = "Internet connection slow"
    OVER_TEMPERATURE = "Over temperature"
    MODULE_COMMUNICATION_FAILURE = "Module communication failure"
    SIGNAL_INSTABILITY = "Signal instability"
    SIGNIFICANT_DRIFT = "Significant navigation drift"
    PERSISTENT_DRIFT = "Persistent navigation drift"
    CUSTOM = "Custom reason"
    GPS_POSITION_JUMP = "GPS position jump detected"
    GPS_HEADING_INCONSISTENCY = "GPS heading inconsistency"
    GPS_ALTITUDE_ANOMALY = "GPS altitude anomaly"
    GPS_VELOCITY_CONSTRAINT = "GPS velocity constraint violation"
    GPS_HIGH_DOP = "GPS high dilution of precision"
    GPS_SIGNAL_MULTIPATH = "GPS signal multipath detected"
    GPS_CONSTELLATION_WEAK = "GPS constellation geometry weak"
    RTK_FIX_LOST = "RTK fix lost"
    RTK_CORRECTION_TIMEOUT = "RTK correction data timeout"
    RTK_BASE_DISCONNECTION = "RTK base station disconnection"
    RTK_FLOAT_DEGRADATION = "RTK float solution degradation"

class DriftSeverity(Enum):
    MINOR = "Minor drift"
    MODERATE = "Moderate drift"
    SEVERE = "Severe drift"
    CRITICAL = "Critical drift"

class DriftAction(Enum):
    MONITOR = "Continue monitoring"
    REALIGN = "Perform realignment"
    SLOW_DOWN = "Reduce speed"
    PAUSE = "Pause movement"
    FAILSAFE = "Enter failsafe mode"

class SimulatedFailsafeModule:
    def __init__(self):
        self.in_failsafe_mode = False
        self.failsafe_reason = None
        self.recovery_attempts = 0
        self.recovery_in_progress = False
        self.last_gps_update = time.time()
        self.drift_events = []
        self.consecutive_drift_events = 0
        self.current_drift_action = DriftAction.MONITOR
        self.monitoring_active = False
        self.failsafe_callbacks = []
        self.recovery_callbacks = []
        self.drift_action_callbacks = []
        
        # Additional failsafe features
        self.gps_signal_quality = 1.0
        self.temperature = 25.0  # Celsius
        self.internet_connection_status = True
        self.rtk_fix_quality = "RTK_FIXED"
        self.last_rtk_correction = time.time()
        self.last_status_check = time.time()
        self.module_communication_status = True

    def register_failsafe_callback(self, callback):
        self.failsafe_callbacks.append(callback)

    def register_recovery_callback(self, callback):
        self.recovery_callbacks.append(callback)

    def register_drift_action_callback(self, callback):
        self.drift_action_callbacks.append(callback)

    def _trigger_failsafe(self, reason):
        if self.in_failsafe_mode:
            return
        self.in_failsafe_mode = True
        self.failsafe_reason = reason
        print(f"⚠️ FAILSAFE MODE ACTIVATED: {reason.value}")
        for callback in self.failsafe_callbacks:
            callback(reason)

    def _clear_failsafe(self):
        if not self.in_failsafe_mode:
            return
        old_reason = self.failsafe_reason
        self.in_failsafe_mode = False
        self.failsafe_reason = None
        self.recovery_attempts = 0
        print(f"✅ Recovered from failsafe mode: {old_reason.value}")

    def attempt_recovery(self):
        if not self.in_failsafe_mode:
            return False
        self.recovery_attempts += 1
        print(f"🔄 Attempting recovery #{self.recovery_attempts} for {self.failsafe_reason.value}")
        success = False
        for callback in self.recovery_callbacks:
            result = callback(self.failsafe_reason)
            success = success or result
        if success:
            self._clear_failsafe()
            return True
        return False

    def trigger_custom_failsafe(self, reason):
        self._trigger_failsafe(reason)

    def report_drift(self, actual_position, expected_position, drift_distance=None):
        if drift_distance is None:
            try:
                drift_distance = float(math.sqrt(
                    (actual_position[0] - expected_position[0])**2 +
                    (actual_position[1] - expected_position[1])**2
                ))
            except Exception as e:
                print(f"Error calculating drift distance: {e}")
                print(f"actual_position: {actual_position}, expected_position: {expected_position}")
                drift_distance = 0.0
                
        severity = None
        if drift_distance >= 2.0:
            severity = DriftSeverity.CRITICAL
        elif drift_distance >= 1.0:
            severity = DriftSeverity.SEVERE
        elif drift_distance >= 0.5:
            severity = DriftSeverity.MODERATE
        elif drift_distance >= 0.2:
            severity = DriftSeverity.MINOR
        if severity is not None:
            self.drift_events.append({
                'timestamp': time.time(),
                'distance': drift_distance,
                'severity': severity,
                'actual': actual_position.copy() if isinstance(actual_position, np.ndarray) else actual_position,
                'expected': expected_position.copy() if isinstance(expected_position, np.ndarray) else expected_position
            })
            self.consecutive_drift_events += 1
            action = DriftAction.MONITOR
            if severity == DriftSeverity.CRITICAL:
                action = DriftAction.FAILSAFE
            elif severity == DriftSeverity.SEVERE:
                if self.consecutive_drift_events >= 5:
                    action = DriftAction.FAILSAFE
                else:
                    action = DriftAction.PAUSE
            elif severity == DriftSeverity.MODERATE:
                if self.consecutive_drift_events >= 5:
                    action = DriftAction.PAUSE
                else:
                    action = DriftAction.SLOW_DOWN
            
            # Set current drift action
            self.current_drift_action = action
            
            for callback in self.drift_action_callbacks:
                callback(action, drift_distance)
            if action == DriftAction.FAILSAFE:
                self._trigger_failsafe(GPSFailsafeReason.SIGNIFICANT_DRIFT)
            return drift_distance, severity
        else:
            self.consecutive_drift_events = 0
            return drift_distance, None

    def get_status(self):
        return {
            "in_failsafe_mode": self.in_failsafe_mode,
            "failsafe_reason": self.failsafe_reason.value if self.failsafe_reason else None,
            "recovery_attempts": self.recovery_attempts,
            "drift_events": len(self.drift_events),
            "consecutive_drift_events": self.consecutive_drift_events,
            "current_drift_action": self.current_drift_action.value,
            "gps_signal_quality": self.gps_signal_quality,
            "temperature": self.temperature,
            "internet_connection": self.internet_connection_status,
            "rtk_fix_quality": self.rtk_fix_quality,
            "module_communication": self.module_communication_status
        }
        
    def check_failsafe_conditions(self):
        """Regularly check various failsafe conditions"""
        current_time = time.time()
        
        # Check GPS signal quality
        if self.gps_signal_quality < 0.3 and not self.in_failsafe_mode:
            self._trigger_failsafe(GPSFailsafeReason.SIGNAL_INSTABILITY)
            return
            
        # Check temperature
        if self.temperature > 50.0 and not self.in_failsafe_mode:
            self._trigger_failsafe(GPSFailsafeReason.OVER_TEMPERATURE)
            return
            
        # Check internet connection
        if not self.internet_connection_status and not self.in_failsafe_mode:
            self._trigger_failsafe(GPSFailsafeReason.INTERNET_CONNECTION_LOST)
            return
            
        # Check RTK correction data timeout
        if current_time - self.last_rtk_correction > 10.0 and not self.in_failsafe_mode:
            self._trigger_failsafe(GPSFailsafeReason.RTK_CORRECTION_TIMEOUT)
            return
            
        # Check module communication
        if not self.module_communication_status and not self.in_failsafe_mode:
            self._trigger_failsafe(GPSFailsafeReason.MODULE_COMMUNICATION_FAILURE)
            return

class RoverSimulation:
    def __init__(self, field_size=(100, 100), row_spacing=20):  # Increased row spacing to reduce zigzags
        self.field_size = field_size
        self.row_spacing = row_spacing
        self.failsafe_module = SimulatedFailsafeModule()
        self.failsafe_module.register_failsafe_callback(self.on_failsafe_triggered)
        self.failsafe_module.register_recovery_callback(self.on_recovery_attempt)
        self.failsafe_module.register_drift_action_callback(self.on_drift_action)
        self.waypoints = self.generate_zigzag_waypoints()
        self.rover_position = self.waypoints[0].copy()
        self.target_waypoint_idx = 1
        self.rover_speed = 1.0
        self.rover_heading = 0.0
        self.rover_paused = False
        self.simulation_running = False
        self.simulation_thread = None
        self.update_interval = 0.1
        self.elapsed_time = 0
        self.position_history = [self.rover_position.copy()]
        self.status_history = []
        self.drift_history = []
        self.event_markers = []
        self.fig = None
        self.ax = None
        self.rover_patch = None
        self.history_line = None
        self.status_text = None
        self.waypoint_patches = []
        self.event_patches = []
        self.failure_scenarios = {
            'gps_jump': {'countdown': 100, 'triggered': False},
            'gps_drift': {'countdown': 200, 'triggered': False},
            'rtk_loss': {'countdown': 300, 'triggered': False},
            'signal_loss': {'countdown': 400, 'triggered': False},
            'module_comm_failure': {'countdown': 500, 'triggered': False},
            'over_temperature': {'countdown': 600, 'triggered': False},  # Added new scenario
            'internet_loss': {'countdown': 700, 'triggered': False},     # Added new scenario
        }

    def generate_zigzag_waypoints(self):
        waypoints = []
        x_max, y_max = self.field_size
        x, y = 0, 0
        waypoints.append(np.array([x, y], dtype=np.float64))
        # Reduced row count by using larger row spacing
        row_count = int(y_max / self.row_spacing) 
        for i in range(row_count):
            x = x_max
            waypoints.append(np.array([x, y], dtype=np.float64))
            y += self.row_spacing
            waypoints.append(np.array([x, y], dtype=np.float64))
            x = 0
            waypoints.append(np.array([x, y], dtype=np.float64))
            if i < row_count - 1:
                y += self.row_spacing
                waypoints.append(np.array([x, y], dtype=np.float64))
        return waypoints

    def on_failsafe_triggered(self, reason):
        print(f"Rover paused due to failsafe: {reason.value}")
        self.rover_paused = True
        self.event_markers.append((self.rover_position.copy(), "failsafe", reason.value))
        self.status_history.append({
            'time': self.elapsed_time,
            'status': f"FAILSAFE: {reason.value}"
        })

    def on_recovery_attempt(self, reason):
        print(f"Attempting to recover from: {reason.value}")
        self.event_markers.append((self.rover_position.copy(), "recovery", f"Recovery attempt #{self.failsafe_module.recovery_attempts}"))
        success = random.random() > 0.5
        if success:
            print(f"Recovery successful!")
            self.rover_paused = False
            self.status_history.append({
                'time': self.elapsed_time,
                'status': f"RECOVERED from {reason.value}"
            })
        else:
            print(f"Recovery failed, still in failsafe mode.")
            self.status_history.append({
                'time': self.elapsed_time,
                'status': f"RECOVERY FAILED for {reason.value}"
            })
        return success

    def on_drift_action(self, action, drift_distance):
        print(f"Drift action: {action.value} (distance: {drift_distance:.2f}m)")
        if action == DriftAction.SLOW_DOWN:
            self.rover_speed = 0.5
        elif action == DriftAction.PAUSE:
            self.rover_paused = True
        elif action == DriftAction.REALIGN:
            try:
                # Fixed realignment logic to be more gentle
                target = self.waypoints[self.target_waypoint_idx - 1]
                direction = target - self.rover_position
                distance = np.linalg.norm(direction)
                if distance > 0:
                    # Move only 20% of the way towards the correct path instead of 50%
                    self.rover_position += direction * 0.2
            except Exception as e:
                print(f"Error during realignment: {e}")
        self.event_markers.append((self.rover_position.copy(), "drift", f"{action.value} ({drift_distance:.2f}m)"))
        self.drift_history.append({
            'time': self.elapsed_time,
            'distance': drift_distance,
            'action': action.value
        })

    def trigger_failure_scenario(self, scenario_type):
        try:
            if scenario_type == 'gps_jump':
                print("Simulating GPS position jump...")
                jump_distance = 10.0
                jump_direction = np.array([random.uniform(-1, 1), random.uniform(-1, 1)], dtype=np.float64)
                norm = np.linalg.norm(jump_direction)
                if norm > 0:  # Avoid division by zero
                    jump_direction = jump_direction / norm * jump_distance
                    self.rover_position += jump_direction
                self.failsafe_module.trigger_custom_failsafe(GPSFailsafeReason.GPS_POSITION_JUMP)
            elif scenario_type == 'gps_drift':
                print("Simulating GPS drift...")
                for _ in range(5):
                    drift = np.array([random.uniform(-0.5, 0.5), random.uniform(-0.5, 0.5)], dtype=np.float64)
                    expected_position = self.waypoints[self.target_waypoint_idx - 1]
                    self.rover_position += drift
                    self.failsafe_module.report_drift(self.rover_position, expected_position)
            elif scenario_type == 'rtk_loss':
                print("Simulating RTK fix loss...")
                self.failsafe_module.rtk_fix_quality = "RTK_FLOAT"
                self.failsafe_module.trigger_custom_failsafe(GPSFailsafeReason.RTK_FIX_LOST)
            elif scenario_type == 'signal_loss':
                print("Simulating GPS signal instability...")
                self.failsafe_module.gps_signal_quality = 0.2
                self.failsafe_module.trigger_custom_failsafe(GPSFailsafeReason.SIGNAL_INSTABILITY)
            elif scenario_type == 'module_comm_failure':
                print("Simulating module communication failure...")
                self.failsafe_module.module_communication_status = False
                self.failsafe_module.trigger_custom_failsafe(GPSFailsafeReason.MODULE_COMMUNICATION_FAILURE)
            elif scenario_type == 'over_temperature':
                print("Simulating over temperature condition...")
                self.failsafe_module.temperature = 55.0
                self.failsafe_module.trigger_custom_failsafe(GPSFailsafeReason.OVER_TEMPERATURE)
            elif scenario_type == 'internet_loss':
                print("Simulating internet connection loss...")
                self.failsafe_module.internet_connection_status = False
                self.failsafe_module.trigger_custom_failsafe(GPSFailsafeReason.INTERNET_CONNECTION_LOST)
        except Exception as e:
            print(f"Error in failure scenario {scenario_type}: {e}")

    def update_rover_position(self):
        if self.rover_paused or self.failsafe_module.in_failsafe_mode:
            return
        if self.target_waypoint_idx < len(self.waypoints):
            try:
                target = self.waypoints[self.target_waypoint_idx]
                direction = target - self.rover_position
                distance = float(np.linalg.norm(direction))
                if distance < 0.1:
                    self.target_waypoint_idx += 1
                    print(f"Reached waypoint {self.target_waypoint_idx-1}, moving to waypoint {self.target_waypoint_idx}")
                    if self.target_waypoint_idx < len(self.waypoints):
                        target = self.waypoints[self.target_waypoint_idx]
                        direction = target - self.rover_position
                        distance = float(np.linalg.norm(direction))
                    else:
                        print("Reached final waypoint!")
                        return
                if distance > 0:
                    direction = direction / distance
                step_distance = min(self.rover_speed * self.update_interval, distance)
                movement = direction * step_distance
                self.rover_position += movement
                if np.linalg.norm(direction) > 0:
                    self.rover_heading = math.degrees(math.atan2(direction[1], direction[0]))
                self.position_history.append(self.rover_position.copy())
                
                # Occasionally simulate drift
                if random.random() < 0.05:
                    drift = np.array([random.uniform(-0.2, 0.2), random.uniform(-0.2, 0.2)], dtype=np.float64)
                    expected_position = self.rover_position - drift
                    self.failsafe_module.report_drift(self.rover_position, expected_position)
            except Exception as e:
                print(f"Error updating rover position: {e}")
                print(f"Current position: {self.rover_position}, Target: {self.waypoints[self.target_waypoint_idx] if self.target_waypoint_idx < len(self.waypoints) else 'None'}")

    def check_failure_scenarios(self):
        for scenario, data in self.failure_scenarios.items():
            if not data['triggered'] and data['countdown'] <= 0:
                self.trigger_failure_scenario(scenario)
                data['triggered'] = True
            elif not data['triggered']:
                data['countdown'] -= 1
        
        # Also periodically check the failsafe module's internal conditions
        if random.random() < 0.02:  # 2% chance each step
            self.failsafe_module.check_failsafe_conditions()

    def recovery_handler(self):
        if self.failsafe_module.in_failsafe_mode and random.random() < 0.1:
            self.failsafe_module.attempt_recovery()

    def run_simulation_step(self):
        if not self.simulation_running:
            return
        self.elapsed_time += self.update_interval
        self.update_rover_position()
        self.check_failure_scenarios()
        self.recovery_handler()
        
        # Simulate varying conditions for failsafe triggers
        if random.random() < 0.01:  # Occasionally change system status
            self.failsafe_module.gps_signal_quality = max(0.1, min(1.0, self.failsafe_module.gps_signal_quality + random.uniform(-0.1, 0.1)))
            self.failsafe_module.temperature = max(20.0, min(60.0, self.failsafe_module.temperature + random.uniform(-0.5, 0.5)))
            if random.random() < 0.005:  # Very occasionally flip internet status
                self.failsafe_module.internet_connection_status = not self.failsafe_module.internet_connection_status
            if random.random() < 0.002:  # Very occasionally flip module comm status
                self.failsafe_module.module_communication_status = not self.failsafe_module.module_communication_status

    def setup_visualization(self):
        try:
            self.fig, self.ax = plt.subplots(figsize=(10, 8))
            self.fig.suptitle('Rover Failsafe Simulation', fontsize=16)
            self.ax.set_xlim(-10, self.field_size[0] + 10)
            self.ax.set_ylim(-10, self.field_size[1] + 10)
            self.ax.set_xlabel('X (meters)')
            self.ax.set_ylabel('Y (meters)')
            self.ax.grid(True)
            waypoints_x = [float(wp[0]) for wp in self.waypoints]
            waypoints_y = [float(wp[1]) for wp in self.waypoints]
            self.waypoint_line, = self.ax.plot(waypoints_x, waypoints_y, 'b--', alpha=0.5, label='Planned Path')
            for i, wp in enumerate(self.waypoints):
                waypoint = Circle((float(wp[0]), float(wp[1])), 1.0, fill=True, alpha=0.6, fc='blue', ec='black')
                self.waypoint_patches.append(waypoint)
                self.ax.add_patch(waypoint)
                self.ax.annotate(f"{i}", (float(wp[0]), float(wp[1])), color='white', ha='center', va='center')
            self.rover_patch = Circle((float(self.rover_position[0]), float(self.rover_position[1])), 2.0, fill=True, fc='green', ec='black')
            self.ax.add_patch(self.rover_patch)
            self.history_line, = self.ax.plot([float(self.rover_position[0])], [float(self.rover_position[1])], 'g-', alpha=0.7, label='Actual Path')
            self.status_text = self.ax.text(0.02, 0.98, "Status: Moving", transform=self.ax.transAxes,
                                    verticalalignment='top', bbox=dict(boxstyle='round', facecolor='white', alpha=0.7))
            self.ax.legend()
            plt.tight_layout()
        except Exception as e:
            print(f"Error setting up visualization: {e}")

    def update_visualization(self, frame):
        try:
            self.run_simulation_step()
            if self.rover_patch:
                self.rover_patch.center = (float(self.rover_position[0]), float(self.rover_position[1]))
            if self.history_line and len(self.position_history) > 0:
                xs = [float(pos[0]) for pos in self.position_history]
                ys = [float(pos[1]) for pos in self.position_history]
                self.history_line.set_data(xs, ys)
            status = "FAILSAFE: " + self.failsafe_module.failsafe_reason.value if self.failsafe_module.in_failsafe_mode else \
                    "PAUSED" if self.rover_paused else "Moving"
            # Enhanced status display to show more system info
            self.status_text.set_text(
                f"Status: {status}\n"
                f"Waypoint: {self.target_waypoint_idx}/{len(self.waypoints)}\n"
                f"Elapsed: {self.elapsed_time:.1f}s\n"
                f"GPS Signal: {self.failsafe_module.gps_signal_quality:.1f}\n"
                f"Temp: {self.failsafe_module.temperature:.1f}°C\n"
                f"Internet: {'Online' if self.failsafe_module.internet_connection_status else 'Offline'}"
            )
            for marker in self.event_markers:
                if marker not in [getattr(patch, '_marker_data', None) for patch in self.event_patches]:
                    pos, event_type, description = marker
                    color = 'red' if event_type == 'failsafe' else 'orange' if event_type == 'drift' else 'green'
                    marker_patch = Circle((float(pos[0]), float(pos[1])), 1.5, fill=True, alpha=0.7, fc=color, ec='black')
                    marker_patch._marker_data = marker
                    self.event_patches.append(marker_patch)
                    self.ax.add_patch(marker_patch)
                    self.ax.annotate(event_type[0].upper(), (float(pos[0]), float(pos[1])), color='white', ha='center', va='center', weight='bold')
            return [self.rover_patch, self.history_line, self.status_text] + self.event_patches
        except Exception as e:
            print(f"Error updating visualization: {e}")
            return []

    def run_simulation(self, duration=None):
        self.simulation_running = True
        try:
            self.setup_visualization()
            frames = int(duration / self.update_interval) if duration else 1000
            animation = FuncAnimation(self.fig, self.update_visualization, frames=frames,
                                    interval=self.update_interval*1000, blit=False)
            plt.show()
        except Exception as e:
            print(f"Error running simulation: {e}")
        finally:
            self.simulation_running = False

def plot_failure_recovery_timeline(simulation):
    if not simulation.status_history:
        print("No status events to plot")
        return
    try:
        fig, ax = plt.subplots(figsize=(12, 6))
        fig.suptitle('Rover Failsafe Timeline', fontsize=16)
        times = [event['time'] for event in simulation.status_history]
        statuses = [event['status'] for event in simulation.status_history]
        colors = []
        for status in statuses:
            if 'FAILSAFE' in status:
                colors.append('red')
            elif 'RECOVERED' in status:
                colors.append('green')
            elif 'RECOVERY FAILED' in status:
                colors.append('orange')
            else:
                colors.append('blue')
        ax.scatter(times, [1] * len(times), c=colors, s=100, zorder=2)
        for i, (time, status) in enumerate(zip(times, statuses)):
            ax.annotate(status, (time, 1.05), ha='center', rotation=45, fontsize=8)
        ax.plot(times, [1] * len(times), 'k-', alpha=0.3, zorder=1)
        ax.set_yticks([])
        ax.set_xlabel('Simulation Time (s)')
        ax.set_title('Failsafe Events Timeline')
        ax.grid(True, axis='x')
        plt.tight_layout()
        plt.show()
    except Exception as e:
        print(f"Error plotting timeline: {e}")

def plot_drift_analysis(simulation):
    if not simulation.drift_history:
        print("No drift events to plot")
        return
    try:
        fig, ax = plt.subplots(figsize=(10, 6))
        fig.suptitle('Rover Drift Analysis', fontsize=16)
        times = [event['time'] for event in simulation.drift_history]
        distances = [event['distance'] for event in simulation.drift_history]
        actions = [event['action'] for event in simulation.drift_history]
        action_colors = {
            'Continue monitoring': 'green',
            'Reduce speed': 'yellow',
            'Perform realignment': 'orange',
            'Pause movement': 'red',
            'Enter failsafe mode': 'purple'
        }
        colors = [action_colors.get(action, 'blue') for action in actions]
        ax.scatter(times, distances, c=colors, s=50, alpha=0.7, zorder=2)
        ax.plot(times, distances, 'k-', alpha=0.3, zorder=1)
        ax.axhline(y=0.2, color='green', linestyle='--', alpha=0.7, label='Minor Drift')
        ax.axhline(y=0.5, color='yellow', linestyle='--', alpha=0.7, label='Moderate Drift')
        ax.axhline(y=1.0, color='red', linestyle='--', alpha=0.7, label='Severe Drift')
        ax.axhline(y=2.0, color='purple', linestyle='--', alpha=0.7, label='Critical Drift')
        for i, (time, distance, action) in enumerate(zip(times, distances, actions)):
            if action != 'Continue monitoring':
                ax.annotate(action, (time, distance), xytext=(0, 10),
                        textcoords='offset points', ha='center', fontsize=8)
        ax.set_xlabel('Simulation Time (s)')
        ax.set_ylabel('Drift Distance (m)')
        ax.set_title('Drift Events and Actions')
        ax.grid(True)
        ax.legend()
        plt.tight_layout()
        plt.show()
    except Exception as e:
        print(f"Error plotting drift analysis: {e}")

def plot_system_status_monitoring(simulation):
    """Plot the system status parameters over time"""
    try:
        # Extract time series data from the simulation
        times = []
        gps_quality = []
        temperature = []
        internet_status = []
        rtk_status = []
        module_status = []
        
        # Sample data every few simulation steps
        step_count = 0
        for position in simulation.position_history:
            if step_count % 10 == 0:  # Sample every 10 steps
                times.append(step_count * simulation.update_interval)
                # These values would normally come from recorded data
                # For demo purposes, generate some simulated values
                gps_val = max(0.1, min(1.0, 0.8 + random.uniform(-0.3, 0.2)))
                temp = max(20, min(60, 30 + random.uniform(-5, 15)))
                internet = random.random() > 0.1  # 90% chance of being online
                rtk = random.random() > 0.2  # 80% chance of RTK fix
                module = random.random() > 0.05  # 95% chance of module working
                
                gps_quality.append(gps_val)
                temperature.append(temp)
                internet_status.append(1 if internet else 0)
                rtk_status.append(1 if rtk else 0)
                module_status.append(1 if module else 0)
            step_count += 1
            
        # Create the plot
        fig, axes = plt.subplots(5, 1, figsize=(12, 10), sharex=True)
        fig.suptitle('System Status Monitoring', fontsize=16)
        
        # GPS Signal Quality
        axes[0].plot(times, gps_quality, 'b-')
        axes[0].set_ylabel('GPS Quality')
        axes[0].set_ylim(0, 1.1)
        axes[0].axhline(y=0.3, color='red', linestyle='--', alpha=0.7)
        axes[0].grid(True)
        
        # Temperature
        axes[1].plot(times, temperature, 'r-')
        axes[1].set_ylabel('Temp (°C)')
        axes[1].axhline(y=50, color='red', linestyle='--', alpha=0.7)
        axes[1].grid(True)
        
        # Internet Status
        axes[2].step(times, internet_status, 'g-', where='post')
        axes[2].set_ylabel('Internet')
        axes[2].set_yticks([0, 1])
        axes[2].set_yticklabels(['Offline', 'Online'])
        axes[2].grid(True)
        
        # RTK Status
        axes[3].step(times, rtk_status, 'c-', where='post')
        axes[3].set_ylabel('RTK Fix')
        axes[3].set_yticks([0, 1])
        axes[3].set_yticklabels(['Lost', 'Fixed'])
        axes[3].grid(True)
        
        # Module Status
        axes[4].step(times, module_status, 'm-', where='post')
        axes[4].set_ylabel('Module Comm')
        axes[4].set_yticks([0, 1])
        axes[4].set_yticklabels(['Failed', 'OK'])
        axes[4].grid(True)
        
        # Add event markers for failsafe activations
        for event in simulation.event_markers:
            pos, event_type, description = event
            if event_type == "failsafe":
                event_time = simulation.position_history.index(pos.tolist() if isinstance(pos, np.ndarray) else pos) * simulation.update_interval
                for ax in axes:
                    ax.axvline(x=event_time, color='red', linestyle='-', alpha=0.3)
        
        axes[4].set_xlabel('Simulation Time (s)')
        plt.tight_layout()
        plt.show()
    except Exception as e:
        print(f"Error plotting system status: {e}")

def main():
    try:
        # Set NumPy error handling to raise exceptions
        np.seterr(all='raise')
        
        sim = RoverSimulation(field_size=(100, 80), row_spacing=10)
        sim.run_simulation(duration=600)
        plot_failure_recovery_timeline(sim)
        plot_drift_analysis(sim)
    except Exception as e:
        print(f"Error in main function: {e}")

if __name__ == "__main__":
    main()