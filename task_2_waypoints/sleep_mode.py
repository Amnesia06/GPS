import time
import threading
import datetime
import logging
from enum import Enum
import math

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler("rover_failsafe.log", encoding='utf-8'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger("RoverFailsafe")

class FailsafeReason(Enum):
    """Enumeration of possible reasons for entering failsafe mode"""
    GPS_DATA_LOSS = "GPS data loss"
    GPS_STALE_DATA = "GPS stale data"
    INTERNET_CONNECTION_LOST = "Internet connection lost"
    INTERNET_CONNECTION_SLOW = "Internet connection slow"
    OVER_TEMPERATURE = "Over temperature"
    MODULE_COMMUNICATION_FAILURE = "Module communication failure"
    SIGNAL_INSTABILITY = "Signal instability"
    CUSTOM = "Custom reason"

class FailsafeModule:
    """
    Module to handle failsafe and sleep mode triggering for rover operations.
    This module is separate from the farm_safety module and handles system-level
    safety rather than navigation-specific safety.
    """
    
    def __init__(self):
        # Status flags
        self.in_failsafe_mode = False
        self.failsafe_reason = None
        self.recovery_attempts = 0
        self.recovery_in_progress = False
        
        # Timestamp tracking
        self.last_gps_update = None
        self.last_internet_check = None
        self.last_module_comm = None
        self.last_recovery_attempt = None
        
        # Thresholds and settings
        self.gps_stale_threshold = 5.0  # seconds
        self.internet_timeout = 10.0  # seconds
        self.recovery_interval = 60.0  # seconds between recovery attempts
        self.max_recovery_attempts = 5  # maximum number of recovery attempts
        self.temperature_threshold = 60.0  # degrees Celsius
        
        # Signal tracking
        self.signal_loss_events = []
        self.signal_loss_window = 60.0  # 1 minute window for signal loss events
        self.signal_loss_threshold = 5  # Number of signal losses to trigger failsafe
        
        # Initialize monitoring threads
        self.monitoring_active = False
        self.monitor_thread = None
        
        # Callback for failsafe activation
        self.failsafe_callback = None
        self.recovery_callback = None
        
        logger.info("Failsafe module initialized")
    
    def set_callbacks(self, failsafe_callback, recovery_callback=None):
        """
        Set callbacks for failsafe activation and recovery
        
        Args:
            failsafe_callback: Function to call when failsafe is triggered
                               Should accept FailsafeReason as an argument
            recovery_callback: Function to call when attempting recovery
                               Should accept FailsafeReason as an argument
        """
        self.failsafe_callback = failsafe_callback
        self.recovery_callback = recovery_callback
        logger.info("Failsafe callbacks set")
    
    def configure(self, gps_stale_threshold=None, internet_timeout=None, 
                  recovery_interval=None, max_recovery_attempts=None,
                  temperature_threshold=None, signal_loss_threshold=None):
        """Configure failsafe thresholds and settings"""
        if gps_stale_threshold is not None:
            self.gps_stale_threshold = gps_stale_threshold
        if internet_timeout is not None:
            self.internet_timeout = internet_timeout
        if recovery_interval is not None:
            self.recovery_interval = recovery_interval
        if max_recovery_attempts is not None:
            self.max_recovery_attempts = max_recovery_attempts
        if temperature_threshold is not None:
            self.temperature_threshold = temperature_threshold
        if signal_loss_threshold is not None:
            self.signal_loss_threshold = signal_loss_threshold
        
        logger.info(f"Failsafe configured: GPS stale threshold={self.gps_stale_threshold}s, "
                   f"Internet timeout={self.internet_timeout}s, "
                   f"Recovery interval={self.recovery_interval}s, "
                   f"Max recovery attempts={self.max_recovery_attempts}, "
                   f"Temperature threshold={self.temperature_threshold}°C, "
                   f"Signal loss threshold={self.signal_loss_threshold} events")
    
    def start_monitoring(self):
        """Start the failsafe monitoring thread"""
        if self.monitoring_active:
            logger.warning("Monitoring already active")
            return
        
        self.monitoring_active = True
        self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.monitor_thread.start()
        logger.info("Failsafe monitoring started")
    
    def stop_monitoring(self):
        """Stop the failsafe monitoring thread"""
        if not self.monitoring_active:
            logger.warning("Monitoring not active")
            return
        
        self.monitoring_active = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=1.0)
        logger.info("Failsafe monitoring stopped")
    
    def _monitor_loop(self):
        """Main monitoring loop that checks for failsafe conditions"""
        while self.monitoring_active:
            try:
                # Check various failsafe conditions
                self._check_gps_status()
                self._check_internet_connection()
                self._check_module_communication()
                self._check_temperature()
                self._check_signal_stability()
                
                # Check if recovery is needed and if it's time for another attempt
                if (self.in_failsafe_mode and not self.recovery_in_progress and 
                    (self.last_recovery_attempt is None or 
                     time.time() - self.last_recovery_attempt >= self.recovery_interval)):
                    self._attempt_recovery()
                
                time.sleep(1.0)  # Check conditions every second
            except Exception as e:
                logger.error(f"Error in failsafe monitor loop: {e}")
    
    def _check_gps_status(self):
        """Check if GPS data is missing or stale"""
        if self.in_failsafe_mode:
            return
            
        current_time = time.time()
        
        # Check if we have any GPS data
        if self.last_gps_update is None:
            self._trigger_failsafe(FailsafeReason.GPS_DATA_LOSS)
            return
            
        # Check if GPS data is stale
        if current_time - self.last_gps_update > self.gps_stale_threshold:
            self._trigger_failsafe(FailsafeReason.GPS_STALE_DATA)
    
    def _check_internet_connection(self):
        """Check if internet connection for RTCM corrections is lost or slow"""
        if self.in_failsafe_mode:
            return
            
        current_time = time.time()
        
        # Check if we have any internet connectivity data
        if self.last_internet_check is None:
            return
            
        # Check if internet connection is lost or too slow
        if current_time - self.last_internet_check > self.internet_timeout:
            self._trigger_failsafe(FailsafeReason.INTERNET_CONNECTION_LOST)
    
    def _check_module_communication(self):
        """Check if module-to-computer communication is working"""
        if self.in_failsafe_mode:
            return
            
        current_time = time.time()
        
        # Check if we have any module communication data
        if self.last_module_comm is None:
            return
            
        # Module communication timeout (5 seconds)
        if current_time - self.last_module_comm > 5.0:
            self._trigger_failsafe(FailsafeReason.MODULE_COMMUNICATION_FAILURE)
    
    def _check_temperature(self):
        """Check if system temperature is too high"""
        if self.in_failsafe_mode:
            return
        
        # This is a placeholder for actual temperature checking
        # In a real system, you would read from temperature sensors
        temperature = self._get_system_temperature()
        
        if temperature > self.temperature_threshold:
            self._trigger_failsafe(FailsafeReason.OVER_TEMPERATURE)
    
    def _check_signal_stability(self):
        """Check for signal instability (≥ 5 signal-loss events per minute)"""
        if self.in_failsafe_mode:
            return
            
        current_time = time.time()
        
        # Remove signal loss events older than the window
        self.signal_loss_events = [t for t in self.signal_loss_events 
                                  if current_time - t <= self.signal_loss_window]
        
        # Check if we have too many signal loss events
        if len(self.signal_loss_events) >= self.signal_loss_threshold:
            self._trigger_failsafe(FailsafeReason.SIGNAL_INSTABILITY)
    
    def _trigger_failsafe(self, reason):
        """Trigger failsafe mode and execute callback"""
        if self.in_failsafe_mode:
            return
            
        self.in_failsafe_mode = True
        self.failsafe_reason = reason
        self.recovery_attempts = 0
        
        logger.warning(f"⚠️ FAILSAFE MODE ACTIVATED: {reason.value}")
        
        # Execute failsafe callback if provided
        if self.failsafe_callback:
            try:
                self.failsafe_callback(reason)
            except Exception as e:
                logger.error(f"Error in failsafe callback: {e}")
    
    def _attempt_recovery(self):
        """Attempt to recover from failsafe mode"""
        if not self.in_failsafe_mode or self.recovery_in_progress:
            return
            
        self.recovery_in_progress = True
        self.recovery_attempts += 1
        self.last_recovery_attempt = time.time()
        
        logger.info(f"🔄 Attempting recovery #{self.recovery_attempts} for {self.failsafe_reason.value}")
        
        # Execute recovery callback if provided
        if self.recovery_callback:
            try:
                success = self.recovery_callback(self.failsafe_reason)
                if success:
                    self._clear_failsafe()
            except Exception as e:
                logger.error(f"Error in recovery callback: {e}")
        
        # If too many attempts, give up and stay in failsafe mode
        if self.recovery_attempts >= self.max_recovery_attempts:
            logger.error(f"❌ Max recovery attempts reached for {self.failsafe_reason.value}")
            # Keep in failsafe mode but allow future recovery attempts
        
        self.recovery_in_progress = False
    
    def _clear_failsafe(self):
        """Clear failsafe mode after successful recovery"""
        if not self.in_failsafe_mode:
            return
            
        old_reason = self.failsafe_reason
        self.in_failsafe_mode = False
        self.failsafe_reason = None
        self.recovery_attempts = 0
        
        logger.info(f"✅ Recovered from failsafe mode: {old_reason.value}")
    
    # API Methods for rover interaction
    
    def update_gps_status(self, has_fix=True, satellites=0, hdop=0.0):
        """
        Update GPS status information
        
        Args:
            has_fix: Whether GPS has a valid fix
            satellites: Number of satellites in view
            hdop: Horizontal dilution of precision
        """
        if not has_fix or satellites < 4 or hdop > 5.0:
            # Register as signal loss event
            self.signal_loss_events.append(time.time())
        else:
            self.last_gps_update = time.time()
    
    def update_internet_status(self, connected=True, latency=0.0):
        """
        Update internet connection status
        
        Args:
            connected: Whether internet is connected
            latency: Latency in seconds
        """
        self.last_internet_check = time.time()
        
        # Consider high latency as a slow connection
        if not connected or latency > 2.0:
            if self.monitoring_active and not self.in_failsafe_mode:
                self._trigger_failsafe(
                    FailsafeReason.INTERNET_CONNECTION_LOST if not connected 
                    else FailsafeReason.INTERNET_CONNECTION_SLOW
                )
    
    def update_module_communication(self, timestamp=None):
        """
        Update module communication timestamp
        
        Args:
            timestamp: Optional timestamp to use, defaults to current time
        """
        self.last_module_comm = timestamp if timestamp is not None else time.time()
    
    def report_signal_loss(self):
        """Report a signal loss event"""
        self.signal_loss_events.append(time.time())
    
    def trigger_custom_failsafe(self, reason_text):
        """
        Manually trigger failsafe mode with a custom reason
        
        Args:
            reason_text: Description of the custom reason
        """
        custom_reason = FailsafeReason.CUSTOM
        self._trigger_failsafe(custom_reason)
        logger.warning(f"Custom failsafe reason: {reason_text}")
    
    def force_recovery(self):
        """Force a recovery attempt regardless of interval"""
        if self.in_failsafe_mode:
            logger.info("Forcing recovery attempt")
            self._attempt_recovery()
    
    def get_status(self):
        """Get current failsafe status information"""
        return {
            "in_failsafe_mode": self.in_failsafe_mode,
            "failsafe_reason": self.failsafe_reason.value if self.failsafe_reason else None,
            "recovery_attempts": self.recovery_attempts,
            "recovery_in_progress": self.recovery_in_progress,
            "last_gps_update": self.last_gps_update,
            "last_internet_check": self.last_internet_check,
            "last_module_comm": self.last_module_comm,
            "signal_loss_events": len(self.signal_loss_events),
            "monitoring_active": self.monitoring_active
        }
    
    def _get_system_temperature(self):
        """
        Get the current system temperature
        This is a placeholder implementation that should be replaced
        with actual hardware temperature readings in a real system
        """
        # In a real system, you would read from temperature sensors
        # For simulation, we'll return a fixed value below threshold
        return 45.0  # Simulated temperature in Celsius