import logging
import time
from threading import Thread, Event

class GPSSystemMonitor:
    def __init__(self, emlid_reader, ntrip_client= None):
        self.emlid_reader = emlid_reader
        self.ntrip_client = ntrip_client
        self.monitor_thread = None
        self.stop_event = Event()
        
    def start_monitoring(self):
        if not self.monitor_thread or not self.monitor_thread.is_alive():
            self.stop_event.clear()
            self.monitor_thread = Thread(target=self._monitor_loop, daemon=True)
            self.monitor_thread.start()
            logging.info("GPS monitoring started")

    def stop_monitoring(self):
        """Stop the monitoring thread"""
        if self.monitor_thread:
            self.stop_event.set()
            self.monitor_thread.join(timeout=2.0)
            
    def _monitor_loop(self):
        """Main monitoring loop"""
        while not self.stop_event.is_set():
            try:
                # Check GPS health
                gps_health = self.emlid_reader.check_health()
                if gps_health['last_update'] > 5.0:  # No updates for 5 seconds
                    logging.warning("GPS data stale - attempting reconnect")
                    self.emlid_reader.disconnect()
                    self.emlid_reader.connect()
                
                # Check NTRIP connection
                if not self.ntrip_client.connected:
                    logging.warning("NTRIP connection lost - attempting reconnect")
                    self.ntrip_client.connect()
                
                # Log system status
                logging.info(f"System Status - GPS: {gps_health['fix_quality']}, "
                           f"Satellites: {gps_health['satellites']}, "
                           f"NTRIP: {self.ntrip_client.connected}")
                
            except Exception as e:
                logging.error(f"Monitor error: {e}")
            
            time.sleep(1)  # Check every second


    def _check_gps_quality(self):
        if self.emlid_reader and self.emlid_reader.last_position:
            fix = self.emlid_reader.last_position.get('solution_status', 'Unknown')
            sats = self.emlid_reader.last_position.get('satellites', 0)
            hdop = self.emlid_reader.last_position.get('hdop', 0.0)
            print(f"GPS Quality: {fix} ({sats} satellites, HDOP: {hdop:.2f})")

    
    def cleanup(self):
        """Stop monitoring and cleanup resources"""
        if self.monitor_thread and self.monitor_thread.is_alive():
            self.stop_event.set()
            self.monitor_thread.join(timeout=2.0)
            logging.info("GPS monitoring stopped")