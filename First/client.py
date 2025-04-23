import socket
import time
import pandas as pd
from gps_utils import parse_nmea, analyze_gps_data, plot_trajectory

class GPSClient:
    def __init__(self, host='localhost', port=9001):
        self.host = host
        self.port = port
        self.socket = None
        self.data = []

    def connect(self):
        self.socket = socket.socket()
        self.socket.connect((self.host, self.port))
        print(f"Connected to {self.host}:{self.port}")

    def run(self, duration=30):
        start_time = time.time()
        distance_traveled = 0
        while time.time() - start_time < duration:
            try:
                data = self.socket.recv(1024).decode()
                if parsed := parse_nmea(data):
                    self.data.append(parsed)
                    print(f"{parsed['timestamp']} | "
                          f"Lat: {parsed['latitude']:.6f} | "
                          f"Lon: {parsed['longitude']:.6f} | "
                          f"Fix: {parsed['fix_quality']} | "
                          f"Sats: {parsed['satellites']}")
                    distance_traveled += 0.1  # Increment distance by 0.1m
            except Exception as e:
                print(f"Error: {e}")
                break

        if self.data:
            df = pd.DataFrame(self.data)
            stats = analyze_gps_data(df)
            print(f"\n--- Statistics ---\n"
                  f"RTK Fix: {stats['rtk_fix_percent']:.1f}%\n"
                  f"Avg Sats: {stats['avg_satellites']:.1f}\n"
                  f"Max HDOP: {stats['max_hdop']:.1f}")
            
            df.to_csv('gps_log.csv', index=False)
            plot_trajectory(df)
        else:
            print("No data received")

if __name__ == '__main__':
    client = GPSClient()
    client.connect()
    client.run(duration=30)
