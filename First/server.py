import socket
import time
import random
from gps_utils import calculate_nmea_checksum

def generate_mock_nmea():
    """Generate realistic mock NMEA data"""
    base_lat = 37.7749
    base_lon = -122.4194
    lat = base_lat + random.uniform(-0.0001, 0.0001)
    lon = base_lon + random.uniform(-0.0001, 0.0001)
    
    # Format as DDMM.MMMM
    lat_deg = int(lat)
    lat_min = (lat - lat_deg) * 60
    lon_deg = int(abs(lon))
    lon_min = (abs(lon) - lon_deg) * 60
    
    fix_quality = random.choices([1, 2, 4], weights=[0.2, 0.6, 0.2])[0]
    satellites = random.randint(8, 12)
    
    nmea_template = (
        f"$GNGGA,{time.strftime('%H%M%S')},"
        f"{lat_deg:02d}{lat_min:08.5f},{'N' if lat >=0 else 'S'},"
        f"{lon_deg:03d}{lon_min:08.5f},{'E' if lon >=0 else 'W'},"
        f"{fix_quality},{satellites},1.2,100.0,M,0.0,M,,*"
    )
    checksum = calculate_nmea_checksum(nmea_template)
    return f"{nmea_template}{checksum:02X}\r\n"

def start_mock_server(host='0.0.0.0', port=9001):
    with socket.socket() as s:
        s.bind((host, port))
        s.listen()
        print(f"Mock server running on {host}:{port}")
        conn, addr = s.accept()
        with conn:
            print(f"Client connected: {addr}")
            try:
                while True:
                    nmea = generate_mock_nmea()
                    conn.sendall(nmea.encode())
                    time.sleep(1)
            except KeyboardInterrupt:
                print("Server stopped")

if __name__ == '__main__':
    start_mock_server()