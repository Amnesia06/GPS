import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime

def calculate_nmea_checksum(sentence):
    """Compute NMEA checksum (XOR of bytes between $ and *)"""
    checksum = 0
    for char in sentence[sentence.find('$')+1:sentence.find('*')]:
        checksum ^= ord(char)
    return checksum

def parse_nmea(nmea_sentence):
    """Correctly parse NMEA GNGGA sentences"""
    if not nmea_sentence.startswith('$GNGGA'):
        return None

    try:
        parts = nmea_sentence.split(',')
        
        # Convert NMEA DDMM.MMMM to decimal degrees
        lat = float(parts[2][:2]) + float(parts[2][2:])/60
        lon = float(parts[4][:3]) + float(parts[4][3:])/60
        
        # Apply direction
        lat = lat if parts[3] == 'N' else -lat
        lon = lon if parts[5] == 'E' else -lon

        return {
            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'latitude': lat,
            'longitude': lon,
            'altitude': float(parts[9]),
            'fix_quality': int(parts[6]),
            'satellites': int(parts[7]),
            'hdop': float(parts[8]),
            'raw_sentence': nmea_sentence.strip()
        }
    except Exception as e:
        print(f"Parse error: {e}")
        return None

def analyze_gps_data(df):
    """Generate statistics from GPS data"""
    return {
        'rtk_fix_percent': (df['fix_quality'] == 2).mean() * 100,
        'avg_satellites': df['satellites'].mean(),
        'max_hdop': df['hdop'].max()
    }

def plot_trajectory(df):
    """Plot GPS trajectory with quality coloring"""
    plt.figure(figsize=(10, 6))
    colors = {1: 'red', 2: 'green', 4: 'orange'}
    plt.scatter(
        df['longitude'], df['latitude'],
        c=df['fix_quality'].map(colors),
        s=df['satellites']*20,
        alpha=0.7
    )
    plt.colorbar(label='Fix Quality (2=RTK Fix)')
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.title('Rover Path with GPS Quality')
    plt.grid(True)
    plt.savefig('gps_trajectory.png')
    plt.show()

def correct_existing_data(input_csv, output_csv):
    """Fix coordinate scaling in existing CSV data"""
    df = pd.read_csv(input_csv)
    df['latitude'] = df['latitude'] * 100
    df['longitude'] = abs(df['longitude']) * 100 * -1
    df.to_csv(output_csv, index=False)