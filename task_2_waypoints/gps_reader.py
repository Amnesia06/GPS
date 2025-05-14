import serial
import time
from datetime import datetime
import argparse
import logging
import sys

def setup_logging():
    """Configure logging to both console and file."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler('gps_data.log'),
            logging.StreamHandler(sys.stdout)
        ]
    )

def parse_gngga(sentence):
    """Parse $GNGGA NMEA sentence and return relevant GPS data."""
    parts = sentence.split(',')
    if len(parts) < 15:
        return None

    try:
        # Check for valid fix (quality > 0)
        fix_quality = int(parts[6]) if parts[6] else 0
        if fix_quality == 0:
            return None

        # Extract latitude
        if parts[2] and parts[3]:
            lat_deg = float(parts[2][:2])
            lat_min = float(parts[2][2:])
            latitude = lat_deg + (lat_min / 60.0)
            if parts[3] == 'S':
                latitude *= -1
        else:
            return None

        # Extract longitude
        if parts[4] and parts[5]:
            lon_deg = float(parts[4][:3])
            lon_min = float(parts[4][3:])
            longitude = lon_deg + (lon_min / 60.0)
            if parts[5] == 'W':
                longitude *= -1
        else:
            return None

        # Extract additional fields
        satellites = int(parts[7]) if parts[7] else 0
        hdop = float(parts[8]) if parts[8] else 0.0
        altitude = float(parts[9]) if parts[9] else 0.0
        altitude_unit = parts[10] if parts[10] else 'M'

        return {
            'latitude': latitude,
            'longitude': longitude,
            'fix_quality': fix_quality,
            'satellites': satellites,
            'hdop': hdop,
            'altitude': altitude,
            'altitude_unit': altitude_unit
        }
    except (ValueError, IndexError):
        return None

def parse_gnrmc(sentence):
    """Parse $GNRMC NMEA sentence for speed and date."""
    parts = sentence.split(',')
    if len(parts) < 12:
        return None

    try:
        # Check status (A = active, V = void)
        if parts[2] != 'A':
            return None

        # Extract speed (knots) and convert to km/h
        speed_knots = float(parts[7]) if parts[7] else 0.0
        speed_kmh = speed_knots * 1.852

        # Extract date
        date = parts[9] if parts[9] else ''
        if date:
            date = f"20{date[4:6]}-{date[2:4]}-{date[0:2]}"  # Format as YYYY-MM-DD

        return {
            'speed_kmh': speed_kmh,
            'date': date
        }
    except (ValueError, IndexError):
        return None

def main():
    """Main function to read and process GPS data."""
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='GPS NMEA Data Reader')
    parser.add_argument('--port', default='COM8', help='Serial port (e.g., COM8 or /dev/ttyUSB0)')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate (e.g., 115200)')
    args = parser.parse_args()

    # Setup logging
    setup_logging()
    logger = logging.getLogger()

    # Configure serial connection
    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            timeout=1
        )
        logger.info(f"Connected to: {ser.port}")
    except serial.SerialException as e:
        logger.error(f"Failed to connect to {args.port}: {e}")
        sys.exit(1)

    try:
        while True:
            try:
                # Flush input buffer
                ser.reset_input_buffer()

                # Read and decode line
                line = ser.readline().decode('ascii', errors='ignore').strip()
                if not line:
                    continue

                timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                data = None

                # Parse NMEA sentences
                if line.startswith('$GNGGA'):
                    data = parse_gngga(line)
                    if data:
                        log_message = (
                            f"[{timestamp}] Latitude: {data['latitude']:.6f}, "
                            f"Longitude: {data['longitude']:.6f}, "
                            f"Altitude: {data['altitude']:.1f} {data['altitude_unit']}, "
                            f"Satellites: {data['satellites']}, "
                            f"Fix Quality: {data['fix_quality']}, "
                            f"HDOP: {data['hdop']:.1f}"
                        )
                        logger.info(log_message)
                    else:
                        logger.info(f"[{timestamp}] No GPS Fix (GNGGA)")
                
                elif line.startswith('$GNRMC'):
                    data = parse_gnrmc(line)
                    if data:
                        log_message = (
                            f"[{timestamp}] Speed: {data['speed_kmh']:.2f} km/h, "
                            f"Date: {data['date']}"
                        )
                        logger.info(log_message)
                    else:
                        logger.info(f"[{timestamp}] No Valid RMC Data")

                time.sleep(0.1)  # Wait 100ms to avoid overwhelming the port

            except serial.SerialException as e:
                logger.error(f"Serial error: {e}")
                break
            except UnicodeDecodeError:
                logger.warning("Failed to decode line, skipping...")
                continue

    except KeyboardInterrupt:
        logger.info("Exiting...")
    finally:
        ser.close()
        logger.info("Serial port closed")

if __name__ == "__main__":
    main()