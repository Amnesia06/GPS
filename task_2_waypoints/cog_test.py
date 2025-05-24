import serial
try:
    ser = serial.Serial('COM12', 115200, timeout=1)
    print("✅ Connected to COM12")
    while True:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                if line.startswith(('$GPRMC', '$GNRMC')):
                    print(f"Received: {line}")
                    parts = line.split(',')
                    if len(parts) >= 10:
                        status = parts[2]
                        cog = parts[8] if parts[8] else "Empty"
                        speed = parts[7] if parts[7] else "0.0"
                        print(f"Status: {status}, COG: {cog}, Speed: {speed} knots")
                    else:
                        print("Incomplete $GPRMC sentence")
                else:
                    print(f"Other NMEA: {line}")  # Log other sentences for context
        except Exception as e:
            print(f"Read error: {e}")
except serial.SerialException as e:
    print(f"Failed to open COM12: {e}")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Closed COM12")