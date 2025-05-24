import serial
import time

def test_com12():
    try:
        print("Attempting to connect to COM12...")
        ser = serial.Serial(
            port='COM12',
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
        print("✅ Connected to COM12")
        
        for _ in range(10):
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(f"Received: {line}")
            time.sleep(0.1)
        
        ser.close()
    except serial.SerialException as e:
        print(f"❌ Failed to connect: {e}")
    except Exception as e:
        print(f"❌ Unexpected error: {e}")

if __name__ == "__main__":
    test_com12()