import os
import sys
import time
import serial
import psutil
import subprocess
import ctypes
def is_admin():
    """Check if the script is running with admin privileges"""
    try:
        return ctypes.windll.shell32.IsUserAnAdmin()
    except:
        return False
def force_cleanup_com12():
    """Aggressive COM12 cleanup with elevated privileges check"""
    print("\n🧹 Starting aggressive COM12 cleanup...")
     # Better admin check
    if not is_admin():
        print("⚠️ Script needs admin privileges!")
        print("Please:")
        print("1. Right-click Command Prompt")
        print("2. Select 'Run as administrator'")
        print("3. Navigate to: F:\\GPS\\task_2_waypoints")
        print("4. Run: python port_cleanup.py")
        return False
        
    # Check if running as admin
   
        
    try:
        # Step 1: Kill any processes using serial ports
        print("\n1️⃣ Killing processes...")
        current_pid = os.getpid()
        
        for proc in psutil.process_iter(['pid', 'name']):
            try:
                if proc.pid != current_pid:
                    if proc.name().lower() in ['python.exe', 'pythonw.exe', 
                                             'putty.exe', 'realterm.exe', 
                                             'terraterm.exe', 'reachview.exe']:
                        print(f"   Killing {proc.name()} (PID: {proc.pid})")
                        proc.kill()
            except:
                continue
                
        time.sleep(2)  # Wait for processes to die
        
        # Step 2: Reset COM port settings
        print("\n2️⃣ Resetting COM12 settings...")
        subprocess.run("mode COM12 BAUD=115200 PARITY=N DATA=8 STOP=1", 
                      shell=True, check=False)
        
        # Step 3: Try to open and close port
        print("\n3️⃣ Testing port access...")
        ser = serial.Serial()
        ser.port = 'COM12'
        ser.baudrate = 115200
        ser.timeout = 1
        
        try:
            ser.open()
            print("✅ Successfully opened COM12")
            ser.close()
            print("✅ Successfully closed COM12")
        except serial.SerialException as e:
            print(f"❌ Port access failed: {e}")
            return False
            
        print("\n✅ COM12 cleanup complete!")
        return True
        
    except Exception as e:
        print(f"\n❌ Cleanup failed: {e}")
        return False

if __name__ == "__main__":
    force_cleanup_com12()