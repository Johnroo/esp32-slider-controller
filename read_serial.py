#!/usr/bin/env python3
import serial
import time
import sys

def read_serial():
    try:
        ser = serial.Serial('/dev/cu.usbserial-0001', 115200, timeout=1)
        print("📡 Reading ESP32 serial output...")
        print("Press Ctrl+C to stop")
        
        start_time = time.time()
        while time.time() - start_time < 15:  # Read for 15 seconds
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"[{time.strftime('%H:%M:%S')}] {line}")
            time.sleep(0.01)
            
    except serial.SerialException as e:
        print(f"❌ Serial error: {e}")
    except KeyboardInterrupt:
        print("\n🛑 Stopped by user")
    finally:
        if 'ser' in locals():
            ser.close()
            print("📡 Serial port closed")

if __name__ == "__main__":
    read_serial()
