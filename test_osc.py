#!/usr/bin/env python3
"""
OSC Test Script for ESP32 Slider
Tests OSC communication with the ESP32
"""

import socket
import struct
import time
from config import ESP32_IP, ESP32_OSC_PORT

def create_osc_message(address, *args):
    """Create an OSC message packet"""
    # OSC address pattern
    address_bytes = address.encode('utf-8')
    address_padding = (4 - (len(address_bytes) % 4)) % 4
    address_data = address_bytes + b'\x00' * address_padding
    
    # Type tag string
    type_tags = ',' + ''.join(['f' if isinstance(arg, float) else 'i' for arg in args])
    type_tag_bytes = type_tags.encode('utf-8')
    type_tag_padding = (4 - (len(type_tag_bytes) % 4)) % 4
    type_tag_data = type_tag_bytes + b'\x00' * type_tag_padding
    
    # Arguments
    args_data = b''
    for arg in args:
        if isinstance(arg, float):
            args_data += struct.pack('>f', arg)
        else:
            args_data += struct.pack('>i', int(arg))
    
    return address_data + type_tag_data + args_data

def send_osc_message(address, *args):
    """Send OSC message to ESP32"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        message = create_osc_message(address, *args)
        sock.sendto(message, (ESP32_IP, ESP32_OSC_PORT))
        sock.close()
        return True
    except Exception as e:
        print(f"Error sending OSC message: {e}")
        return False

def test_connection():
    """Test basic OSC communication"""
    print(f"🔍 Testing OSC connection to {ESP32_IP}:{ESP32_OSC_PORT}")
    
    # Test 1: Send jog command
    print("📤 Sending jog command (0.5)...")
    if send_osc_message('/jog', 0.5):
        print("✅ Jog command sent successfully")
    else:
        print("❌ Failed to send jog command")
        return False
    
    time.sleep(1)
    
    # Test 2: Send pan offset
    print("📤 Sending pan offset (0.2)...")
    if send_osc_message('/pan', 0.2):
        print("✅ Pan command sent successfully")
    else:
        print("❌ Failed to send pan command")
        return False
    
    time.sleep(1)
    
    # Test 3: Send tilt offset
    print("📤 Sending tilt offset (0.1)...")
    if send_osc_message('/tilt', 0.1):
        print("✅ Tilt command sent successfully")
    else:
        print("❌ Failed to send tilt command")
        return False
    
    time.sleep(1)
    
    # Test 4: Send preset
    print("📤 Sending preset A (100, 200, 300, 400)...")
    if send_osc_message('/setPresetA', 100, 200, 300, 400):
        print("✅ Preset A command sent successfully")
    else:
        print("❌ Failed to send preset A command")
        return False
    
    time.sleep(1)
    
    # Test 5: Stop movement
    print("📤 Sending stop command...")
    if send_osc_message('/jog', 0.0):
        print("✅ Stop command sent successfully")
    else:
        print("❌ Failed to send stop command")
        return False
    
    time.sleep(1)
    
    # Test 6: Direct axis control
    print("📤 Testing direct axis control...")
    axis_tests = [
        ('/axis_pan', 0.5, "Pan axis to center"),
        ('/axis_tilt', 0.5, "Tilt axis to center"),
        ('/axis_zoom', 0.5, "Zoom axis to center"),
        ('/axis_slide', 0.5, "Slide axis to center")
    ]
    
    for address, value, description in axis_tests:
        print(f"📤 {description}...")
        if send_osc_message(address, value):
            print(f"✅ {description} sent successfully")
        else:
            print(f"❌ Failed to send {description}")
            return False
        time.sleep(0.5)
    
    print("\n🎉 All OSC tests completed!")
    print("If your ESP32 is running the slider firmware,")
    print("you should see movement and log messages.")
    
    return True

def interactive_test():
    """Interactive OSC testing"""
    print("\n🎮 Interactive OSC Test")
    print("Commands: jog, pan, tilt, preset_a, preset_b, axis_pan, axis_tilt, axis_zoom, axis_slide, stop, quit")
    
    while True:
        try:
            cmd = input("\nEnter command: ").strip().lower()
            
            if cmd == 'quit':
                break
            elif cmd == 'jog':
                value = float(input("Jog value (-1.0 to 1.0): "))
                send_osc_message('/jog', value)
            elif cmd == 'pan':
                value = float(input("Pan offset (-1.0 to 1.0): "))
                send_osc_message('/pan', value)
            elif cmd == 'tilt':
                value = float(input("Tilt offset (-1.0 to 1.0): "))
                send_osc_message('/tilt', value)
            elif cmd == 'axis_pan':
                value = float(input("Pan axis position (0.0 to 1.0): "))
                send_osc_message('/axis_pan', value)
            elif cmd == 'axis_tilt':
                value = float(input("Tilt axis position (0.0 to 1.0): "))
                send_osc_message('/axis_tilt', value)
            elif cmd == 'axis_zoom':
                value = float(input("Zoom axis position (0.0 to 1.0): "))
                send_osc_message('/axis_zoom', value)
            elif cmd == 'axis_slide':
                value = float(input("Slide axis position (0.0 to 1.0): "))
                send_osc_message('/axis_slide', value)
            elif cmd == 'preset_a':
                pan = int(input("Pan steps: "))
                tilt = int(input("Tilt steps: "))
                zoom = int(input("Zoom steps: "))
                slide = int(input("Slide steps: "))
                send_osc_message('/setPresetA', pan, tilt, zoom, slide)
            elif cmd == 'preset_b':
                pan = int(input("Pan steps: "))
                tilt = int(input("Tilt steps: "))
                zoom = int(input("Zoom steps: "))
                slide = int(input("Slide steps: "))
                send_osc_message('/setPresetB', pan, tilt, zoom, slide)
            elif cmd == 'stop':
                send_osc_message('/jog', 0.0)
                send_osc_message('/pan', 0.0)
                send_osc_message('/tilt', 0.0)
            else:
                print("Unknown command. Try: jog, pan, tilt, axis_pan, axis_tilt, axis_zoom, axis_slide, preset_a, preset_b, stop, quit")
                
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Error: {e}")
    
    print("Goodbye!")

if __name__ == '__main__':
    print("🎬 ESP32 Slider OSC Test")
    print("=" * 30)
    
    if test_connection():
        print("\n" + "=" * 30)
        response = input("Run interactive test? (y/n): ")
        if response.lower() == 'y':
            interactive_test()
    else:
        print("\n❌ OSC test failed!")
        print("Make sure your ESP32 is connected and running the slider firmware.")
