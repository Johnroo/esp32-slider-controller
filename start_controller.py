#!/usr/bin/env python3
"""
ESP32 Slider Controller Startup Script
This script helps you configure and start the web controller
"""

import os
import sys
import socket
from config import ESP32_IP, ESP32_OSC_PORT, FLASK_PORT

def check_esp32_connection():
    """Check if ESP32 is reachable"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(2)
        sock.connect((ESP32_IP, ESP32_OSC_PORT))
        sock.close()
        return True
    except:
        return False

def main():
    print("🎬 ESP32 Slider Controller")
    print("=" * 40)
    
    # Check configuration
    print(f"📡 ESP32 IP: {ESP32_IP}")
    print(f"🔌 OSC Port: {ESP32_OSC_PORT}")
    print(f"🌐 Web Port: {FLASK_PORT}")
    print()
    
    # Test ESP32 connection
    print("🔍 Testing ESP32 connection...")
    if check_esp32_connection():
        print("✅ ESP32 is reachable!")
    else:
        print("❌ ESP32 not reachable!")
        print(f"   Make sure your ESP32 is connected to WiFi")
        print(f"   and running the slider firmware.")
        print(f"   Current IP configured: {ESP32_IP}")
        print()
        response = input("Continue anyway? (y/n): ")
        if response.lower() != 'y':
            print("Exiting...")
            sys.exit(1)
    
    print()
    print("🚀 Starting web server...")
    print(f"📱 Open your browser to: http://localhost:{FLASK_PORT}")
    print("🛑 Press Ctrl+C to stop the server")
    print()
    
    # Import and run the Flask app
    from slider_controller import app, _start_joystick_thread
    
    # Start joystick thread
    print("🎮 Starting joystick thread...")
    _start_joystick_thread()
    
    # Supprimer les logs HTTP pour avoir une console propre
    import logging
    log = logging.getLogger('werkzeug')
    log.setLevel(logging.ERROR)
    
    app.run(host='0.0.0.0', port=FLASK_PORT, debug=False)

if __name__ == '__main__':
    main()

