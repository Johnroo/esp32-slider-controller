"""
Configuration file for ESP32 Slider Controller
Update these settings according to your setup
"""

# ESP32 Network Configuration
ESP32_IP = "192.168.1.22"  # ESP32 IP address (esproo.local)
ESP32_OSC_PORT = 8000

# Flask Web Server Configuration
FLASK_HOST = "0.0.0.0"  # Listen on all interfaces
FLASK_PORT = 9000
FLASK_DEBUG = True

# OSC Message Configuration
OSC_TIMEOUT = 1.0  # seconds

# Default Preset Values
DEFAULT_PRESET_A = {
    "pan": 0,
    "tilt": 0,
    "zoom": 0,
    "slide": 0
}

DEFAULT_PRESET_B = {
    "pan": 1000,
    "tilt": 500,
    "zoom": 2000,
    "slide": 5000
}
