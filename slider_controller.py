#!/usr/bin/env python3
"""
ESP32 Slider Controller - Flask Web Application
Controls the ESP32 slider via OSC messages
"""

from flask import Flask, render_template, request, jsonify
import socket
import struct
import json
import time
from datetime import datetime

app = Flask(__name__)

# OSC Configuration
OSC_HOST = "192.168.1.22"  # ESP32 IP address (esproo.local)
OSC_PORT = 8000

# OSC Message Builder
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
        sock.sendto(message, (OSC_HOST, OSC_PORT))
        sock.close()
        return True
    except Exception as e:
        print(f"Error sending OSC message: {e}")
        return False

# Web Routes
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/status', methods=['GET'])
def api_status():
    """Check server status"""
    return jsonify({'status': 'ok', 'connected': True})

@app.route('/api/jog', methods=['POST'])
def api_jog():
    """Send jog command (-1.0 to 1.0)"""
    data = request.get_json()
    value = float(data.get('value', 0.0))
    value = max(-1.0, min(1.0, value))  # Clamp to [-1, 1]
    
    success = send_osc_message('/jog', value)
    return jsonify({'success': success, 'value': value})

@app.route('/api/pan', methods=['POST'])
def api_pan():
    """Send pan offset (-1.0 to 1.0)"""
    data = request.get_json()
    value = float(data.get('value', 0.0))
    value = max(-1.0, min(1.0, value))  # Clamp to [-1, 1]
    
    success = send_osc_message('/pan', value)
    return jsonify({'success': success, 'value': value})

@app.route('/api/tilt', methods=['POST'])
def api_tilt():
    """Send tilt offset (-1.0 to 1.0)"""
    data = request.get_json()
    value = float(data.get('value', 0.0))
    value = max(-1.0, min(1.0, value))  # Clamp to [-1, 1]
    
    success = send_osc_message('/tilt', value)
    return jsonify({'success': success, 'value': value})

@app.route('/api/preset_a', methods=['POST'])
def api_preset_a():
    """Set Preset A (pan, tilt, zoom, slide in steps)"""
    data = request.get_json()
    pan = int(data.get('pan', 0))
    tilt = int(data.get('tilt', 0))
    zoom = int(data.get('zoom', 0))
    slide = int(data.get('slide', 0))
    
    success = send_osc_message('/setPresetA', pan, tilt, zoom, slide)
    return jsonify({'success': success, 'preset': {'pan': pan, 'tilt': tilt, 'zoom': zoom, 'slide': slide}})

@app.route('/api/preset_b', methods=['POST'])
def api_preset_b():
    """Set Preset B (pan, tilt, zoom, slide in steps)"""
    data = request.get_json()
    pan = int(data.get('pan', 0))
    tilt = int(data.get('tilt', 0))
    zoom = int(data.get('zoom', 0))
    slide = int(data.get('slide', 0))
    
    success = send_osc_message('/setPresetB', pan, tilt, zoom, slide)
    return jsonify({'success': success, 'preset': {'pan': pan, 'tilt': tilt, 'zoom': zoom, 'slide': slide}})

@app.route('/api/stop', methods=['POST'])
def api_stop():
    """Stop all movement"""
    success = send_osc_message('/jog', 0.0)
    return jsonify({'success': success})

@app.route('/api/reset_offsets', methods=['POST'])
def api_reset_offsets():
    """Reset pan and tilt offsets"""
    pan_success = send_osc_message('/pan', 0.0)
    tilt_success = send_osc_message('/tilt', 0.0)
    return jsonify({'success': pan_success and tilt_success})

# Nouvelles routes pour contrôle direct des axes
@app.route('/api/axis_pan', methods=['POST'])
def api_axis_pan():
    """Send direct pan position (0.0 to 1.0)"""
    data = request.get_json()
    value = float(data.get('value', 0.0))
    value = max(0.0, min(1.0, value))  # Clamp to [0, 1]
    
    success = send_osc_message('/axis_pan', value)
    return jsonify({'success': success, 'value': value})

@app.route('/api/axis_tilt', methods=['POST'])
def api_axis_tilt():
    """Send direct tilt position (0.0 to 1.0)"""
    data = request.get_json()
    value = float(data.get('value', 0.0))
    value = max(0.0, min(1.0, value))  # Clamp to [0, 1]
    
    success = send_osc_message('/axis_tilt', value)
    return jsonify({'success': success, 'value': value})

@app.route('/api/axis_zoom', methods=['POST'])
def api_axis_zoom():
    """Send direct zoom position (0.0 to 1.0)"""
    data = request.get_json()
    value = float(data.get('value', 0.0))
    value = max(0.0, min(1.0, value))  # Clamp to [0, 1]
    
    success = send_osc_message('/axis_zoom', value)
    return jsonify({'success': success, 'value': value})

@app.route('/api/axis_slide', methods=['POST'])
def api_axis_slide():
    """Send direct slide position (0.0 to 1.0)"""
    data = request.get_json()
    value = float(data.get('value', 0.0))
    value = max(0.0, min(1.0, value))  # Clamp to [0, 1]
    
    success = send_osc_message('/axis_slide', value)
    return jsonify({'success': success, 'value': value})

@app.route('/api/reset_all_axes', methods=['POST'])
def api_reset_all_axes():
    """Reset all axes to center position (0.5)"""
    pan_success = send_osc_message('/axis_pan', 0.5)
    tilt_success = send_osc_message('/axis_tilt', 0.5)
    zoom_success = send_osc_message('/axis_zoom', 0.5)
    slide_success = send_osc_message('/axis_slide', 0.5)
    return jsonify({'success': pan_success and tilt_success and zoom_success and slide_success})

if __name__ == '__main__':
    print(f"ESP32 Slider Controller starting...")
    print(f"OSC Target: {OSC_HOST}:{OSC_PORT}")
    print(f"Web interface: http://localhost:5000")
    app.run(debug=True, host='0.0.0.0', port=5000)
