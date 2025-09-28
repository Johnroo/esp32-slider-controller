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

@app.route('/advanced')
def advanced():
    return render_template('advanced.html')

@app.route('/api/status', methods=['GET'])
def api_status():
    """Check server status"""
    return jsonify({'status': 'ok', 'connected': True})

@app.route('/api/slide/jog', methods=['POST'])
def api_slide_jog():
    """Send slide jog command (-1.0 to 1.0)"""
    data = request.get_json()
    value = float(data.get('value', 0.0))
    value = max(-1.0, min(1.0, value))  # Clamp to [-1, 1]
    
    success = send_osc_message('/slide/jog', value)
    return jsonify({'success': success, 'value': value})

@app.route('/api/pan', methods=['POST'])
def api_pan():
    """Send pan joystick offset (-1.0 to 1.0)"""
    data = request.get_json()
    value = float(data.get('value', 0.0))
    value = max(-1.0, min(1.0, value))  # Clamp to [-1, 1]
    
    success = send_osc_message('/pan', value)
    return jsonify({'success': success, 'value': value})

@app.route('/api/tilt', methods=['POST'])
def api_tilt():
    """Send tilt joystick offset (-1.0 to 1.0)"""
    data = request.get_json()
    value = float(data.get('value', 0.0))
    value = max(-1.0, min(1.0, value))  # Clamp to [-1, 1]
    
    success = send_osc_message('/tilt', value)
    return jsonify({'success': success, 'value': value})

@app.route('/api/joystick/combined', methods=['POST'])
def api_joystick_combined():
    """Send combined pan/tilt joystick (-1.0 to 1.0)"""
    data = request.get_json()
    pan = float(data.get('pan', 0.0))
    tilt = float(data.get('tilt', 0.0))
    pan = max(-1.0, min(1.0, pan))  # Clamp to [-1, 1]
    tilt = max(-1.0, min(1.0, tilt))  # Clamp to [-1, 1]
    
    success = send_osc_message('/joy/pt', pan, tilt)
    return jsonify({'success': success, 'pan': pan, 'tilt': tilt})

@app.route('/api/joystick/config', methods=['POST'])
def api_joystick_config():
    """Configure joystick parameters"""
    data = request.get_json()
    deadzone = float(data.get('deadzone', 0.06))
    expo = float(data.get('expo', 0.35))
    slew = float(data.get('slew', 8000.0))
    filter_hz = float(data.get('filter_hz', 60.0))
    
    # Clamp values to valid ranges
    deadzone = max(0.0, min(0.5, deadzone))
    expo = max(0.0, min(0.95, expo))
    slew = max(0.0, slew)
    filter_hz = max(0.0, filter_hz)
    
    success = send_osc_message('/joy/config', deadzone, expo, slew, filter_hz)
    return jsonify({
        'success': success, 
        'deadzone': deadzone, 
        'expo': expo, 
        'slew': slew, 
        'filter_hz': filter_hz
    })

@app.route('/api/stop', methods=['POST'])
def api_stop():
    """Stop all movement"""
    # Stop slide jog
    success1 = send_osc_message('/slide/jog', 0.0)
    # Reset joystick offsets
    success2 = send_osc_message('/pan', 0.0)
    success3 = send_osc_message('/tilt', 0.0)
    return jsonify({'success': success1 and success2 and success3})

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

#==================== NEW: Advanced Motion Control Routes ====================

@app.route('/api/preset/set', methods=['POST'])
def set_preset():
    """Set preset position"""
    try:
        data = request.get_json()
        preset_id = int(data.get('id', 0))
        pan = int(data.get('pan', 0))
        tilt = int(data.get('tilt', 0))
        zoom = int(data.get('zoom', 0))
        slide = int(data.get('slide', 0))
        
        # Send OSC message
        send_osc_message('/preset/set', preset_id, pan, tilt, zoom, slide)
        
        return jsonify({
            'success': True,
            'preset_id': preset_id,
            'positions': {'pan': pan, 'tilt': tilt, 'zoom': zoom, 'slide': slide},
            'timestamp': datetime.now().isoformat()
        })
    except Exception as e:
        return jsonify({'error': str(e)}), 400

@app.route('/api/preset/recall', methods=['POST'])
def recall_preset():
    """Recall preset with duration"""
    try:
        data = request.get_json()
        preset_id = int(data.get('id', 0))
        duration = float(data.get('duration', 2.0))
        
        # Send OSC message
        send_osc_message('/preset/recall', preset_id, duration)
        
        return jsonify({
            'success': True,
            'preset_id': preset_id,
            'duration': duration,
            'timestamp': datetime.now().isoformat()
        })
    except Exception as e:
        return jsonify({'error': str(e)}), 400

@app.route('/api/joystick/pan', methods=['POST'])
def set_pan_offset():
    """Set pan joystick offset (-1.0 to 1.0)"""
    try:
        data = request.get_json()
        value = float(data.get('value', 0.0))
        
        # Clamp value between -1.0 and 1.0
        value = max(-1.0, min(1.0, value))
        
        # Send OSC message
        send_osc_message('/pan', value)
        
        return jsonify({
            'success': True,
            'offset': value,
            'timestamp': datetime.now().isoformat()
        })
    except Exception as e:
        return jsonify({'error': str(e)}), 400

@app.route('/api/joystick/tilt', methods=['POST'])
def set_tilt_offset():
    """Set tilt joystick offset (-1.0 to 1.0)"""
    try:
        data = request.get_json()
        value = float(data.get('value', 0.0))
        
        # Clamp value between -1.0 and 1.0
        value = max(-1.0, min(1.0, value))
        
        # Send OSC message
        send_osc_message('/tilt', value)
        
        return jsonify({
            'success': True,
            'offset': value,
            'timestamp': datetime.now().isoformat()
        })
    except Exception as e:
        return jsonify({'error': str(e)}), 400

@app.route('/api/slide/jog', methods=['POST'])
def slide_jog():
    """Slide jog mode (-1.0 to 1.0)"""
    try:
        data = request.get_json()
        value = float(data.get('value', 0.0))
        
        # Clamp value between -1.0 and 1.0
        value = max(-1.0, min(1.0, value))
        
        # Send OSC message
        send_osc_message('/slide/jog', value)
        
        return jsonify({
            'success': True,
            'jog_speed': value,
            'timestamp': datetime.now().isoformat()
        })
    except Exception as e:
        return jsonify({'error': str(e)}), 400

@app.route('/api/slide/goto', methods=['POST'])
def slide_goto():
    """Slide goto position with duration"""
    try:
        data = request.get_json()
        position = float(data.get('position', 0.5))  # 0.0 to 1.0
        duration = float(data.get('duration', 2.0))
        
        # Clamp position between 0.0 and 1.0
        position = max(0.0, min(1.0, position))
        
        # Send OSC message
        send_osc_message('/slide/goto', position, duration)
        
        return jsonify({
            'success': True,
            'position': position,
            'duration': duration,
            'timestamp': datetime.now().isoformat()
        })
    except Exception as e:
        return jsonify({'error': str(e)}), 400

@app.route('/api/config/offset_range', methods=['POST'])
def set_offset_range():
    """Set joystick offset ranges"""
    try:
        data = request.get_json()
        pan_range = int(data.get('pan_range', 800))
        tilt_range = int(data.get('tilt_range', 800))
        
        # Send OSC message
        send_osc_message('/config/offset_range', pan_range, tilt_range)
        
        return jsonify({
            'success': True,
            'pan_range': pan_range,
            'tilt_range': tilt_range,
            'timestamp': datetime.now().isoformat()
        })
    except Exception as e:
        return jsonify({'error': str(e)}), 400

@app.route('/api/config/pan_map', methods=['POST'])
def set_pan_mapping():
    """Set slide to pan mapping"""
    try:
        data = request.get_json()
        min_value = int(data.get('min', 800))
        max_value = int(data.get('max', -800))
        
        # Send OSC message
        send_osc_message('/config/pan_map', min_value, max_value)
        
        return jsonify({
            'success': True,
            'min': min_value,
            'max': max_value,
            'timestamp': datetime.now().isoformat()
        })
    except Exception as e:
        return jsonify({'error': str(e)}), 400

@app.route('/api/config/tilt_map', methods=['POST'])
def set_tilt_mapping():
    """Set slide to tilt mapping"""
    try:
        data = request.get_json()
        min_value = int(data.get('min', 0))
        max_value = int(data.get('max', 0))
        
        # Send OSC message
        send_osc_message('/config/tilt_map', min_value, max_value)
        
        return jsonify({
            'success': True,
            'min': min_value,
            'max': max_value,
            'timestamp': datetime.now().isoformat()
        })
    except Exception as e:
        return jsonify({'error': str(e)}), 400

if __name__ == '__main__':
    print(f"ESP32 Slider Controller starting...")
    print(f"OSC Target: {OSC_HOST}:{OSC_PORT}")
    print(f"Web interface: http://localhost:5000")
    app.run(debug=True, host='0.0.0.0', port=5000)
