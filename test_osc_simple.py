#!/usr/bin/env python3
"""
Test OSC simple pour vérifier la communication avec l'ESP32
"""

import socket
import struct

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
        sock.sendto(message, ("192.168.1.22", 8000))
        sock.close()
        print(f"Sent OSC: {address} with args: {args}")
        return True
    except Exception as e:
        print(f"Error sending OSC message: {e}")
        return False

if __name__ == "__main__":
    print("Testing OSC communication with ESP32...")
    
    # Test simple
    send_osc_message("/axis_pan", 0.5)
    send_osc_message("/jog", 0.0)
    send_osc_message("/pan", 0.0)
    send_osc_message("/tilt", 0.0)
    
    print("OSC messages sent!")
