#!/usr/bin/env python3
"""
Test de communication avec l'ESP32
"""

import socket
import struct
import time
import requests

def create_osc_message(address, *args):
    """Créer un message OSC"""
    # Adresse
    address_data = address.encode('utf-8') + b'\x00'
    pad = (4 - (len(address_data) % 4)) % 4
    address_data += b'\x00' * pad
    
    # Type tag
    tags = ',' + ''.join('f' if isinstance(a, float) else 'i' for a in args)
    type_tag_data = tags.encode('utf-8') + b'\x00'
    pad = (4 - (len(type_tag_data) % 4)) % 4
    type_tag_data += b'\x00' * pad
    
    # Arguments
    args_data = b''
    for a in args:
        if isinstance(a, float):
            args_data += struct.pack('>f', a)
        else:
            args_data += struct.pack('>i', int(a))
    
    return address_data + type_tag_data + args_data

def test_esp32_communication():
    """Test de communication avec l'ESP32"""
    print("🔌 Test de communication ESP32")
    print("=" * 50)
    
    # 1. Test de ping
    print("1️⃣ Test de ping ESP32...")
    try:
        import subprocess
        result = subprocess.run(['ping', '-c', '1', '192.168.1.198'], 
                              capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            print("✅ ESP32 accessible")
        else:
            print("❌ ESP32 non accessible")
            return
    except Exception as e:
        print(f"❌ Erreur ping: {e}")
        return
    
    # 2. Test d'envoi OSC simple
    print("\n2️⃣ Test d'envoi OSC simple...")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Test avec des valeurs fixes
        message = create_osc_message('/joy/pt', 0.5, 0.3)
        sock.sendto(message, ('192.168.1.198', 8000))
        print("✅ Message OSC envoyé: /joy/pt 0.5, 0.3")
        
        time.sleep(0.1)
        
        # Test avec des valeurs différentes
        message = create_osc_message('/joy/pt', -0.2, 0.8)
        sock.sendto(message, ('192.168.1.198', 8000))
        print("✅ Message OSC envoyé: /joy/pt -0.2, 0.8")
        
        sock.close()
    except Exception as e:
        print(f"❌ Erreur OSC: {e}")
    
    # 3. Test d'autres commandes OSC
    print("\n3️⃣ Test d'autres commandes OSC...")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Test commande pan directe
        message = create_osc_message('/pan', 0.5)
        sock.sendto(message, ('192.168.1.198', 8000))
        print("✅ Message OSC envoyé: /pan 0.5")
        
        time.sleep(0.1)
        
        # Test commande tilt directe
        message = create_osc_message('/tilt', 0.3)
        sock.sendto(message, ('192.168.1.198', 8000))
        print("✅ Message OSC envoyé: /tilt 0.3")
        
        sock.close()
    except Exception as e:
        print(f"❌ Erreur OSC: {e}")
    
    # 4. Test via l'API Flask
    print("\n4️⃣ Test via l'API Flask...")
    try:
        # Test de l'API joystick
        response = requests.get("http://localhost:9001/api/joystick/state")
        if response.status_code == 200:
            print("✅ API Flask fonctionne")
            
            # Test d'envoi de commandes via l'API
            data = {"value": 0.5}
            response = requests.post("http://localhost:9001/api/joystick/pan", json=data)
            if response.status_code == 200:
                print("✅ Commande pan envoyée via API")
            
            data = {"value": 0.3}
            response = requests.post("http://localhost:9001/api/joystick/tilt", json=data)
            if response.status_code == 200:
                print("✅ Commande tilt envoyée via API")
        else:
            print(f"❌ Erreur API: {response.status_code}")
    except Exception as e:
        print(f"❌ Erreur API: {e}")
    
    print("\n💡 Vérifications à faire:")
    print("1. Regardez les logs de l'ESP32 (moniteur série)")
    print("2. Vérifiez que les moteurs bougent")
    print("3. Testez avec l'interface web: http://localhost:9001/advanced")

if __name__ == "__main__":
    test_esp32_communication()
