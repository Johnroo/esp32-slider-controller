#!/usr/bin/env python3
"""
Test des commandes OSC vers l'ESP32
"""

import requests
import time
import socket

def test_osc_commands():
    """Test des commandes OSC"""
    print("🎮 Test des commandes OSC vers l'ESP32")
    print("=" * 50)
    
    # 1. Test de l'API joystick
    print("1️⃣ Test de l'API joystick...")
    try:
        response = requests.get("http://localhost:9001/api/joystick/state")
        data = response.json()
        print(f"✅ Joystick connecté: {data.get('connected', False)}")
        print(f"📍 Position: X={data.get('x', 0):.3f}, Y={data.get('y', 0):.3f}")
    except Exception as e:
        print(f"❌ Erreur API: {e}")
        return
    
    # 2. Test de ping ESP32
    print("\n2️⃣ Test de connectivité ESP32...")
    try:
        import subprocess
        result = subprocess.run(['ping', '-c', '1', '192.168.1.198'], 
                              capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            print("✅ ESP32 accessible")
        else:
            print("❌ ESP32 non accessible")
    except Exception as e:
        print(f"❌ Erreur ping: {e}")
    
    # 3. Test d'envoi OSC direct
    print("\n3️⃣ Test d'envoi OSC direct...")
    try:
        # Créer un socket UDP
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Message OSC simple
        message = b'/joy/pt,ff\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'
        
        # Envoyer vers l'ESP32
        sock.sendto(message, ('192.168.1.198', 8000))
        print("✅ Message OSC envoyé vers ESP32")
        
        sock.close()
    except Exception as e:
        print(f"❌ Erreur OSC: {e}")
    
    # 4. Test de l'interface web
    print("\n4️⃣ Test de l'interface web...")
    try:
        response = requests.get("http://localhost:9001/advanced")
        if response.status_code == 200:
            print("✅ Interface web accessible")
            print("🌐 Ouvrez: http://localhost:9001/advanced")
        else:
            print(f"❌ Erreur HTTP: {response.status_code}")
    except Exception as e:
        print(f"❌ Erreur interface: {e}")
    
    print("\n💡 Instructions:")
    print("1. Ouvrez http://localhost:9001/advanced")
    print("2. Vous devriez voir un cercle avec un point qui bouge")
    print("3. Les commandes OSC sont envoyées vers l'ESP32")
    print("4. Vérifiez que les moteurs bougent sur l'ESP32")

if __name__ == "__main__":
    test_osc_commands()
