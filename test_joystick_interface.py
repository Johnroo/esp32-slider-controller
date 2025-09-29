#!/usr/bin/env python3
"""
Test de l'interface joystick avec simulation
"""

import requests
import json
import time

def test_joystick_interface():
    """Test de l'interface joystick"""
    print("🎮 Test de l'interface joystick")
    print("=" * 50)
    
    # 1. Test du serveur
    try:
        response = requests.get("http://localhost:9000/api/status")
        print(f"✅ Serveur actif: {response.status_code}")
    except Exception as e:
        print(f"❌ Serveur non accessible: {e}")
        return
    
    # 2. Test de l'état du joystick
    print("\n📊 Test de l'état du joystick...")
    for i in range(5):
        try:
            response = requests.get("http://localhost:9000/api/joystick/state")
            data = response.json()
            print(f"Poll {i+1}: X={data.get('x', 0):.3f}, Y={data.get('y', 0):.3f}, Connected={data.get('connected', False)}")
            time.sleep(1)
        except Exception as e:
            print(f"❌ Erreur: {e}")
    
    # 3. Test de l'interface web
    print("\n🌐 Test de l'interface web...")
    try:
        response = requests.get("http://localhost:9000/advanced")
        if response.status_code == 200:
            print("✅ Interface web accessible")
            if "joyCanvas" in response.text:
                print("✅ Canvas joystick détecté")
            else:
                print("❌ Canvas joystick non trouvé")
        else:
            print(f"❌ Erreur HTTP: {response.status_code}")
    except Exception as e:
        print(f"❌ Erreur interface: {e}")
    
    print("\n💡 Instructions:")
    print("1. Ouvrez http://localhost:9000/advanced dans votre navigateur")
    print("2. Vous devriez voir un cercle avec un point qui bouge")
    print("3. Le point devrait faire un mouvement circulaire (simulation)")

if __name__ == "__main__":
    test_joystick_interface()
