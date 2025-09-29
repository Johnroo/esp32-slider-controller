#!/usr/bin/env python3
"""
Script de test pour l'intégration joystick pygame
"""

import requests
import json
import time

# Configuration
BASE_URL = "http://localhost:9000"

def test_joystick_integration():
    """Test de l'intégration joystick"""
    print("🎮 Test de l'intégration joystick")
    print("=" * 50)
    
    # 1. Test du statut du joystick
    print("\n1️⃣ Test du statut du joystick...")
    try:
        response = requests.get(f"{BASE_URL}/api/joystick/state")
        print(f"✅ Status: {response.status_code}")
        data = response.json()
        print(f"📄 Response: {json.dumps(data, indent=2)}")
        
        if data.get('connected'):
            print(f"🎮 Joystick connecté: {data.get('name', 'Unknown')}")
            print(f"📍 Position: X={data.get('x', 0):.2f}, Y={data.get('y', 0):.2f}")
        else:
            print("❌ Aucun joystick détecté")
            
    except Exception as e:
        print(f"❌ Erreur: {e}")
    
    # 2. Test de l'interface web
    print("\n2️⃣ Test de l'interface web...")
    try:
        response = requests.get(f"{BASE_URL}/advanced")
        print(f"✅ Status: {response.status_code}")
        if "joyCanvas" in response.text:
            print("✅ Canvas joystick détecté dans l'HTML")
        else:
            print("❌ Canvas joystick non trouvé")
    except Exception as e:
        print(f"❌ Erreur: {e}")
    
    # 3. Test de polling continu (simulation)
    print("\n3️⃣ Test de polling continu...")
    for i in range(3):
        try:
            response = requests.get(f"{BASE_URL}/api/joystick/state")
            data = response.json()
            print(f"📊 Poll {i+1}: X={data.get('x', 0):.2f}, Y={data.get('y', 0):.2f}, Connected={data.get('connected', False)}")
            time.sleep(0.5)
        except Exception as e:
            print(f"❌ Erreur: {e}")
    
    print("\n🎉 Tests terminés !")
    print("\n💡 Instructions:")
    print("1. Connectez votre Logitech Extreme 3D Pro")
    print("2. Ouvrez http://localhost:9000/advanced")
    print("3. Vous devriez voir un cercle avec un point qui bouge")
    print("4. Les faders disparaîtront automatiquement si le joystick est détecté")

if __name__ == "__main__":
    test_joystick_integration()
