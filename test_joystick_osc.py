#!/usr/bin/env python3
"""
Test des commandes OSC depuis le joystick physique
"""

import requests
import time
import json

def test_joystick_osc():
    """Test des commandes OSC depuis le joystick"""
    print("🎮 Test des commandes OSC depuis le joystick physique")
    print("=" * 60)
    
    print("1️⃣ Test de l'API joystick...")
    try:
        response = requests.get("http://localhost:9001/api/joystick/state")
        if response.status_code == 200:
            data = response.json()
            print(f"✅ API accessible")
            print(f"   Joystick: {data.get('name', 'Inconnu')}")
            print(f"   Connecté: {data.get('connected', False)}")
            print(f"   Position: X={data.get('x', 0):.3f}, Y={data.get('y', 0):.3f}")
            print(f"   Z (twist): {data.get('z', 0):.3f}")
            print(f"   Throttle: {data.get('throttle', 0):.3f}")
            
            # Compter les boutons pressés
            buttons = data.get('buttons', [])
            pressed = sum(1 for b in buttons if b)
            print(f"   Boutons pressés: {pressed}/{len(buttons)}")
        else:
            print(f"❌ Erreur API: {response.status_code}")
            return
    except Exception as e:
        print(f"❌ Erreur connexion: {e}")
        return
    
    print("\n2️⃣ Test de mouvement du joystick...")
    print("💡 Bougez le joystick pour voir les changements de position")
    print("   (Appuyez sur Ctrl+C pour arrêter)")
    
    try:
        last_x, last_y = 0, 0
        for i in range(20):  # 20 échantillons
            response = requests.get("http://localhost:9001/api/joystick/state")
            if response.status_code == 200:
                data = response.json()
                x = data.get('x', 0)
                y = data.get('y', 0)
                
                # Afficher seulement si il y a un changement significatif
                if abs(x - last_x) > 0.01 or abs(y - last_y) > 0.01:
                    print(f"   Position: X={x:.3f}, Y={y:.3f}")
                    last_x, last_y = x, y
                
                # Vérifier les boutons
                buttons = data.get('buttons', [])
                pressed_buttons = [i for i, b in enumerate(buttons) if b]
                if pressed_buttons:
                    print(f"   Boutons pressés: {pressed_buttons}")
            
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n🛑 Test interrompu par l'utilisateur")
    
    print("\n3️⃣ Test de l'interface web...")
    try:
        response = requests.get("http://localhost:9001/advanced")
        if response.status_code == 200:
            print("✅ Interface web accessible")
            print("🌐 Ouvrez: http://localhost:9001/advanced")
            print("💡 Vous devriez voir le cercle avec le point qui bouge selon le joystick")
        else:
            print(f"❌ Erreur interface: {response.status_code}")
    except Exception as e:
        print(f"❌ Erreur interface: {e}")
    
    print("\n🎯 Instructions pour tester:")
    print("1. Ouvrez http://localhost:9001/advanced dans votre navigateur")
    print("2. Vous devriez voir un cercle avec un point")
    print("3. Bougez le joystick - le point devrait suivre vos mouvements")
    print("4. Les moteurs devraient bouger en temps réel")
    print("5. Testez les boutons du joystick")

if __name__ == "__main__":
    test_joystick_osc()
