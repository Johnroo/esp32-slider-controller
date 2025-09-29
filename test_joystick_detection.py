#!/usr/bin/env python3
"""
Test de détection du joystick Logitech Extreme 3D Pro
"""

import os
import time

def test_joystick_detection():
    """Test de détection du joystick"""
    print("🎮 Test de détection du joystick")
    print("=" * 50)
    
    # Configuration SDL pour macOS
    os.environ['SDL_VIDEODRIVER'] = 'dummy'
    os.environ['SDL_AUDIODRIVER'] = 'dummy'
    os.environ['SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS'] = '1'
    os.environ['SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS'] = '1'
    
    print("1️⃣ Configuration SDL...")
    print("✅ Variables d'environnement SDL configurées")
    
    # Délai pour éviter les conflits
    print("2️⃣ Attente de stabilisation...")
    time.sleep(2)
    
    try:
        print("3️⃣ Import de pygame...")
        import pygame
        print("✅ pygame importé avec succès")
    except Exception as e:
        print(f"❌ Erreur import pygame: {e}")
        return False
    
    try:
        print("4️⃣ Initialisation pygame...")
        pygame.init()
        print("✅ pygame.init() OK")
        
        pygame.joystick.init()
        print("✅ pygame.joystick.init() OK")
    except Exception as e:
        print(f"❌ Erreur initialisation: {e}")
        return False
    
    # Délai pour la stabilisation
    time.sleep(1)
    
    try:
        count = pygame.joystick.get_count()
        print(f"5️⃣ Nombre de joysticks détectés: {count}")
    except Exception as e:
        print(f"❌ Erreur get_count: {e}")
        return False
    
    if count == 0:
        print("❌ Aucun joystick détecté")
        print("💡 Vérifiez que le joystick est branché et reconnu par le système")
        return False
    
    print("6️⃣ Détection des joysticks...")
    js = None
    for i in range(count):
        try:
            j = pygame.joystick.Joystick(i)
            j.init()
            name = j.get_name() or ""
            print(f"   Joystick {i}: {name}")
            
            # Chercher le Logitech Extreme 3D
            if "Logitech" in name and "Extreme" in name:
                js = j
                print(f"✅ Logitech Extreme 3D Pro trouvé: {name}")
            elif js is None:
                js = j
                print(f"⚠️  Utilisation du premier joystick: {name}")
        except Exception as e:
            print(f"❌ Erreur joystick {i}: {e}")
            continue
    
    if js is None:
        print("❌ Aucun joystick utilisable")
        return False
    
    try:
        print(f"7️⃣ Test de lecture du joystick...")
        print(f"   Nom: {js.get_name()}")
        print(f"   Axes: {js.get_numaxes()}")
        print(f"   Boutons: {js.get_numbuttons()}")
        
        # Test de lecture des axes
        for i in range(min(4, js.get_numaxes())):
            try:
                value = js.get_axis(i)
                print(f"   Axe {i}: {value:.3f}")
            except Exception as e:
                print(f"   Axe {i}: Erreur - {e}")
        
        # Test de lecture des boutons
        for i in range(min(4, js.get_numbuttons())):
            try:
                pressed = js.get_button(i)
                print(f"   Bouton {i}: {'Pressé' if pressed else 'Relâché'}")
            except Exception as e:
                print(f"   Bouton {i}: Erreur - {e}")
        
        print("✅ Lecture du joystick fonctionne")
        return True
        
    except Exception as e:
        print(f"❌ Erreur lecture: {e}")
        return False

if __name__ == "__main__":
    success = test_joystick_detection()
    
    if success:
        print("\n🎉 Joystick détecté et fonctionnel !")
        print("💡 Vous pouvez maintenant lancer le serveur Flask")
    else:
        print("\n❌ Problème de détection du joystick")
        print("💡 Solutions possibles:")
        print("   - Vérifiez que le joystick est branché")
        print("   - Testez avec un autre port USB")
        print("   - Redémarrez le système")
        print("   - Vérifiez les permissions macOS")
