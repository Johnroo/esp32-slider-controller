#!/usr/bin/env python3
"""
Test direct de pygame pour diagnostiquer le problème
"""

import os
import threading
import time

# Configuration pygame
os.environ.setdefault('SDL_VIDEODRIVER', 'dummy')

def test_pygame_direct():
    """Test direct de pygame"""
    print("🧪 Test direct de pygame")
    print("=" * 40)
    
    try:
        import pygame
        print("✅ pygame importé avec succès")
    except Exception as e:
        print(f"❌ Erreur import pygame: {e}")
        return
    
    try:
        pygame.init()
        print("✅ pygame.init() réussi")
    except Exception as e:
        print(f"❌ Erreur pygame.init(): {e}")
        return
    
    try:
        pygame.joystick.init()
        print("✅ pygame.joystick.init() réussi")
    except Exception as e:
        print(f"❌ Erreur pygame.joystick.init(): {e}")
        return
    
    count = pygame.joystick.get_count()
    print(f"📊 Nombre de joysticks: {count}")
    
    if count == 0:
        print("❌ Aucun joystick détecté")
        return
    
    for i in range(count):
        try:
            j = pygame.joystick.Joystick(i)
            j.init()
            name = j.get_name()
            axes = j.get_numaxes()
            buttons = j.get_numbuttons()
            print(f"🎮 Joystick {i}: {name}")
            print(f"   Axes: {axes}, Buttons: {buttons}")
            
            # Test de lecture
            print("📡 Test de lecture des axes...")
            for _ in range(5):
                try:
                    x = j.get_axis(0) if axes > 0 else 0.0
                    y = j.get_axis(1) if axes > 1 else 0.0
                    print(f"   X: {x:.3f}, Y: {y:.3f}")
                    time.sleep(0.1)
                except Exception as e:
                    print(f"   ❌ Erreur lecture: {e}")
                    break
                    
        except Exception as e:
            print(f"❌ Erreur joystick {i}: {e}")

def test_pygame_thread():
    """Test pygame dans un thread"""
    print("\n🧵 Test pygame dans un thread")
    print("=" * 40)
    
    def worker():
        try:
            import pygame
            pygame.init()
            pygame.joystick.init()
            count = pygame.joystick.get_count()
            print(f"[THREAD] Joysticks détectés: {count}")
            
            if count > 0:
                j = pygame.joystick.Joystick(0)
                j.init()
                print(f"[THREAD] Joystick: {j.get_name()}")
                
                # Lecture continue
                for i in range(10):
                    try:
                        x = j.get_axis(0)
                        y = j.get_axis(1)
                        print(f"[THREAD] X: {x:.3f}, Y: {y:.3f}")
                        time.sleep(0.1)
                    except Exception as e:
                        print(f"[THREAD] Erreur: {e}")
                        break
        except Exception as e:
            print(f"[THREAD] Erreur: {e}")
    
    t = threading.Thread(target=worker, daemon=True)
    t.start()
    t.join(timeout=2)

if __name__ == "__main__":
    test_pygame_direct()
    test_pygame_thread()
