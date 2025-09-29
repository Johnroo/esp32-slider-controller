#!/usr/bin/env python3
"""
Test simple de détection de joystick sans pygame
"""

import subprocess
import json
import time

def test_joystick_detection():
    """Test de détection de joystick via système"""
    print("🎮 Test de détection de joystick")
    print("=" * 40)
    
    # 1. Vérifier les périphériques USB
    try:
        result = subprocess.run(['system_profiler', 'SPUSBDataType'], 
                              capture_output=True, text=True, timeout=10)
        if 'Logitech' in result.stdout:
            print("✅ Logitech détecté dans les périphériques USB")
        else:
            print("❌ Logitech non trouvé dans les périphériques USB")
    except Exception as e:
        print(f"❌ Erreur system_profiler: {e}")
    
    # 2. Vérifier les périphériques d'entrée
    try:
        result = subprocess.run(['system_profiler', 'SPHIDDataType'], 
                              capture_output=True, text=True, timeout=10)
        if 'Logitech' in result.stdout:
            print("✅ Logitech détecté dans les périphériques HID")
        else:
            print("❌ Logitech non trouvé dans les périphériques HID")
    except Exception as e:
        print(f"❌ Erreur HID: {e}")
    
    # 3. Test pygame simple
    print("\n🧪 Test pygame simple...")
    try:
        import pygame
        print("✅ pygame disponible")
        
        # Test sans thread
        pygame.init()
        pygame.joystick.init()
        count = pygame.joystick.get_count()
        print(f"📊 Joysticks détectés: {count}")
        
        if count > 0:
            for i in range(count):
                j = pygame.joystick.Joystick(i)
                j.init()
                print(f"🎮 {i}: {j.get_name()}")
        
    except Exception as e:
        print(f"❌ Erreur pygame: {e}")

if __name__ == "__main__":
    test_joystick_detection()
