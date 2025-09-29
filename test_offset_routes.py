#!/usr/bin/env python3
"""
Script de test pour les nouvelles routes d'offsets latched
"""

import requests
import json
import time

# Configuration
BASE_URL = "http://localhost:9000"

def test_offset_routes():
    """Test des nouvelles routes d'offsets"""
    print("🧪 Test des routes d'offsets latched")
    print("=" * 50)
    
    # 1. Test du statut
    print("\n1️⃣ Test du statut des offsets...")
    try:
        response = requests.get(f"{BASE_URL}/api/offset/status")
        print(f"✅ Status: {response.status_code}")
        print(f"📄 Response: {response.json()}")
    except Exception as e:
        print(f"❌ Erreur: {e}")
    
    # 2. Test reset des offsets
    print("\n2️⃣ Test reset des offsets...")
    try:
        data = {"pan": True, "tilt": True}
        response = requests.post(f"{BASE_URL}/api/offset/zero", json=data)
        print(f"✅ Status: {response.status_code}")
        print(f"📄 Response: {response.json()}")
    except Exception as e:
        print(f"❌ Erreur: {e}")
    
    # 3. Test ajout d'offsets
    print("\n3️⃣ Test ajout d'offsets...")
    try:
        data = {"d_pan": 100, "d_tilt": 50}
        response = requests.post(f"{BASE_URL}/api/offset/add", json=data)
        print(f"✅ Status: {response.status_code}")
        print(f"📄 Response: {response.json()}")
    except Exception as e:
        print(f"❌ Erreur: {e}")
    
    # 4. Test définition d'offsets
    print("\n4️⃣ Test définition d'offsets...")
    try:
        data = {"pan": 200, "tilt": 150}
        response = requests.post(f"{BASE_URL}/api/offset/set", json=data)
        print(f"✅ Status: {response.status_code}")
        print(f"📄 Response: {response.json()}")
    except Exception as e:
        print(f"❌ Erreur: {e}")
    
    # 5. Test bake des offsets
    print("\n5️⃣ Test bake des offsets...")
    try:
        response = requests.post(f"{BASE_URL}/api/offset/bake")
        print(f"✅ Status: {response.status_code}")
        print(f"📄 Response: {response.json()}")
    except Exception as e:
        print(f"❌ Erreur: {e}")
    
    print("\n🎉 Tests terminés !")

if __name__ == "__main__":
    test_offset_routes()
