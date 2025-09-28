# 🎯 Contrôle Direct des Axes - ESP32 Slider

## Nouvelles Fonctionnalités Ajoutées

### **Voies OSC Directes**

J'ai ajouté 4 nouvelles voies OSC pour contrôler directement chaque axe avec des valeurs float de 0.0 à 1.0 :

- **`/axis_pan`** (float 0.0-1.0) : Contrôle direct de l'axe Pan
- **`/axis_tilt`** (float 0.0-1.0) : Contrôle direct de l'axe Tilt  
- **`/axis_zoom`** (float 0.0-1.0) : Contrôle direct de l'axe Zoom
- **`/axis_slide`** (float 0.0-1.0) : Contrôle direct de l'axe Slide

### **Mapping des Valeurs**

Les valeurs float sont automatiquement mappées sur les limites de chaque axe :

| Axe | Limite Min | Limite Max | Mapping |
|-----|------------|------------|---------|
| **Pan** | -12000 | +12000 | 0.0 → -12000, 1.0 → +12000 |
| **Tilt** | -6000 | +6000 | 0.0 → -6000, 1.0 → +6000 |
| **Zoom** | -20000 | +20000 | 0.0 → -20000, 1.0 → +20000 |
| **Slide** | -10000 | +10000 | 0.0 → -10000, 1.0 → +10000 |

### **Interface Web Améliorée**

L'interface Flask a été mise à jour avec :

#### **🎯 Nouveaux Faders Directs**
- **Pan Direct** : Fader 0.0-1.0 avec bouton "Center"
- **Tilt Direct** : Fader 0.0-1.0 avec bouton "Center"  
- **Zoom Direct** : Fader 0.0-1.0 avec bouton "Center"
- **Slide Direct** : Fader 0.0-1.0 avec bouton "Center"

#### **🔄 Boutons de Contrôle**
- **Center** : Remet chaque axe au centre (0.5)
- **Reset All Axes** : Remet tous les axes au centre d'un coup

### **API Routes Ajoutées**

```python
# Nouvelles routes API
/api/axis_pan      # POST: {"value": 0.5}
/api/axis_tilt     # POST: {"value": 0.5}  
/api/axis_zoom     # POST: {"value": 0.5}
/api/axis_slide    # POST: {"value": 0.5}
/api/reset_all_axes # POST: Reset tous les axes
```

## **Utilisation**

### **1. Contrôle Manuel**
- Utilisez les faders "Direct" pour positionner chaque axe précisément
- 0.0 = Position minimale de l'axe
- 1.0 = Position maximale de l'axe  
- 0.5 = Position centrale de l'axe

### **2. Contrôle par OSC**
```python
# Exemples d'utilisation
send_osc_message('/axis_pan', 0.0)    # Pan à gauche
send_osc_message('/axis_pan', 1.0)    # Pan à droite
send_osc_message('/axis_pan', 0.5)    # Pan au centre

send_osc_message('/axis_zoom', 0.0)   # Zoom au minimum
send_osc_message('/axis_zoom', 1.0)   # Zoom au maximum
```

### **3. Test des Nouvelles Fonctionnalités**

```bash
# Test automatique
python test_osc.py

# Test interactif
python test_osc.py
# Puis utilisez: axis_pan, axis_tilt, axis_zoom, axis_slide
```

## **Différence avec les Anciens Contrôles**

| Type | Ancien | Nouveau |
|------|--------|---------|
| **Jog** | `/jog` (-1.0 à 1.0) | Mouvement relatif entre presets |
| **Pan/Tilt** | `/pan`, `/tilt` (-1.0 à 1.0) | Offsets manuels |
| **Axes** | ❌ | **`/axis_*` (0.0 à 1.0)** | **Position absolue** |

## **Avantages du Contrôle Direct**

✅ **Position Absolue** : Chaque valeur correspond à une position précise  
✅ **Reproductible** : Même valeur = même position  
✅ **Intuitif** : 0.0 = début, 1.0 = fin, 0.5 = centre  
✅ **Compatible** : Fonctionne avec tous les logiciels OSC  
✅ **Précis** : Contrôle au pas près  

## **Exemples d'Utilisation**

### **Cinématographie**
```python
# Plan large
send_osc_message('/axis_zoom', 0.0)   # Zoom minimum
send_osc_message('/axis_pan', 0.5)     # Pan centré

# Plan serré  
send_osc_message('/axis_zoom', 1.0)    # Zoom maximum
send_osc_message('/axis_tilt', 0.3)    # Tilt légèrement vers le bas
```

### **Mouvements Programmés**
```python
# Mouvement de gauche à droite
for i in range(11):
    pos = i / 10.0  # 0.0, 0.1, 0.2, ..., 1.0
    send_osc_message('/axis_slide', pos)
    time.sleep(0.5)
```

### **Positions Prédéfinies**
```python
# Positions de tournage
positions = {
    'plan_large': {'pan': 0.5, 'tilt': 0.5, 'zoom': 0.0, 'slide': 0.0},
    'plan_serre': {'pan': 0.5, 'tilt': 0.4, 'zoom': 1.0, 'slide': 0.5},
    'angle_droit': {'pan': 0.0, 'tilt': 0.5, 'zoom': 0.5, 'slide': 1.0}
}
```

## **Interface Web Complète**

L'interface web propose maintenant :

1. **🎮 Jog Control** : Mouvement entre presets
2. **↔️ Pan/Tilt Offsets** : Ajustements fins  
3. **🎯 Direct Axis Control** : Contrôle précis de chaque axe
4. **📍 Preset Management** : Gestion des positions sauvegardées

Tous les contrôles sont synchronisés et offrent un contrôle total de votre slider ESP32 ! 🎬✨

