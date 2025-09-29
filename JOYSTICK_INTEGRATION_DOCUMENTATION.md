# Intégration Joystick Logitech Extreme 3D Pro - Documentation

## 🎯 Vue d'ensemble

Le système Flask a été étendu pour lire le joystick Logitech Extreme 3D Pro via pygame et afficher une interface visuelle avec un cercle et un point en temps réel, remplaçant les faders traditionnels.

## 🔧 Fonctionnalités implémentées

### 1. **Backend Flask (slider_controller.py)**

#### Configuration pygame
```python
JOYSTICK_USE_PYGAME = True
JOYSTICK_NAME_HINT = "Logitech Extreme 3D"
JOYSTICK_DEADZONE   = 0.08      # zone morte
JOYSTICK_EXPO       = 0.35      # courbe exponentielle
JOYSTICK_RATE_HZ    = 120       # fréquence de polling
JOYSTICK_SEND_EPS   = 0.01      # seuil de variation
JOYSTICK_SEND_HZ    = 60        # max envois OSC/s
```

#### État global du joystick
```python
joystick_state = {
    'connected': False,
    'name': None,
    'x': 0.0,     # -1..+1 (pan)
    'y': 0.0,     # -1..+1 (tilt)
    'z': 0.0,     # twist
    'throttle': 0.0,
    'buttons': [],
    'timestamp': 0.0
}
```

#### Worker thread pygame
- **Lecture continue** : 120 Hz de polling
- **Traitement des axes** : deadzone + expo + lissage
- **Envoi OSC** : `/joy/pt` vers l'ESP32 (throttlé à 60 Hz)
- **Détection automatique** : recherche du joystick par nom

#### API REST
- **`/api/joystick/state`** (GET) : État en temps réel du joystick

### 2. **Frontend HTML (advanced.html)**

#### Interface visuelle
- **Canvas 180x180px** : Cercle avec croix centrale
- **Point animé** : Position en temps réel (30 FPS)
- **Affichage des valeurs** : X et Y en temps réel
- **Statut de connexion** : Indication visuelle

#### Fallback automatique
- **Faders cachés** : Quand joystick connecté
- **Faders visibles** : Quand joystick déconnecté
- **Transition fluide** : CSS avec animations

#### CSS ajouté
```css
.joy-flex { display: flex; gap: 24px; align-items: center; }
.joy-visual { display: flex; flex-direction: column; align-items: center; }
#joyCanvas { background: #f8f9fa; border-radius: 12px; }
body[data-joy="connected"] #sliderJoystickGroup { display: none; }
```

#### JavaScript ajouté
```javascript
// Dessin du cercle et point
function drawJoy(x, y) { /* ... */ }

// Polling de l'état (30 FPS)
async function pollJoystick() { /* ... */ }
```

## 🎮 Comportement obtenu

### Avec joystick connecté
1. **Détection automatique** : Le joystick est détecté au démarrage
2. **Interface visuelle** : Cercle avec point qui suit les mouvements
3. **Envoi OSC** : `/joy/pt` envoyé vers l'ESP32 en temps réel
4. **Faders masqués** : Interface épurée

### Sans joystick
1. **Faders visibles** : Contrôle manuel via sliders
2. **Message d'information** : "Aucun joystick détecté"
3. **Fonctionnalité complète** : Toutes les fonctions restent disponibles

## 🔄 Intégration avec les offsets latched

Le joystick envoie des commandes `/joy/pt` qui sont traitées par l'ESP32 avec le système d'offsets latched :

1. **Pendant un preset** : Les mouvements du joystick s'accumulent dans `pan_offset_latched` et `tilt_offset_latched`
2. **Persistance** : Les offsets restent même après relâchement du joystick
3. **Comportement "s'empile"** : Les corrections s'ajoutent à la trajectoire de base

## 🛠️ Utilisation

### 1. **Connexion du joystick**
```bash
# Connectez votre Logitech Extreme 3D Pro via USB
# Le système le détectera automatiquement
```

### 2. **Interface web**
```bash
# Ouvrez http://localhost:9000/advanced
# Vous verrez le cercle joystick si connecté
```

### 3. **Test de l'API**
```bash
# Vérifier l'état du joystick
curl http://localhost:9000/api/joystick/state
```

## 📊 Flux de données

```
Logitech 3D Pro → pygame → Flask → OSC → ESP32
                     ↓
                Interface web (cercle animé)
```

## 🧪 Tests

### Script de test inclus
```bash
python3 test_joystick_integration.py
```

### Tests manuels
1. **Connexion** : Brancher/débrancher le joystick
2. **Mouvement** : Bouger le stick et observer le point
3. **Preset** : Lancer un preset et corriger avec le joystick
4. **Fallback** : Vérifier que les faders réapparaissent sans joystick

## ⚙️ Configuration avancée

### Ajuster la sensibilité
```python
JOYSTICK_DEADZONE = 0.05    # Plus sensible
JOYSTICK_EXPO = 0.2         # Plus linéaire
```

### Changer le joystick
```python
JOYSTICK_NAME_HINT = "Votre Joystick"
```

### Ajuster les fréquences
```python
JOYSTICK_RATE_HZ = 60       # Moins de CPU
JOYSTICK_SEND_HZ = 30       # Moins de trafic réseau
```

## 🎯 Avantages

1. **Interface intuitive** : Cercle visuel plus clair que des faders
2. **Feedback temps réel** : Position visible instantanément
3. **Fallback robuste** : Faders en secours
4. **Intégration parfaite** : Compatible avec les offsets latched
5. **Performance** : Thread séparé, pas de blocage

## 🚀 Prochaines étapes

- **Mapping des boutons** : Utiliser les boutons du joystick
- **Profils** : Sauvegarder différentes configurations
- **Calibration** : Interface de calibrage du joystick
- **Multi-joystick** : Support de plusieurs joysticks
