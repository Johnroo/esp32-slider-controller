# Routes d'Offsets Latched - Documentation

## 🎯 Vue d'ensemble

Les nouvelles routes Flask permettent de contrôler les offsets joystick "latched" (persistants) qui s'accumulent pendant les presets au lieu de se remettre à zéro quand on relâche le joystick.

## 🔧 Routes disponibles

### 1. `/api/offset/status` (GET)
**Description**: Obtenir le statut des offsets  
**Méthode**: GET  
**Paramètres**: Aucun  
**Réponse**:
```json
{
  "success": true,
  "message": "Offset status endpoint available",
  "note": "Actual offset values not available via OSC",
  "timestamp": "2025-09-29T15:25:01.222759"
}
```

### 2. `/api/offset/zero` (POST)
**Description**: Remettre les offsets à zéro  
**Méthode**: POST  
**Paramètres**:
```json
{
  "pan": true,    // Reset l'offset pan (défaut: true)
  "tilt": true    // Reset l'offset tilt (défaut: true)
}
```
**Réponse**:
```json
{
  "success": true,
  "pan_reset": true,
  "tilt_reset": true,
  "timestamp": "2025-09-29T15:25:01.225160"
}
```

### 3. `/api/offset/add` (POST)
**Description**: Ajouter des steps aux offsets existants  
**Méthode**: POST  
**Paramètres**:
```json
{
  "d_pan": 100,   // Steps à ajouter au pan
  "d_tilt": 50    // Steps à ajouter au tilt
}
```
**Réponse**:
```json
{
  "success": true,
  "d_pan": 100,
  "d_tilt": 50,
  "timestamp": "2025-09-29T15:25:01.226895"
}
```

### 4. `/api/offset/set` (POST)
**Description**: Définir les offsets absolument  
**Méthode**: POST  
**Paramètres**:
```json
{
  "pan": 200,     // Valeur absolue pour pan
  "tilt": 150     // Valeur absolue pour tilt
}
```
**Réponse**:
```json
{
  "success": true,
  "pan_offset": 200,
  "tilt_offset": 150,
  "timestamp": "2025-09-29T15:25:01.228566"
}
```

### 5. `/api/offset/bake` (POST)
**Description**: Intégrer les offsets dans le preset actuel et les remettre à zéro  
**Méthode**: POST  
**Paramètres**: Aucun  
**Réponse**:
```json
{
  "success": true,
  "message": "Offsets baked into preset and reset",
  "timestamp": "2025-09-29T15:25:01.230381"
}
```

## 🎬 Comportement "S'empile"

### Avant (mapping instantané)
1. Preset: tilt 0 → 1000 en 30s
2. À t=10s: joystick +100 → offset = +100
3. Relâche joystick → offset = 0
4. **Résultat**: axe finit à 1000 ❌

### Maintenant (comportement latched)
1. Preset: tilt 0 → 1000 en 30s
2. À t=10s: joystick +100 → offset s'accumule progressivement
3. Relâche joystick → offset reste à +100
4. **Résultat**: axe finit à 1100 ✅

## 🛠️ Utilisation pratique

### Scénario 1: Correction pendant un preset
```bash
# 1. Lancer un preset
curl -X POST http://localhost:9000/api/preset/recall -d '{"id": 1, "duration": 30}'

# 2. Pendant le preset, ajouter un offset
curl -X POST http://localhost:9000/api/offset/add -d '{"d_tilt": 100}'

# 3. L'offset persiste même après relâchement du joystick
# 4. L'axe termine à la position finale + offset
```

### Scénario 2: Reset manuel
```bash
# Remettre les offsets à zéro
curl -X POST http://localhost:9000/api/offset/zero -d '{"pan": true, "tilt": true}'
```

### Scénario 3: "Figer" une correction
```bash
# Intégrer l'offset dans le preset et le remettre à zéro
curl -X POST http://localhost:9000/api/offset/bake
```

## 🔗 Correspondance OSC

Chaque route Flask envoie des messages OSC correspondants à l'ESP32:

- `/api/offset/zero` → `/offset/zero [pan] [tilt]`
- `/api/offset/add` → `/offset/add [dPan] [dTilt]`
- `/api/offset/set` → `/offset/set [pan] [tilt]`
- `/api/offset/bake` → `/offset/bake`

## 📝 Notes importantes

1. **Intégration automatique**: Les offsets ne s'accumulent que pendant qu'un preset est actif (`sync_move.active = true`)
2. **Limites**: Les offsets sont limités par `PAN_OFFSET_RANGE` et `TILT_OFFSET_RANGE`
3. **Vitesse**: La vitesse d'accumulation est de 30% de la vitesse max de l'axe
4. **Persistance**: Les offsets restent en mémoire jusqu'à reset manuel ou bake

## 🧪 Test

Utilisez le script de test inclus:
```bash
python3 test_offset_routes.py
```
