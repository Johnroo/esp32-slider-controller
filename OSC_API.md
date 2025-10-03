# ESP32 Slider Controller - API OSC

## Vue d'ensemble

Ce document liste toutes les adresses OSC supportées par l'ESP32 Slider Controller après refactorisation. L'ESP32 écoute sur le port **8000** par défaut.

## Table des matières

- [Contrôle des axes](#contrôle-des-axes)
- [Joystick et jog](#joystick-et-jog)
- [Presets](#presets)
- [Interpolation multi-presets](#interpolation-multi-presets)
- [Gestion des banques](#gestion-des-banques)
- [Configuration des moteurs](#configuration-des-moteurs)
- [Homing et StallGuard](#homing-et-stallguard)
- [Offsets](#offsets)
- [Configuration](#configuration)

---

## Contrôle des axes

| Adresse | Arguments | Description |
|---------|-----------|-------------|
| `/axis_pan` | `float` (0.0-1.0) | Position absolue Pan (0=min, 1=max) |
| `/axis_tilt` | `float` (0.0-1.0) | Position absolue Tilt (0=min, 1=max) |
| `/axis_zoom` | `float` (0.0-1.0) | Position absolue Zoom (0=min, 1=max) |
| `/axis_slide` | `float` (0.0-1.0) | Position absolue Slide (0=min, 1=max) |

**Comportement :** Annule automatiquement les mouvements synchronisés en cours si la politique d'annulation l'autorise.

---

## Joystick et jog

| Adresse | Arguments | Description |
|---------|-----------|-------------|
| `/pan` | `float` (-1.0 à +1.0) | Jog Pan en continu (vitesse relative) |
| `/tilt` | `float` (-1.0 à +1.0) | Jog Tilt en continu (vitesse relative) |
| `/joy/pt` | `float pan, float tilt` | Jog Pan et Tilt simultanés |
| `/slide/jog` | `float` (-1.0 à +1.0) | Jog Slide en continu (vitesse relative) |
| `/joy/config` | `float deadzone, float expo, float slew_per_s, float filt_hz, float slide_speed` | Configuration du joystick |

**Configuration joystick :**
- `deadzone` : Zone morte (0.0-0.5)
- `expo` : Courbure exponentielle (0.0-0.95)
- `slew_per_s` : Vitesse de slew (steps/s)
- `filt_hz` : Fréquence de filtrage (Hz)
- `slide_speed` : Vitesse du slide (0.1-3.0x)

---

## Presets

| Adresse | Arguments | Description |
|---------|-----------|-------------|
| `/preset/set` | `int index, int pan, int tilt, int zoom, int slide` | Définit un preset avec positions absolues |
| `/preset/store` | `int index` | Capture les positions actuelles et les sauvegarde dans le preset |
| `/preset/recall` | `int index, float duration_sec` | Rappel d'un preset avec durée spécifiée |
| `/preset/recall_policy` | `int policy` | Politique de recall slide (0=KEEP_AB, 1=GOTO_THEN_RESUME) |
| `/preset/cancel_policy` | `int by_joystick, int by_axis` | Politique d'annulation (0/1 pour chaque) |

**Politiques :**
- **KEEP_AB** : Conserve la position slide actuelle
- **GOTO_THEN_RESUME** : Va à la position slide du preset puis reprend le jog

---

## Interpolation multi-presets

| Adresse | Arguments | Description |
|---------|-----------|-------------|
| `/interp/setpoints` | `int N, int preset1, float fraction1, int preset2, float fraction2, ...` | Définit les points d'interpolation |
| `/interp/auto` | `int enable, float duration_sec` | Active l'interpolation automatique |
| `/interp/goto` | `float fraction` | Va à une position interpolée (0.0-1.0) |
| `/interp/jog` | `float speed` | Jog de l'axe d'interpolation (-1.0 à +1.0) |

**Exemple setpoints :**
```
/interp/setpoints 3 0 0.0 1 0.5 2 1.0
```
Définit 3 points : preset 0 à 0%, preset 1 à 50%, preset 2 à 100%.

---

## Gestion des banques

| Adresse | Arguments | Description |
|---------|-----------|-------------|
| `/bank/set` | `int bank_index` | Change la banque active (0-9) |
| `/bank/save` | Aucun | Sauvegarde la banque active |
| `/bank/get_interp` | Aucun | Retourne les points d'interpolation actuels (JSON) |

**Comportement :** Le changement de banque recharge automatiquement les points d'interpolation.

---

## Configuration des moteurs

| Adresse | Arguments | Description |
|---------|-----------|-------------|
| `/motor/pan/max_speed` | `int speed` | Vitesse max Pan (2000-20000 steps/s) |
| `/motor/pan/max_accel` | `int accel` | Accélération max Pan (1000-999999 steps/s²) |
| `/motor/tilt/max_speed` | `int speed` | Vitesse max Tilt (2000-20000 steps/s) |
| `/motor/tilt/max_accel` | `int accel` | Accélération max Tilt (1000-999999 steps/s²) |
| `/motor/zoom/max_speed` | `int speed` | Vitesse max Zoom (2000-20000 steps/s) |
| `/motor/zoom/max_accel` | `int accel` | Accélération max Zoom (1000-999999 steps/s²) |
| `/motor/slide/max_speed` | `int speed` | Vitesse max Slide (2000-20000 steps/s) |
| `/motor/slide/max_accel` | `int accel` | Accélération max Slide (1000-999999 steps/s²) |

---

## Homing et StallGuard

| Adresse | Arguments | Description |
|---------|-----------|-------------|
| `/slide/home` | Aucun | Lance le homing du slide (StallGuard) |
| `/slide/sgthrs` | `int threshold` | Seuil StallGuard (0-255) |
| `/slide/goto` | `float position, float duration_sec` | Déplacement slide à position relative (0.0-1.0) |

---

## Offsets

| Adresse | Arguments | Description |
|---------|-----------|-------------|
| `/offset/zero` | `int do_pan, int do_tilt` | Remet les offsets à zéro |
| `/offset/add` | `int pan_offset, int tilt_offset` | Ajoute aux offsets actuels |
| `/offset/set` | `int pan_offset, int tilt_offset` | Définit les offsets absolus |
| `/offset/bake` | Aucun | Intègre les offsets dans le mouvement en cours |

**Comportement :** Les offsets sont "latched" (verrouillés) et persistent jusqu'à reset.

---

## Configuration

| Adresse | Arguments | Description |
|---------|-----------|-------------|
| `/config/offset_range` | `int min_range, int max_range` | Définit la plage des offsets |

---

## Notes importantes

### Annulation automatique
- Les mouvements synchronisés sont automatiquement annulés lors de l'utilisation du joystick ou des axes directs
- Configurable via `/preset/cancel_policy`

### Interpolation
- L'interpolation multi-presets permet de créer des mouvements fluides entre plusieurs positions
- Support jusqu'à 6 points d'interpolation
- Mode automatique et manuel disponibles

### Homing
- Le homing du slide utilise le StallGuard des drivers TMC
- Seuil ajustable en temps réel
- Un seul homing à la fois autorisé

### Gestion des erreurs
- Toutes les valeurs sont clampées dans leurs plages valides
- Les commandes invalides sont ignorées silencieusement
- Logs détaillés disponibles sur le port série

---

## Exemples d'utilisation

### Mouvement simple
```
/axis_pan 0.5        # Pan à 50%
/axis_tilt 0.8        # Tilt à 80%
```

### Jog continu
```
/pan 0.3              # Jog Pan à 30% de vitesse
/slide/jog -0.5       # Jog Slide à -50% de vitesse
```

### Preset
```
/preset/store 0       # Capturer position actuelle
/preset/recall 0 3.0  # Rappel preset 0 en 3 secondes
```

### Interpolation
```
/interp/setpoints 3 0 0.0 1 0.5 2 1.0
/interp/auto 1 10.0   # Auto 10 secondes
```

### Configuration moteur
```
/motor/pan/max_speed 5000
/motor/slide/max_accel 2000
```
