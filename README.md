# ESP32 Slider Controller

Un système de contrôle de slider motorisé avec ESP32, interface web Flask et communication OSC.

## 🚀 Fonctionnalités

- **4 axes motorisés** : Pan, Tilt, Zoom, Slide
- **Drivers TMC2209** avec contrôle de courant et microstepping
- **Interface web Flask** pour contrôle à distance
- **Communication OSC** en temps réel
- **WiFiManager** pour configuration réseau automatique
- **OTA Updates** pour mises à jour sans fil
- **Console web** pour monitoring et debug

## 🔧 Matériel

- ESP32 DevKit
- 4x Drivers TMC2209
- 4x Moteurs pas-à-pas
- Alimentation 24V

## 📡 Brochage

### Moteurs
- **Pan** : STEP=18, DIR=19, EN=13
- **Tilt** : STEP=21, DIR=22, EN=14  
- **Zoom** : STEP=23, DIR=25, EN=32
- **Slide** : STEP=26, DIR=27, EN=33

### TMC2209 UART
- **TX** : GPIO 17
- **RX** : GPIO 16

## 🛠️ Installation

### ESP32 (PlatformIO)

```bash
# Cloner le repository
git clone <repo-url>
cd esp32-slider-controller

# Compiler et uploader
pio run --target upload

# Monitor série
pio device monitor
```

### Interface Web (Python)

```bash
# Installer les dépendances
pip install -r requirements.txt

# Démarrer le serveur
python start_controller.py
```

## 🌐 Utilisation

1. **Premier démarrage** : L'ESP32 crée un AP "SliderAP"
2. **Configuration WiFi** : Se connecter à l'AP et configurer le WiFi
3. **Interface web** : Ouvrir http://localhost:9000
4. **Contrôle** : Utiliser les faders pour contrôler les moteurs

## 📱 Interface Web

- **Faders temps réel** pour chaque axe
- **Presets** A/B pour positions prédéfinies
- **Jog** pour déplacement continu
- **Monitoring** des positions et statuts

## 📡 Communication OSC

### Messages reçus (ESP32)
- `/axis_pan` : Position Pan (0.0-1.0)
- `/axis_tilt` : Position Tilt (0.0-1.0)
- `/axis_zoom` : Position Zoom (0.0-1.0)
- `/axis_slide` : Position Slide (0.0-1.0)
- `/jog` : Vitesse jog (-1.0 à +1.0)
- `/pan` : Offset Pan (-1.0 à +1.0)
- `/tilt` : Offset Tilt (-1.0 à +1.0)

### Configuration
- **Port OSC** : 8000
- **IP ESP32** : Configurée via WiFiManager

## 🔧 Configuration

### Limites des axes
```cpp
AxisConfig cfg[NUM_MOTORS] = {
  {-10000, 10000, 800, 16, 20000, 8000, 0, false, true, false},  // Pan
  {-10000, 10000, 800, 16, 20000, 8000, 0, false, true, false},  // Tilt
  {-20000, 20000, 200, 4,  20000, 8000, 0, false, false, false}, // Zoom
  {-10000, 10000, 800, 16, 20000, 8000, 0, false, true, false}   // Slide
};
```

### TMC2209
- **Adresses** : Pan=0, Tilt=1, Zoom=2, Slide=3
- **R_SENSE** : 0.11Ω
- **Microstepping** : Configurable par axe
- **Courant** : Ajustable par axe

## 🐛 Debug

### Console série
- **Baudrate** : 115200
- **Logs détaillés** : Positions, OSC, drivers TMC

### Console web
- **URL** : http://[ESP32_IP]/
- **WebSocket** : Monitoring temps réel

## 📦 Dépendances

### ESP32
- FastAccelStepper
- TMCStepper  
- OSC (CNMAT)
- WiFiManager
- ESPAsyncWebServer
- ArduinoOTA

### Python
- Flask
- python-osc
- requests

## 🚀 Développement

### Structure du projet
```
├── src/
│   └── main.cpp          # Code principal ESP32
├── templates/
│   └── index.html        # Interface web
├── slider_controller.py  # Serveur Flask
├── start_controller.py   # Script de démarrage
├── config.py            # Configuration
└── platformio.ini       # Configuration PlatformIO
```

### Ajout de fonctionnalités
1. Modifier `src/main.cpp` pour l'ESP32
2. Modifier `slider_controller.py` pour l'interface
3. Tester avec `python start_controller.py`
4. Uploader avec `pio run --target upload`

## 📄 Licence

MIT License - Voir LICENSE pour plus de détails.

## 🤝 Contribution

Les contributions sont les bienvenues ! N'hésitez pas à ouvrir une issue ou une pull request.

## 📞 Support

Pour toute question ou problème, ouvrez une issue sur GitHub.
