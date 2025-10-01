#!/usr/bin/env python3
"""
ESP32 Slider Controller - Version améliorée avec reconnexion automatique du joystick
"""

import os
import time
import threading
import math
from flask import Flask, render_template, request, jsonify
from pythonosc import udp_client
from config import ESP32_IP, ESP32_OSC_PORT, FLASK_PORT

# Configuration Flask
app = Flask(__name__)

# Configuration OSC
osc_client = udp_client.SimpleUDPClient(ESP32_IP, ESP32_OSC_PORT)

# --- Joystick (pygame) config ---
JOYSTICK_USE_PYGAME = True
JOYSTICK_NAME_HINT = "Logitech Extreme 3D"
JOYSTICK_DEADZONE = 0.08  # zone morte
JOYSTICK_EXPO = 0.35      # 0..1 (0 = linéaire, 0.35 = doux, 0.7 = très expo)
JOYSTICK_RATE_HZ = 120    # fréquence de polling
JOYSTICK_SEND_EPS = 0.01  # variation minimale pour renvoyer
JOYSTICK_SEND_HZ = 60     # max envois OSC/s (throttle)

joystick_state = {
    'connected': False,
    'name': None,
    'x': 0.0,  # -1..+1 (pan)
    'y': 0.0,  # -1..+1 (tilt, positif = stick vers le haut)
    'z': 0.0,
    'throttle': 0.0,
    'buttons': [],
    'timestamp': 0.0
}

# Signal de rescan (hot reconnect manuel via API)
JOY_RESCAN = threading.Event()

def _expo(v: float, expo: float) -> float:
    """mix linéaire/cubique : classique RC"""
    return (1.0 - expo) * v + expo * (v ** 3)

def _apply_deadzone(v: float, dz: float) -> float:
    return 0.0 if abs(v) < dz else v

def send_osc_message(address, *args):
    """Envoie un message OSC vers l'ESP32"""
    try:
        osc_client.send_message(address, list(args) if args else [])
        # Log détaillé de tous les messages OSC
        args_str = ', '.join([f'{arg:.3f}' if isinstance(arg, float) else str(arg) for arg in args])
        print(f"[OSC] 📤 {address} [{args_str}]")
        return True
    except Exception as e:
        print(f"[OSC] ❌ Erreur envoi {address}: {e}")
        return False

def _start_joystick_thread():
    """Démarre le thread joystick"""
    if not JOYSTICK_USE_PYGAME:
        return
    t = threading.Thread(target=_joystick_worker, name="pygame-joystick", daemon=True)
    t.start()

def _joystick_worker():
    """Worker thread pour lire le joystick en continu avec reconnexion automatique"""
    global joystick_state
    
    # Configuration SDL pour macOS - approche plus conservative
    os.environ['SDL_VIDEODRIVER'] = 'dummy'
    os.environ['SDL_AUDIODRIVER'] = 'dummy'
    os.environ['SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS'] = '1'
    os.environ['SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS'] = '1'
    
    # Délai pour éviter les conflits
    time.sleep(2)
    
    try:
        import pygame
        print("[JOYSTICK] pygame importé avec succès")
    except Exception as e:
        print(f"[JOYSTICK] pygame indisponible: {e}")
        return

    # Initialisation pygame
    try:
        print("[JOYSTICK] Initialisation pygame...")
        pygame.init()
        print("[JOYSTICK] pygame.init() OK")
        pygame.joystick.init()
        print("[JOYSTICK] pygame.joystick.init() OK")
    except Exception as e:
        print(f"[JOYSTICK] Erreur initialisation: {e}")
        return
    
    js = None
    last_joystick_count = 0
    connection_retry_count = 0
    max_retries = 5
    
    while True:
        try:
            # Demande manuelle de rescan depuis l'API
            if JOY_RESCAN.is_set():
                try:
                    pygame.joystick.quit()
                    time.sleep(0.2)
                    pygame.joystick.init()
                    print("[JOYSTICK] Rescan demandé: sous-système joystick réinitialisé")
                except Exception as e:
                    print(f"[JOYSTICK] Erreur rescan: {e}")
                js = None
                joystick_state['connected'] = False
                joystick_state['name'] = None
                JOY_RESCAN.clear()
                time.sleep(0.2)
            current_count = pygame.joystick.get_count()
            
            # Détection de changement de joystick
            if current_count != last_joystick_count:
                print(f"[JOYSTICK] Changement détecté: {last_joystick_count} -> {current_count} joysticks")
                # Réinitialise le sous-système joystick pour gérer les hotplug proprement
                try:
                    pygame.joystick.quit()
                    time.sleep(0.2)
                    pygame.joystick.init()
                    print("[JOYSTICK] Sous-système joystick réinitialisé")
                except Exception as e:
                    print(f"[JOYSTICK] Erreur réinit joystick: {e}")
                js = None
                last_joystick_count = current_count
                connection_retry_count = 0
                joystick_state['connected'] = False
                joystick_state['name'] = None
            
            # Sélection du joystick si pas encore fait
            if js is None and current_count > 0:
                print(f"[JOYSTICK] Recherche de joystick parmi {current_count} dispositifs...")
                for i in range(current_count):
                    try:
                        j = pygame.joystick.Joystick(i)
                        j.init()
                        name = j.get_name() or ""
                        print(f"[JOYSTICK] Joystick {i}: {name}")
                        if JOYSTICK_NAME_HINT.lower() in name.lower() or js is None:
                            js = j
                            print(f"[JOYSTICK] Sélectionné: {name}")
                            break
                    except Exception as e:
                        print(f"[JOYSTICK] Erreur joystick {i}: {e}")
                        continue
                
                if js is None:
                    print("[JOYSTICK] Aucun joystick sélectionné")
                    joystick_state['connected'] = False
                    joystick_state['name'] = None
                    time.sleep(1)
                    continue
                
                try:
                    print(f"[JOYSTICK] Connecté: {js.get_name()}")
                    joystick_state['connected'] = True
                    joystick_state['name'] = js.get_name()
                    connection_retry_count = 0
                except Exception as e:
                    print(f"[JOYSTICK] Erreur connexion: {e}")
                    js = None
                    connection_retry_count += 1
                    if connection_retry_count >= max_retries:
                        print("[JOYSTICK] Trop d'erreurs, attente avant retry...")
                        time.sleep(5)
                        connection_retry_count = 0
                    continue
            
            # Si pas de joystick, attendre
            if js is None:
                joystick_state['connected'] = False
                joystick_state['name'] = None
                time.sleep(1)
                continue
            
            # Vérifier que le joystick est toujours valide
            try:
                js.get_name()  # Test de connexion
            except Exception:
                print("[JOYSTICK] Joystick déconnecté, recherche d'un nouveau...")
                js = None
                joystick_state['connected'] = False
                joystick_state['name'] = None
                continue
            
            # Traitement normal du joystick
            clock = pygame.time.Clock()
            last_send = time.time()
            last_x, last_y = 0.0, 0.0
            ema_x = 0.0
            ema_y = 0.0
            alpha = 0.2  # lissage léger
            
            while js is not None:
                # vider la queue d'événements
                for _ in pygame.event.get():
                    pass
                
                try:
                    x = js.get_axis(0) if js.get_numaxes() > 0 else 0.0     # X
                    y = js.get_axis(1) if js.get_numaxes() > 1 else 0.0     # Y
                    z = js.get_axis(2) if js.get_numaxes() > 2 else 0.0     # twist
                    thr = js.get_axis(3) if js.get_numaxes() > 3 else 0.0   # throttle
                except Exception:
                    print("[JOYSTICK] Erreur lecture axes, reconnexion...")
                    js = None
                    break
                
                # deadzone + expo + lissage ; inverser Y pour "stick en haut = +"
                x = _apply_deadzone(x, JOYSTICK_DEADZONE)
                y = _apply_deadzone(y, JOYSTICK_DEADZONE)
                x = _expo(x, JOYSTICK_EXPO)
                y = _expo(y, JOYSTICK_EXPO)
                ema_x = (1 - alpha) * ema_x + alpha * x
                ema_y = (1 - alpha) * ema_y + alpha * (-y)  # invert Y
                
                # état pour l'UI
                try:
                    buttons = [bool(js.get_button(i)) for i in range(js.get_numbuttons())]
                except Exception:
                    buttons = []
                
                now = time.time()
                joystick_state.update({
                    'x': float(max(-1.0, min(1.0, ema_x))),
                    'y': float(max(-1.0, min(1.0, ema_y))),
                    'z': float(z),
                    'throttle': float(thr),
                    'buttons': buttons,
                    'timestamp': now
                })
                
                # Envoi OSC throttlé
                send_due = (now - last_send) >= (1.0 / JOYSTICK_SEND_HZ)
                moved_enough = (abs(ema_x - last_x) > JOYSTICK_SEND_EPS) or (abs(ema_y - last_y) > JOYSTICK_SEND_EPS)
                
                if send_due and moved_enough:
                    # /joy/pt : pan, tilt dans [-1..1]
                    send_osc_message('/joy/pt', float(joystick_state['x']), float(joystick_state['y']))
                    last_send = now
                    last_x, last_y = ema_x, ema_y
                
                clock.tick(JOYSTICK_RATE_HZ)
                
                # Vérifier périodiquement si le joystick est toujours connecté
                if pygame.joystick.get_count() == 0:
                    print("[JOYSTICK] Plus de joysticks détectés")
                    js = None
                    break
                    
        except Exception as e:
            print(f"[JOYSTICK] Erreur générale: {e}")
            js = None
            joystick_state['connected'] = False
            joystick_state['name'] = None
            time.sleep(1)

# Routes Flask
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/advanced')
def advanced():
    return render_template('advanced.html')

# ==================== HOMING (Slide, StallGuard) ====================
@app.route('/api/slide/home', methods=['POST'])
def api_slide_home():
    """Déclenche le homing du slide côté ESP32"""
    try:
        send_osc_message('/slide/home')
        return jsonify({"status": "ok"})
    except Exception as e:
        return jsonify({"status": "error", "error": str(e)}), 500

@app.route('/api/slide/sgthrs', methods=['POST'])
def api_slide_sgthrs():
    """Met à jour la sensibilité StallGuard (0..255)"""
    data = request.get_json(force=True, silent=True) or {}
    try:
        val = int(data.get('value', 8))
    except Exception:
        val = 8
    if val < 0:
        val = 0
    if val > 255:
        val = 255
    try:
        send_osc_message('/slide/sgthrs', val)
        return jsonify({"status": "ok", "value": val})
    except Exception as e:
        return jsonify({"status": "error", "error": str(e)}), 500

@app.route('/api/joystick/state', methods=['GET'])
def api_joystick_state():
    """Retourne l'état actuel du joystick"""
    return jsonify(joystick_state)

# Routes OSC pour offsets
@app.route('/api/offset/zero', methods=['POST'])
def api_offset_zero():
    """Reset des offsets latched"""
    data = request.get_json()
    do_pan = data.get('pan', True)
    do_tilt = data.get('tilt', True)
    send_osc_message('/offset/zero', 1 if do_pan else 0, 1 if do_tilt else 0)
    return "ok"

@app.route('/api/offset/add', methods=['POST'])
def api_offset_add():
    """Ajoute des offsets"""
    data = request.get_json()
    pan = data.get('pan', 0)
    tilt = data.get('tilt', 0)
    send_osc_message('/offset/add', pan, tilt)
    return "ok"

@app.route('/api/offset/set', methods=['POST'])
def api_offset_set():
    """Définit des offsets absolus"""
    data = request.get_json()
    pan = data.get('pan', 0)
    tilt = data.get('tilt', 0)
    send_osc_message('/offset/set', pan, tilt)
    return "ok"

@app.route('/api/offset/bake', methods=['POST'])
def api_offset_bake():
    """Intègre les offsets dans le preset actuel"""
    send_osc_message('/offset/bake')
    return "ok"

@app.route('/api/offset/status', methods=['GET'])
def api_offset_status():
    """Retourne le statut des offsets (simulation)"""
    return jsonify({
        'pan_latched': 0,
        'tilt_latched': 0,
        'pan_steps': 0,
        'tilt_steps': 0
    })

@app.post('/api/joystick/rescan')
def api_joystick_rescan():
    """Force un rescan/reinit du sous-système joystick (hot reconnect)."""
    JOY_RESCAN.set()
    return "ok"

# Routes pour presets
@app.route('/api/preset/recall', methods=['POST'])
def api_preset_recall():
    """Rappelle un preset"""
    data = request.get_json()
    preset_id = int(data.get('id', 0))
    duration = float(data.get('duration', 2.0))
    send_osc_message('/preset/recall', preset_id, duration)
    return "ok"

@app.route('/api/preset/store', methods=['POST'])
def api_preset_store():
    """Stocke un preset avec positions réelles"""
    data = request.get_json()
    preset_id = data.get('id', 0)
    send_osc_message('/preset/store', preset_id)
    return "ok"

@app.route('/api/preset/mode', methods=['POST'])
def api_preset_mode():
    """Définit le mode d'un preset"""
    data = request.get_json()
    preset_id = data.get('id', 0)
    mode = data.get('mode', 'abs')
    mode_code = 1 if mode == 'follow' else 0
    send_osc_message('/preset/mode', preset_id, mode_code)
    return "ok"

@app.route('/api/preset/recall_policy', methods=['POST'])
def api_preset_recall_policy():
    """Définit la politique de recall"""
    data = request.get_json()
    slide_policy = data.get('slide', 0)
    send_osc_message('/preset/recall_policy', slide_policy)
    return "ok"

# Routes pour les axes individuels
@app.route('/api/axis_slide', methods=['POST'])
def api_axis_slide():
    """Commande directe pour l'axe slide"""
    data = request.get_json()
    value = float(data.get('value', 0.5))  # Force float
    send_osc_message('/axis_slide', value)
    return "ok"

@app.route('/api/axis_pan', methods=['POST'])
def api_axis_pan():
    """Commande directe pour l'axe pan"""
    data = request.get_json()
    value = float(data.get('value', 0.5))  # Force float
    send_osc_message('/axis_pan', value)
    return "ok"

@app.route('/api/axis_tilt', methods=['POST'])
def api_axis_tilt():
    """Commande directe pour l'axe tilt"""
    data = request.get_json()
    value = float(data.get('value', 0.5))  # Force float
    send_osc_message('/axis_tilt', value)
    return "ok"

@app.route('/api/axis_zoom', methods=['POST'])
def api_axis_zoom():
    """Commande directe pour l'axe zoom"""
    data = request.get_json()
    value = float(data.get('value', 0.5))  # Force float
    send_osc_message('/axis_zoom', value)
    return "ok"

# Routes pour joystick offsets (sliders pan/tilt sans joystick physique)
@app.route('/api/joystick/pan', methods=['POST'])
def api_joystick_pan():
    """Envoie un offset pan via le slider"""
    data = request.get_json()
    value = float(data.get('value', 0.0))  # -1.0 to 1.0
    value = max(-1.0, min(1.0, value))
    
    success = send_osc_message('/pan', value)
    return jsonify({'success': True, 'value': value})

@app.route('/api/joystick/tilt', methods=['POST'])
def api_joystick_tilt():
    """Envoie un offset tilt via le slider"""
    data = request.get_json()
    value = float(data.get('value', 0.0))  # -1.0 to 1.0
    value = max(-1.0, min(1.0, value))
    
    success = send_osc_message('/tilt', value)
    return jsonify({'success': True, 'value': value})

@app.route('/api/joystick/combined', methods=['POST'])
def api_joystick_combined():
    """Envoie pan et tilt combinés"""
    data = request.get_json()
    pan = float(data.get('pan', 0.0))
    tilt = float(data.get('tilt', 0.0))
    
    pan = max(-1.0, min(1.0, pan))
    tilt = max(-1.0, min(1.0, tilt))
    
    success = send_osc_message('/joy/pt', pan, tilt)
    return jsonify({'success': True, 'pan': pan, 'tilt': tilt})

# Routes pour slide control
@app.route('/api/slide/jog', methods=['POST'])
def api_slide_jog():
    """Contrôle jog du slide"""
    data = request.get_json()
    value = float(data.get('value', 0.0))  # -1.0 to 1.0
    value = max(-1.0, min(1.0, value))
    
    success = send_osc_message('/slide/jog', value)
    return jsonify({'success': True, 'value': value})

@app.route('/api/slide/goto', methods=['POST'])
def api_slide_goto():
    """Déplace le slide vers une position"""
    data = request.get_json()
    position = float(data.get('position', 0.5))  # 0.0 to 1.0
    duration = float(data.get('duration', 2.0))
    
    position = max(0.0, min(1.0, position))
    duration = max(0.1, min(60.0, duration))
    
    success = send_osc_message('/slide/goto', position, duration)
    return jsonify({'success': True, 'position': position, 'duration': duration})

# Routes pour configuration
@app.route('/api/config/offset_range', methods=['POST'])
def api_config_offset_range():
    """Configure les ranges des offsets"""
    data = request.get_json()
    pan_range = int(data.get('pan_range', 800))
    tilt_range = int(data.get('tilt_range', 800))
    
    success = send_osc_message('/config/offset_range', pan_range, tilt_range)
    return jsonify({'success': True, 'pan_range': pan_range, 'tilt_range': tilt_range})

@app.route('/api/config/pan_map', methods=['POST'])
def api_config_pan_map():
    """Configure le mapping pan"""
    data = request.get_json()
    min_val = int(data.get('min', 800))
    max_val = int(data.get('max', -800))
    
    success = send_osc_message('/config/pan_map', min_val, max_val)
    return jsonify({'success': True, 'min': min_val, 'max': max_val})

@app.route('/api/config/tilt_map', methods=['POST'])
def api_config_tilt_map():
    """Configure le mapping tilt"""
    data = request.get_json()
    min_val = int(data.get('min', 0))
    max_val = int(data.get('max', 0))
    
    success = send_osc_message('/config/tilt_map', min_val, max_val)
    return jsonify({'success': True, 'min': min_val, 'max': max_val})

# Routes pour presets avancés
@app.route('/api/preset/set', methods=['POST'])
def api_preset_set():
    """Définit un preset avec des valeurs spécifiques"""
    data = request.get_json()
    preset_id = int(data.get('id', 0))
    pan = int(data.get('pan', 0))
    tilt = int(data.get('tilt', 0))
    zoom = int(data.get('zoom', 0))
    slide = int(data.get('slide', 0))
    
    success = send_osc_message('/preset/set', preset_id, pan, tilt, zoom, slide)
    return jsonify({'success': True, 'id': preset_id})

# Routes pour joystick config
@app.route('/api/joystick/config', methods=['POST'])
def api_joystick_config():
    """Configure joystick parameters"""
    data = request.get_json()
    deadzone = float(data.get('deadzone', 0.06))
    expo = float(data.get('expo', 0.35))
    slew = float(data.get('slew', 8000.0))
    filter_hz = float(data.get('filter_hz', 60.0))
    pan_tilt_speed = float(data.get('pan_tilt_speed', 1.0))
    slide_speed = float(data.get('slide_speed', 1.0))
    
    # Clamp values to valid ranges
    deadzone = max(0.0, min(0.5, deadzone))
    expo = max(0.0, min(0.95, expo))
    slew = max(0.0, slew)
    filter_hz = max(0.0, filter_hz)
    pan_tilt_speed = max(0.1, min(3.0, pan_tilt_speed))
    slide_speed = max(0.1, min(3.0, slide_speed))
    
    success = send_osc_message('/joy/config', deadzone, expo, slew, filter_hz, pan_tilt_speed, slide_speed)
    return jsonify({
        'success': True, 
        'deadzone': deadzone, 
        'expo': expo, 
        'slew': slew, 
        'filter_hz': filter_hz,
        'pan_tilt_speed': pan_tilt_speed,
        'slide_speed': slide_speed
    })

# Routes pour follow mapping et AB mode
@app.route('/api/follow/enable', methods=['POST'])
def api_follow_enable():
    """Enable/disable follow mapping"""
    data = request.get_json()
    enable = bool(data.get('enable', True))
    
    success = send_osc_message('/follow/en', 1 if enable else 0)
    return jsonify({'success': True, 'enabled': enable})

@app.route('/api/slide/ab', methods=['POST'])
def api_slide_ab():
    """Enable/disable infinite AB mode"""
    data = request.get_json()
    enable = bool(data.get('enable', False))
    
    success = send_osc_message('/slide/ab', 1 if enable else 0)
    return jsonify({'success': True, 'enabled': enable})

@app.route('/api/slide/ab/set', methods=['POST'])
def api_slide_ab_set():
    """Set AB points and duration"""
    data = request.get_json()
    point_a = float(data.get('point_a', 0.0))  # 0.0 to 1.0
    point_b = float(data.get('point_b', 1.0))  # 0.0 to 1.0
    duration = float(data.get('duration', 4.0))  # seconds
    
    # Clamp values
    point_a = max(0.0, min(1.0, point_a))
    point_b = max(0.0, min(1.0, point_b))
    duration = max(0.5, min(60.0, duration))
    
    success = send_osc_message('/slide/ab/set', point_a, point_b, duration)
    return jsonify({
        'success': True,
        'point_a': point_a,
        'point_b': point_b,
        'duration': duration
    })

@app.route('/api/stop', methods=['POST'])
def api_stop():
    """Stop all movement"""
    # Stop slide jog
    send_osc_message('/slide/jog', 0.0)
    # Reset joystick offsets
    send_osc_message('/pan', 0.0)
    send_osc_message('/tilt', 0.0)
    return jsonify({'success': True})

if __name__ == "__main__":
    print("ESP32 Slider Controller starting...")
    print(f"OSC Target: {ESP32_IP}:{ESP32_OSC_PORT}")
    print(f"Web interface: http://localhost:{FLASK_PORT}")
    
    # Démarrer le thread joystick
    _start_joystick_thread()
    
    # Démarrer Flask (sans auto-reload pour éviter les redémarrages du thread joystick)
    app.run(debug=False, use_reloader=False, host='0.0.0.0', port=FLASK_PORT)
