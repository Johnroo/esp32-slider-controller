#!/usr/bin/env python3
"""
ESP32 Slider Controller - Flask Web Application
Controls the ESP32 slider via OSC messages
"""

from flask import Flask, render_template, request, jsonify
import socket
import struct
import json
import time
import os
import threading
import math
from datetime import datetime

app = Flask(__name__)

# OSC Configuration
OSC_HOST = "192.168.1.198"  # ESP32 IP address (esproo.local)
OSC_PORT = 8000

# --- Joystick (pygame) config ---
JOYSTICK_USE_PYGAME = True   # Activé pour le joystick physique
JOYSTICK_SIMULATION = False  # Mode simulation désactivé
JOYSTICK_NAME_HINT = "Logitech Extreme 3D"
JOYSTICK_DEADZONE   = 0.08      # zone morte
JOYSTICK_EXPO       = 0.35      # 0..1 (0 = linéaire, 0.35 = doux, 0.7 = très expo)
JOYSTICK_RATE_HZ    = 60        # fréquence de polling (réduit pour stabilité)
JOYSTICK_SEND_EPS   = 0.01      # variation minimale pour renvoyer
JOYSTICK_SEND_HZ    = 30        # max envois OSC/s (réduit pour stabilité)

joystick_state = {
    'connected': True,  # Simulation activée par défaut
    'name': "Simulateur (Logitech Extreme 3D Pro)",
    'x': 0.0,     # -1..+1 (pan)
    'y': 0.0,     # -1..+1 (tilt, positif = stick vers le haut)
    'z': 0.0,
    'throttle': 0.0,
    'buttons': [False] * 12,
    'timestamp': 0.0
}

# OSC Message Builder
def pad_osc_string(s: str) -> bytes:
    b = s.encode('utf-8') + b'\x00'              # toujours 1 nul de fin
    pad = (4 - (len(b) % 4)) % 4                 # puis padding à /4
    return b + (b'\x00' * pad)

def create_osc_message(address, *args):
    # Adresse
    address_data = pad_osc_string(address)

    # Type tag (commence par ',')
    tags = ',' + ''.join('f' if isinstance(a, float) else 'i' for a in args)
    type_tag_data = pad_osc_string(tags)

    # Arguments (big-endian)
    import struct
    args_data = b''
    for a in args:
        if isinstance(a, float):
            args_data += struct.pack('>f', a)
        else:
            args_data += struct.pack('>i', int(a))

    return address_data + type_tag_data + args_data

def send_osc_message(address, *args):
    """Send OSC message to ESP32"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        message = create_osc_message(address, *args)
        sock.sendto(message, (OSC_HOST, OSC_PORT))
        sock.close()
        return True
    except Exception as e:
        print(f"Error sending OSC message: {e}")
        return False

#==================== Joystick (pygame) helpers ====================

def _expo(v: float, expo: float) -> float:
    """Mix linéaire/cubique : classique RC"""
    return (1.0 - expo) * v + expo * (v ** 3)

def _apply_deadzone(v: float, dz: float) -> float:
    """Applique la zone morte"""
    return 0.0 if abs(v) < dz else v

def _start_joystick_thread():
    """Démarre le thread de lecture du joystick"""
    if JOYSTICK_SIMULATION:
        print("[JOYSTICK] Mode simulation activé")
        # Démarrer un thread de simulation pour tester l'interface
        t = threading.Thread(target=_joystick_simulator, name="joystick-simulator", daemon=True)
        t.start()
        print("[JOYSTICK] Thread simulateur démarré")
        return
    elif not JOYSTICK_USE_PYGAME:
        print("[JOYSTICK] Joystick désactivé")
        return
    
    print("[JOYSTICK] Démarrage du thread pygame...")
    t = threading.Thread(target=_joystick_worker, name="pygame-joystick", daemon=True)
    t.start()
    print("[JOYSTICK] Thread démarré")

def _joystick_simulator():
    """Simulateur de joystick pour tester l'interface"""
    global joystick_state
    import math
    
    print("[JOYSTICK] Mode simulation activé")
    joystick_state['connected'] = True
    joystick_state['name'] = "Simulateur (Logitech Extreme 3D Pro)"
    
    # Simulation d'un mouvement circulaire
    t = 0
    while True:
        # Mouvement circulaire lent
        x = 0.3 * math.sin(t * 0.1)
        y = 0.3 * math.cos(t * 0.1)
        
        joystick_state.update({
            'x': float(x),
            'y': float(y),
            'z': 0.0,
            'throttle': 0.0,
            'buttons': [False] * 12,
            'timestamp': time.time()
        })
        
        # Envoyer les données simulées
        send_osc_message('/joy/pt', float(x), float(y))
        
        t += 1
        time.sleep(1.0 / 30)  # 30 FPS

def _joystick_worker():
    """Worker thread pour lire le joystick en continu"""
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

    # Initialisation en plusieurs étapes
    try:
        print("[JOYSTICK] Initialisation pygame...")
        pygame.init()
        print("[JOYSTICK] pygame.init() OK")
        
        pygame.joystick.init()
        print("[JOYSTICK] pygame.joystick.init() OK")
    except Exception as e:
        print(f"[JOYSTICK] Erreur initialisation: {e}")
        return
    
    # Délai pour la stabilisation
    time.sleep(0.5)
    
    try:
        count = pygame.joystick.get_count()
        print(f"[JOYSTICK] Nombre de joysticks détectés: {count}")
    except Exception as e:
        print(f"[JOYSTICK] Erreur get_count: {e}")
        return
    
    if count == 0:
        print("[JOYSTICK] Aucun joystick détecté")
        return
    
    js = None
    try:
        for i in range(count):
            try:
                j = pygame.joystick.Joystick(i)
                j.init()
                name = j.get_name() or ""
                print(f"[JOYSTICK] Joystick {i}: {name}")
                if JOYSTICK_NAME_HINT.lower() in name.lower() or js is None:
                    js = j
                    print(f"[JOYSTICK] Sélectionné: {name}")
            except Exception as e:
                print(f"[JOYSTICK] Erreur joystick {i}: {e}")
                continue
    except Exception as e:
        print(f"[JOYSTICK] Erreur lors de la sélection: {e}")
        return
    
    if js is None:
        print("[JOYSTICK] Aucun joystick sélectionné")
        return

    try:
        print(f"[JOYSTICK] Connecté: {js.get_name()}")
        joystick_state['connected'] = True
        joystick_state['name'] = js.get_name()
    except Exception as e:
        print(f"[JOYSTICK] Erreur connexion: {e}")
        return

    clock = pygame.time.Clock()
    last_send = time.time()
    last_x, last_y = 0.0, 0.0

    ema_x = 0.0
    ema_y = 0.0
    alpha = 0.2  # lissage léger

    while True:
        # vider la queue d'événements
        for _ in pygame.event.get():
            pass

        try:
            x = js.get_axis(0) if js.get_numaxes() > 0 else 0.0     # X
            y = js.get_axis(1) if js.get_numaxes() > 1 else 0.0     # Y
            z = js.get_axis(2) if js.get_numaxes() > 2 else 0.0     # twist
            thr = js.get_axis(3) if js.get_numaxes() > 3 else 0.0   # throttle
        except Exception:
            x = y = z = thr = 0.0

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

        # Envoi OSC throttlé (route combinée déjà présente côté ESP)
        send_due = (now - last_send) >= (1.0 / JOYSTICK_SEND_HZ)
        moved_enough = (abs(ema_x - last_x) > JOYSTICK_SEND_EPS) or (abs(ema_y - last_y) > JOYSTICK_SEND_EPS)
        if send_due and moved_enough:
            # /joy/pt  : pan, tilt  dans [-1..1]
            send_osc_message('/joy/pt', float(joystick_state['x']), float(joystick_state['y']))
            last_send = now
            last_x, last_y = ema_x, ema_y

        clock.tick(JOYSTICK_RATE_HZ)

# Web Routes
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/advanced')
def advanced():
    return render_template('advanced.html')

@app.route('/api/status', methods=['GET'])
def api_status():
    """Check server status"""
    return jsonify({'status': 'ok', 'connected': True})

@app.route('/api/joystick/state', methods=['GET'])
def api_joystick_state():
    """Obtenir l'état du joystick en temps réel"""
    # Retourne simplement l'état actuel du joystick (mis à jour par le thread)
    return jsonify(joystick_state)

@app.route('/api/slide/jog', methods=['POST'])
def api_slide_jog():
    """Send slide jog command (-1.0 to 1.0)"""
    data = request.get_json()
    value = float(data.get('value', 0.0))
    value = max(-1.0, min(1.0, value))  # Clamp to [-1, 1]
    
    success = send_osc_message('/slide/jog', value)
    return jsonify({'success': success, 'value': value})

@app.route('/api/pan', methods=['POST'])
def api_pan():
    """Send pan joystick offset (-1.0 to 1.0)"""
    data = request.get_json()
    value = float(data.get('value', 0.0))
    value = max(-1.0, min(1.0, value))  # Clamp to [-1, 1]
    
    success = send_osc_message('/pan', value)
    return jsonify({'success': success, 'value': value})

@app.route('/api/tilt', methods=['POST'])
def api_tilt():
    """Send tilt joystick offset (-1.0 to 1.0)"""
    data = request.get_json()
    value = float(data.get('value', 0.0))
    value = max(-1.0, min(1.0, value))  # Clamp to [-1, 1]
    
    success = send_osc_message('/tilt', value)
    return jsonify({'success': success, 'value': value})

@app.route('/api/joystick/combined', methods=['POST'])
def api_joystick_combined():
    """Send combined pan/tilt joystick (-1.0 to 1.0)"""
    data = request.get_json()
    pan = float(data.get('pan', 0.0))
    tilt = float(data.get('tilt', 0.0))
    pan = max(-1.0, min(1.0, pan))  # Clamp to [-1, 1]
    tilt = max(-1.0, min(1.0, tilt))  # Clamp to [-1, 1]
    
    success = send_osc_message('/joy/pt', pan, tilt)
    return jsonify({'success': success, 'pan': pan, 'tilt': tilt})

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
        'success': success, 
        'deadzone': deadzone, 
        'expo': expo, 
        'slew': slew, 
        'filter_hz': filter_hz,
        'pan_tilt_speed': pan_tilt_speed,
        'slide_speed': slide_speed
    })

@app.route('/api/follow/enable', methods=['POST'])
def api_follow_enable():
    """Enable/disable follow mapping"""
    data = request.get_json()
    enable = bool(data.get('enable', True))
    
    success = send_osc_message('/follow/en', 1 if enable else 0)
    return jsonify({'success': success, 'enabled': enable})

@app.route('/api/slide/ab', methods=['POST'])
def api_slide_ab():
    """Enable/disable infinite AB mode"""
    data = request.get_json()
    enable = bool(data.get('enable', False))
    
    success = send_osc_message('/slide/ab', 1 if enable else 0)
    return jsonify({'success': success, 'enabled': enable})

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
        'success': success,
        'point_a': point_a,
        'point_b': point_b,
        'duration': duration
    })

@app.route('/api/stop', methods=['POST'])
def api_stop():
    """Stop all movement"""
    # Stop slide jog
    success1 = send_osc_message('/slide/jog', 0.0)
    # Reset joystick offsets
    success2 = send_osc_message('/pan', 0.0)
    success3 = send_osc_message('/tilt', 0.0)
    return jsonify({'success': success1 and success2 and success3})

@app.route('/api/reset_offsets', methods=['POST'])
def api_reset_offsets():
    """Reset pan and tilt offsets"""
    pan_success = send_osc_message('/pan', 0.0)
    tilt_success = send_osc_message('/tilt', 0.0)
    return jsonify({'success': pan_success and tilt_success})

# Nouvelles routes pour contrôle direct des axes
@app.route('/api/axis_pan', methods=['POST'])
def api_axis_pan():
    """Send direct pan position (0.0 to 1.0)"""
    data = request.get_json()
    value = float(data.get('value', 0.0))
    value = max(0.0, min(1.0, value))  # Clamp to [0, 1]
    
    success = send_osc_message('/axis_pan', value)
    return jsonify({'success': success, 'value': value})

@app.route('/api/axis_tilt', methods=['POST'])
def api_axis_tilt():
    """Send direct tilt position (0.0 to 1.0)"""
    data = request.get_json()
    value = float(data.get('value', 0.0))
    value = max(0.0, min(1.0, value))  # Clamp to [0, 1]
    
    success = send_osc_message('/axis_tilt', value)
    return jsonify({'success': success, 'value': value})

@app.route('/api/axis_zoom', methods=['POST'])
def api_axis_zoom():
    """Send direct zoom position (0.0 to 1.0)"""
    data = request.get_json()
    value = float(data.get('value', 0.0))
    value = max(0.0, min(1.0, value))  # Clamp to [0, 1]
    
    success = send_osc_message('/axis_zoom', value)
    return jsonify({'success': success, 'value': value})

@app.route('/api/axis_slide', methods=['POST'])
def api_axis_slide():
    """Send direct slide position (0.0 to 1.0)"""
    data = request.get_json()
    value = float(data.get('value', 0.0))
    value = max(0.0, min(1.0, value))  # Clamp to [0, 1]
    
    success = send_osc_message('/axis_slide', value)
    return jsonify({'success': success, 'value': value})

@app.route('/api/reset_all_axes', methods=['POST'])
def api_reset_all_axes():
    """Reset all axes to center position (0.5)"""
    pan_success = send_osc_message('/axis_pan', 0.5)
    tilt_success = send_osc_message('/axis_tilt', 0.5)
    zoom_success = send_osc_message('/axis_zoom', 0.5)
    slide_success = send_osc_message('/axis_slide', 0.5)
    return jsonify({'success': pan_success and tilt_success and zoom_success and slide_success})

#==================== NEW: Advanced Motion Control Routes ====================


@app.route('/api/preset/recall', methods=['POST'])
def recall_preset():
    """Recall preset with duration"""
    try:
        data = request.get_json()
        preset_id = int(data.get('id', 0))
        duration = float(data.get('duration', 2.0))
        
        # Send OSC message
        send_osc_message('/preset/recall', preset_id, duration)
        
        return jsonify({
            'success': True,
            'preset_id': preset_id,
            'duration': duration,
            'timestamp': datetime.now().isoformat()
        })
    except Exception as e:
        return jsonify({'error': str(e)}), 400

@app.post('/api/preset/store')
def preset_store():
    try:
        data = request.get_json()
        i = int(data.get('id'))
        send_osc_message('/preset/store', i)
        return jsonify({'success': True, 'id': i})
    except Exception as e:
        return jsonify({'error': str(e)}), 400

@app.post('/api/preset/mode')
def preset_mode():
    try:
        data = request.get_json()
        i = int(data.get('id'))
        mode = data.get('mode', 'abs')
        m = 1 if mode == 'follow' else 0
        send_osc_message('/preset/mode', i, m)
        return jsonify({'success': True, 'id': i, 'mode': mode})
    except Exception as e:
        return jsonify({'error': str(e)}), 400

@app.post('/api/preset/recall_policy')
def preset_recall_policy():
    try:
        data = request.get_json()
        slide = int(data.get('slide', 0))  # 0: KEEP_AB, 1: GOTO_THEN_RESUME
        send_osc_message('/preset/recall_policy', slide)
        return jsonify({'success': True, 'slide': slide})
    except Exception as e:
        return jsonify({'error': str(e)}), 400

@app.route('/api/joystick/pan', methods=['POST'])
def set_pan_offset():
    """Set pan joystick offset (-1.0 to 1.0)"""
    try:
        data = request.get_json()
        value = float(data.get('value', 0.0))
        
        # Clamp value between -1.0 and 1.0
        value = max(-1.0, min(1.0, value))
        
        # Send OSC message
        send_osc_message('/pan', value)
        
        return jsonify({
            'success': True,
            'offset': value,
            'timestamp': datetime.now().isoformat()
        })
    except Exception as e:
        return jsonify({'error': str(e)}), 400

@app.route('/api/joystick/tilt', methods=['POST'])
def set_tilt_offset():
    """Set tilt joystick offset (-1.0 to 1.0)"""
    try:
        data = request.get_json()
        value = float(data.get('value', 0.0))
        
        # Clamp value between -1.0 and 1.0
        value = max(-1.0, min(1.0, value))
        
        # Send OSC message
        send_osc_message('/tilt', value)
        
        return jsonify({
            'success': True,
            'offset': value,
            'timestamp': datetime.now().isoformat()
        })
    except Exception as e:
        return jsonify({'error': str(e)}), 400

@app.route('/api/slide/jog', methods=['POST'])
def slide_jog():
    """Slide jog mode (-1.0 to 1.0)"""
    try:
        data = request.get_json()
        value = float(data.get('value', 0.0))
        
        # Clamp value between -1.0 and 1.0
        value = max(-1.0, min(1.0, value))
        
        # Send OSC message
        send_osc_message('/slide/jog', value)
        
        return jsonify({
            'success': True,
            'jog_speed': value,
            'timestamp': datetime.now().isoformat()
        })
    except Exception as e:
        return jsonify({'error': str(e)}), 400

@app.route('/api/slide/goto', methods=['POST'])
def slide_goto():
    """Slide goto position with duration"""
    try:
        data = request.get_json()
        position = float(data.get('position', 0.5))  # 0.0 to 1.0
        duration = float(data.get('duration', 2.0))
        
        # Clamp position between 0.0 and 1.0
        position = max(0.0, min(1.0, position))
        
        # Send OSC message
        send_osc_message('/slide/goto', position, duration)
        
        return jsonify({
            'success': True,
            'position': position,
            'duration': duration,
            'timestamp': datetime.now().isoformat()
        })
    except Exception as e:
        return jsonify({'error': str(e)}), 400

@app.route('/api/config/offset_range', methods=['POST'])
def set_offset_range():
    """Set joystick offset ranges"""
    try:
        data = request.get_json()
        pan_range = int(data.get('pan_range', 800))
        tilt_range = int(data.get('tilt_range', 800))
        
        # Send OSC message
        send_osc_message('/config/offset_range', pan_range, tilt_range)
        
        return jsonify({
            'success': True,
            'pan_range': pan_range,
            'tilt_range': tilt_range,
            'timestamp': datetime.now().isoformat()
        })
    except Exception as e:
        return jsonify({'error': str(e)}), 400

@app.route('/api/config/pan_map', methods=['POST'])
def set_pan_mapping():
    """Set slide to pan mapping"""
    try:
        data = request.get_json()
        min_value = int(data.get('min', 800))
        max_value = int(data.get('max', -800))
        
        # Send OSC message
        send_osc_message('/config/pan_map', min_value, max_value)
        
        return jsonify({
            'success': True,
            'min': min_value,
            'max': max_value,
            'timestamp': datetime.now().isoformat()
        })
    except Exception as e:
        return jsonify({'error': str(e)}), 400

@app.route('/api/config/tilt_map', methods=['POST'])
def set_tilt_mapping():
    """Set slide to tilt mapping"""
    try:
        data = request.get_json()
        min_value = int(data.get('min', 0))
        max_value = int(data.get('max', 0))
        
        # Send OSC message
        send_osc_message('/config/tilt_map', min_value, max_value)
        
        return jsonify({
            'success': True,
            'min': min_value,
            'max': max_value,
            'timestamp': datetime.now().isoformat()
        })
    except Exception as e:
        return jsonify({'error': str(e)}), 400

#==================== NEW: Routes pour offsets latched ====================

@app.route('/api/offset/zero', methods=['POST'])
def api_offset_zero():
    """Reset les offsets latched (pan et/ou tilt)"""
    try:
        data = request.get_json()
        do_pan = data.get('pan', True) if data else True
        do_tilt = data.get('tilt', True) if data else True
        
        # Envoyer commande OSC
        if do_pan and do_tilt:
            success = send_osc_message('/offset/zero', 1, 1)
        elif do_pan:
            success = send_osc_message('/offset/zero', 1, 0)
        elif do_tilt:
            success = send_osc_message('/offset/zero', 0, 1)
        else:
            success = True
        
        return jsonify({
            'success': success,
            'pan_reset': do_pan,
            'tilt_reset': do_tilt,
            'timestamp': datetime.now().isoformat()
        })
    except Exception as e:
        return jsonify({'error': str(e)}), 400

@app.route('/api/offset/add', methods=['POST'])
def api_offset_add():
    """Ajouter des steps aux offsets latched"""
    try:
        data = request.get_json()
        d_pan = int(data.get('d_pan', 0))
        d_tilt = int(data.get('d_tilt', 0))
        
        # Envoyer commande OSC
        success = send_osc_message('/offset/add', d_pan, d_tilt)
        
        return jsonify({
            'success': success,
            'd_pan': d_pan,
            'd_tilt': d_tilt,
            'timestamp': datetime.now().isoformat()
        })
    except Exception as e:
        return jsonify({'error': str(e)}), 400

@app.route('/api/offset/set', methods=['POST'])
def api_offset_set():
    """Définir les offsets latched absolument"""
    try:
        data = request.get_json()
        pan_offset = int(data.get('pan', 0))
        tilt_offset = int(data.get('tilt', 0))
        
        # Envoyer commande OSC
        success = send_osc_message('/offset/set', pan_offset, tilt_offset)
        
        return jsonify({
            'success': success,
            'pan_offset': pan_offset,
            'tilt_offset': tilt_offset,
            'timestamp': datetime.now().isoformat()
        })
    except Exception as e:
        return jsonify({'error': str(e)}), 400

@app.route('/api/offset/bake', methods=['POST'])
def api_offset_bake():
    """Intégrer les offsets dans le preset et les remettre à zéro"""
    try:
        # Envoyer commande OSC
        success = send_osc_message('/offset/bake')
        
        return jsonify({
            'success': success,
            'message': 'Offsets baked into preset and reset',
            'timestamp': datetime.now().isoformat()
        })
    except Exception as e:
        return jsonify({'error': str(e)}), 400

@app.route('/api/offset/status', methods=['GET'])
def api_offset_status():
    """Obtenir le statut des offsets (lecture seule)"""
    try:
        # Note: Cette route ne peut pas lire les valeurs actuelles depuis l'ESP32
        # car nous n'avons pas implémenté de route OSC de lecture
        # Elle retourne juste le statut de connexion
        return jsonify({
            'success': True,
            'message': 'Offset status endpoint available',
            'note': 'Actual offset values not available via OSC',
            'timestamp': datetime.now().isoformat()
        })
    except Exception as e:
        return jsonify({'error': str(e)}), 400

if __name__ == '__main__':
    from config import FLASK_PORT
    
    print(f"ESP32 Slider Controller starting...")
    print(f"OSC Target: {OSC_HOST}:{OSC_PORT}")
    print(f"Web interface: http://localhost:{FLASK_PORT}")
    
    # Démarrer le thread du joystick
    _start_joystick_thread()
    
    # Attendre un peu pour que le simulateur démarre
    time.sleep(1)
    
    app.run(debug=True, host='0.0.0.0', port=FLASK_PORT)
