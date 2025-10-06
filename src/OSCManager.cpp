#include "OSCManager.h"
#include "Config.h"
#include "MotorControl.h"
#include "Joystick.h"
#include "Presets.h"
#include "MotionPlanner.h"
#include "Homing.h"
#include "Utils.h"
#include <ArduinoJson.h>

// Déclarations externes pour les offsets persistants
extern long interp_offset_p;
extern long interp_offset_t;
extern long interp_offset_z;
extern long interp_offset_s;

// Variables statiques
WiFiUDP OSCManager::udp;
OSCErrorCode OSCManager::error;

bool OSCManager::initOSC() {
    if (udp.begin(OSC_PORT)) {
        Serial.println("✅ OSC Server started on port " + String(OSC_PORT));
        Serial.println("📡 Waiting for OSC messages...");
        return true;
    } else {
        Serial.println("❌ Failed to start OSC server on port " + String(OSC_PORT));
        return false;
    }
}

void OSCManager::processOSC() {
    OSCMessage msg;
    int size = udp.parsePacket();
    
    if (size > 0) {
        Serial.println("🔔 OSC packet received, size: " + String(size));
        
        while (size--) {
            msg.fill(udp.read());
        }
        
        Serial.println("🔍 OSC message address: " + String(msg.getAddress()));
        Serial.println("🔍 OSC message size: " + String(msg.size()));
        
        if (!msg.hasError()) {
            // Traitement des messages OSC par catégories
            handleJoystickRoutes(msg);
            handleAxisRoutes(msg);
            handlePresetRoutes(msg);
            handleInterpolationRoutes(msg);
            handleBankRoutes(msg);
            handleMotorConfigRoutes(msg);
            handleHomingRoutes(msg);
            handleOffsetRoutes(msg);
            handleConfigRoutes(msg);
        } else {
            Serial.println("❌ OSC Error: " + String(msg.getError()));
        }
    }
}

void OSCManager::handleJoystickRoutes(OSCMessage &msg) {
    // Joystick en OSC (-1..+1)
    msg.dispatch("/pan", [](OSCMessage &m){ 
        float pan = clampF(m.getFloat(0), -1.f, +1.f);
        setRawJoystickValues(pan, joy_raw.tilt, joy_raw.slide);
    });
    
    msg.dispatch("/tilt", [](OSCMessage &m){ 
        float tilt = clampF(m.getFloat(0), -1.f, +1.f);
        setRawJoystickValues(joy_raw.pan, tilt, joy_raw.slide);
    });

    // Zoom offset direct (aligné avec pan/tilt)
    msg.dispatch("/zoom", [](OSCMessage &m){
        float z = clampF(m.getFloat(0), -1.f, +1.f);
        joy_raw.zoom = z;
        Serial.printf("🎛️ Zoom (offset) = %.3f\n", z);
    });
    
    msg.dispatch("/joy/pt", [](OSCMessage &m){ 
        float pan = clampF(m.getFloat(0), -1.f, +1.f);
        float tilt = clampF(m.getFloat(1), -1.f, +1.f);
        setRawJoystickValues(pan, tilt, joy_raw.slide);
    });
    
    // Slide offset direct (aligné avec pan/tilt/zoom)
    msg.dispatch("/slide", [](OSCMessage &m){ 
        float slide = clampF(m.getFloat(0), -1.f, +1.f);
        setRawJoystickValues(joy_raw.pan, joy_raw.tilt, slide);
        Serial.printf("🎛️ Slide (offset) = %.3f\n", slide);
    });
    
    // Alias rétrocompatibilité
    msg.dispatch("/slide/jog", [](OSCMessage &m){ 
        float slide = clampF(m.getFloat(0), -1.f, +1.f);
        setRawJoystickValues(joy_raw.pan, joy_raw.tilt, slide);
    });
    msg.dispatch("/joy/zoom", [](OSCMessage &m){
        float z = clampF(m.getFloat(0), -1.f, +1.f);
        joy_raw.zoom = z;
    });
    
    // Optionnel: réglages runtime
    msg.dispatch("/joy/config", [](OSCMessage &m){
        JoyCfg config = getJoystickConfig();
        if (m.size() > 0) config.deadzone = clampF(m.getFloat(0), 0.f, 0.5f);
        if (m.size() > 1) config.expo = clampF(m.getFloat(1), 0.f, 0.95f);
        if (m.size() > 2) config.slew_per_s = fabsf(m.getFloat(2));
        if (m.size() > 3) config.filt_hz = fabsf(m.getFloat(3));
        if (m.size() > 4) config.slide_speed = clampF(m.getFloat(4), 0.1f, 3.0f);
        setJoystickConfig(config);
        
        Serial.printf("🎛 Joy cfg: dz=%.2f expo=%.2f slew=%.0f filt=%.1f S=%.2fx\n",
                      config.deadzone, config.expo, config.slew_per_s, config.filt_hz,
                      config.slide_speed);
    });
    
}

void OSCManager::handleAxisRoutes(OSCMessage &msg) {
    msg.dispatch("/axis_pan", [](OSCMessage &msg) {
        float value = clampF(msg.getFloat(0), 0.0f, 1.0f);
        long pos_val = (long)(value * (cfg[0].max_limit - cfg[0].min_limit) + cfg[0].min_limit);
        Serial.println("🔧 Moving Pan to: " + String(pos_val));
        steppers[0]->moveTo(pos_val);
        Serial.println("Axis Pan: " + String(value) + " -> " + String(pos_val));
        Serial.println("🔧 Pan stepper running: " + String(steppers[0]->isRunning()));
    });
    
    msg.dispatch("/axis_tilt", [](OSCMessage &msg) {
        float value = clampF(msg.getFloat(0), 0.0f, 1.0f);
        long pos_val = (long)(value * (cfg[1].max_limit - cfg[1].min_limit) + cfg[1].min_limit);
        Serial.println("🔧 Moving Tilt to: " + String(pos_val));
        steppers[1]->moveTo(pos_val);
        Serial.println("Axis Tilt: " + String(value) + " -> " + String(pos_val));
        Serial.println("🔧 Tilt stepper running: " + String(steppers[1]->isRunning()));
    });
    
    msg.dispatch("/axis_zoom", [](OSCMessage &msg) {
        float value = clampF(msg.getFloat(0), 0.0f, 1.0f);
        long pos_val = (long)(value * (cfg[2].max_limit - cfg[2].min_limit) + cfg[2].min_limit);
        Serial.println("🔧 Moving Zoom to: " + String(pos_val));
        steppers[2]->moveTo(pos_val);
        Serial.println("Axis Zoom: " + String(value) + " -> " + String(pos_val));
        Serial.println("🔧 Zoom stepper running: " + String(steppers[2]->isRunning()));
    });
    
    msg.dispatch("/axis_slide", [](OSCMessage &msg) {
        float value = clampF(msg.getFloat(0), 0.0f, 1.0f);
        long pos_val = (long)(value * (cfg[3].max_limit - cfg[3].min_limit) + cfg[3].min_limit);
        Serial.println("🔧 Moving Slide to: " + String(pos_val));
        steppers[3]->moveTo(pos_val);
        Serial.println("Axis Slide: " + String(value) + " -> " + String(pos_val));
        Serial.println("🔧 Slide stepper running: " + String(steppers[3]->isRunning()));
    });
}

void OSCManager::handlePresetRoutes(OSCMessage &msg) {
    msg.dispatch("/preset/set", [](OSCMessage &m){
        int i = m.getInt(0);
        presets[i].p = m.getInt(1);
        presets[i].t = m.getInt(2);
        presets[i].z = m.getInt(3);
        presets[i].s = m.getInt(4);
        Serial.printf("Preset %d saved\n", i);
    });

    // Store réel: capture positions courantes et calcule les ancres
    msg.dispatch("/preset/store", [](OSCMessage &m){
        int i = m.getInt(0);
        long S = steppers[3]->getCurrentPosition();
        long P = steppers[0]->getCurrentPosition();
        long T = steppers[1]->getCurrentPosition();
        long Z = steppers[2]->getCurrentPosition();

        // Calculs de compensation follow supprimés (mode obsolète)

        presets[i].p = P; presets[i].t = T; presets[i].z = Z; presets[i].s = S;

        Serial.printf("\xF0\x9F\x92\xBE Store preset %d | ABS P:%ld T:%ld Z:%ld S:%ld\n",
                      i, P,T,Z,S);
    });


    msg.dispatch("/preset/recall", [](OSCMessage &m){
        int i = m.getInt(0);
        float Tsec = m.getFloat(1);
        if (Tsec <= 0) Tsec = 2.0f;
        activePreset = i;

        // Réinitialiser tous les offsets avant le recall
        resetLatchedOffsets();
        saveOffsetBaseline();
        interp_offset_p = interp_offset_t = interp_offset_z = interp_offset_s = 0;
        Serial.println("🔄 Offsets joystick réinitialisés au début du preset recall");

        uint32_t Tms_req = (uint32_t)lround(Tsec * 1000.0);
        long base_goal[NUM_MOTORS] = { presets[i].p, presets[i].t, presets[i].z, presets[i].s };

        if (planSynchronizedMove(base_goal, Tms_req)) {
            Serial.printf("▶️ Recall(ABS): P:%ld T:%ld Z:%ld S:%ld\n",
                          base_goal[0], base_goal[1], base_goal[2], base_goal[3]);
        }
    });

    // Slide: goto [0..1] en T sec (déplacement temps imposé)
    msg.dispatch("/slide/goto", [](OSCMessage &m){
        float u = clampF(m.getFloat(0), 0.0f, 1.0f);
        float Tsec = m.getFloat(1); if (Tsec <= 0) Tsec = 2.0f;

        long s_goal = (long)lround(lerp(cfg[3].min_limit, cfg[3].max_limit, u));
        uint32_t Tms_req = (uint32_t)lround(Tsec*1000.0);
        
        if (planSlideMove(s_goal, Tsec)) {
            Serial.printf("🎬 Slide goto: %ld (%.1f%%)\n", s_goal, u * 100.0f);
        }
    });
}

void OSCManager::handleInterpolationRoutes(OSCMessage &msg) {
    // Routes OSC pour interpolation multi-presets
    msg.dispatch("/interp/setpoints", [](OSCMessage &m){
        if (m.size() < 1) return;
        uint8_t N = m.getInt(0);
        if (N > 6) N = 6;  // max 6 points
        if (N < 2) N = 2;  // min 2 points
        
        interpCount = N;
        for (uint8_t j = 0; j < N; ++j) {
            if ((1 + 2*j + 1) < m.size()) {
                interpPoints[j].presetIndex = m.getInt(1 + 2*j);
                interpPoints[j].fraction = m.getFloat(1 + 2*j + 1);
            }
        }
        Serial.printf("🎯 Interp points set: N=%d\n", N);
        for (uint8_t j = 0; j < N; ++j) {
            Serial.printf("   Point %d: Preset %d @ %.1f%%\n", j, interpPoints[j].presetIndex, interpPoints[j].fraction * 100.0f);
        }
    });
    
    msg.dispatch("/interp/auto", [](OSCMessage &m){
        bool enable = m.getInt(0) != 0;
        float duration = (m.size() > 1 ? m.getFloat(1) : 5.0f);
        
        if (enable) {
            uint32_t T_ms = (duration <= 0 ? 5000 : (uint32_t)lround(duration * 1000));
            interpAuto.T_ms = T_ms;
            interpAuto.t0_ms = millis();
            interpAuto.active = true;
            
            // Sauvegarder le baseline des offsets
            saveOffsetBaseline();
            
            // Désactiver modes concurrents
            stopSynchronizedMove();
            
            Serial.printf("▶️ Interpolation auto ON (T=%u ms, %.1fs)\n", T_ms, duration);
        } else {
            interpAuto.active = false;
            Serial.println("⏹️ Interpolation auto OFF");
        }
    });
    
    msg.dispatch("/interp/goto", [](OSCMessage &m){
        float fraction = clampF(m.getFloat(0), 0.0f, 1.0f);
        
        // Annuler modes auto
        interpAuto.active = false;
        stopSynchronizedMove();
        
        // Calculer position interpolée
        long P, T, Z, S;
        computeInterpolatedPosition(fraction, P, T, Z, S);
        
        // Commander les moteurs
        steppers[0]->moveTo(P);
        steppers[1]->moveTo(T);
        steppers[2]->moveTo(Z);
        steppers[3]->moveTo(S);
        
        Serial.printf("🎛️ Manual interp goto %.1f%% -> P=%ld T=%ld Z=%ld S=%ld\n", 
                      fraction * 100.0f, P, T, Z, S);
    });

    msg.dispatch("/interp/jog", [](OSCMessage &m){
        float value = clampF(m.getFloat(0), -1.0f, 1.0f);
        
        // Annuler les mouvements automatiques ou presets en cours
        interpAuto.active = false;
        stopSynchronizedMove();
        
        // Appliquer la nouvelle consigne de vitesse de l'axe d'interpolation
        interp_jog_cmd = value;
        Serial.printf("🎛️ Interp jog speed = %.2f\n", value);
    });
}

void OSCManager::handleBankRoutes(OSCMessage &msg) {
    // Bank Management Routes
    msg.dispatch("/bank/set", [](OSCMessage &m){
        if (m.size() > 0) {
            uint8_t idx = m.getInt(0);
            if (idx < 10) {
                activeBank = idx;
                loadBank(idx);

                // Réinitialiser les offsets joystick au changement de banque
                resetLatchedOffsets();
                saveOffsetBaseline();
                interp_offset_p = interp_offset_t = interp_offset_z = interp_offset_s = 0;
                Serial.println("🔄 Offsets joystick réinitialisés au changement de banque");

                Serial.printf("🏦 Banque active changée vers %d\n", idx);
                
                // Renvoyer les points d'interpolation après le chargement
                DynamicJsonDocument doc(1024);
                doc["interpCount"] = interpCount;
                
                JsonArray interpArray = doc.createNestedArray("interp");
                for (int i = 0; i < interpCount; i++) {
                    JsonObject interp = interpArray.createNestedObject();
                    interp["presetIndex"] = interpPoints[i].presetIndex;
                    interp["fraction"] = interpPoints[i].fraction * 100.0f; // Convertir en pourcentage
                }
                
                // Sérialiser et envoyer
                String jsonString;
                serializeJson(doc, jsonString);
                Serial.printf("📤 Bank %d interp points: %s\n", idx, jsonString.c_str());
            }
        }
    });

    msg.dispatch("/bank/save", [](OSCMessage &m){
        saveActiveBank();
        Serial.printf("💾 Banque %d sauvegardée\n", activeBank);
    });

    msg.dispatch("/bank/get_interp", [](OSCMessage &m){
        // Construire le JSON avec les points d'interpolation actuels
        DynamicJsonDocument doc(1024);
        doc["interpCount"] = interpCount;
        
        JsonArray interpArray = doc.createNestedArray("interp");
        for (int i = 0; i < interpCount; i++) {
            JsonObject interp = interpArray.createNestedObject();
            interp["presetIndex"] = interpPoints[i].presetIndex;
            interp["fraction"] = interpPoints[i].fraction * 100.0f; // Convertir en pourcentage
        }
        
        // Sérialiser en string
        String jsonString;
        serializeJson(doc, jsonString);
        
        // Envoyer via OSC (on va utiliser une route spéciale pour le retour)
        // Pour l'instant, on log le JSON
        Serial.printf("📤 Interp points JSON: %s\n", jsonString.c_str());
    });
}

void OSCManager::handleMotorConfigRoutes(OSCMessage &msg) {
    // Motor Configuration Routes
    msg.dispatch("/motor/pan/max_speed", [](OSCMessage &m){
        if (m.size() > 0) {
            int speed = m.getInt(0);
            if (speed >= 2000 && speed <= 20000){
                cfg[0].max_speed = speed;
                steppers[0]->setSpeedInHz(speed);
                Serial.printf("🔧 Pan max_speed = %d steps/s\n", speed);
            }
        }
    });

    msg.dispatch("/motor/pan/max_accel", [](OSCMessage &m){
        if (m.size() > 0) {
            int accel = m.getInt(0);
            if (accel >= 1000 && accel <= 999999){
                cfg[0].max_accel = accel;
                steppers[0]->setAcceleration(accel);
                Serial.printf("🔧 Pan max_accel = %d steps/s²\n", accel);
            }
        }
    });

    msg.dispatch("/motor/tilt/max_speed", [](OSCMessage &m){
        if (m.size() > 0) {
            int speed = m.getInt(0);
            if (speed >= 2000 && speed <= 20000){
                cfg[1].max_speed = speed;
                steppers[1]->setSpeedInHz(speed);
                Serial.printf("🔧 Tilt max_speed = %d steps/s\n", speed);
            }
        }
    });

    msg.dispatch("/motor/tilt/max_accel", [](OSCMessage &m){
        if (m.size() > 0) {
            int accel = m.getInt(0);
            if (accel >= 1000 && accel <= 999999){
                cfg[1].max_accel = accel;
                steppers[1]->setAcceleration(accel);
                Serial.printf("🔧 Tilt max_accel = %d steps/s²\n", accel);
            }
        }
    });

    msg.dispatch("/motor/zoom/max_speed", [](OSCMessage &m){
        if (m.size() > 0) {
            int speed = m.getInt(0);
            if (speed >= 2000 && speed <= 20000){
                cfg[2].max_speed = speed;
                steppers[2]->setSpeedInHz(speed);
                Serial.printf("🔧 Zoom max_speed = %d steps/s\n", speed);
            }
        }
    });

    msg.dispatch("/motor/zoom/max_accel", [](OSCMessage &m){
        if (m.size() > 0) {
            int accel = m.getInt(0);
            if (accel >= 1000 && accel <= 999999){
                cfg[2].max_accel = accel;
                steppers[2]->setAcceleration(accel);
                Serial.printf("🔧 Zoom max_accel = %d steps/s²\n", accel);
            }
        }
    });

    msg.dispatch("/motor/slide/max_speed", [](OSCMessage &m){
        if (m.size() > 0) {
            int speed = m.getInt(0);
            if (speed >= 2000 && speed <= 20000){
                cfg[3].max_speed = speed;
                steppers[3]->setSpeedInHz(speed);
                Serial.printf("🔧 Slide max_speed = %d steps/s\n", speed);
            }
        }
    });

    msg.dispatch("/motor/slide/max_accel", [](OSCMessage &m){
        if (m.size() > 0) {
            int accel = m.getInt(0);
            if (accel >= 1000 && accel <= 999999){
                cfg[3].max_accel = accel;
                steppers[3]->setAcceleration(accel);
                Serial.printf("🔧 Slide max_accel = %d steps/s²\n", accel);
            }
        }
    });
}

void OSCManager::handleHomingRoutes(OSCMessage &msg) {
    // Homing slide & StallGuard threshold
    msg.dispatch("/slide/home", [](OSCMessage &m){
        Serial.println("\xF0\x9F\x8F\xA0 Commande OSC: homing du slide");
        Serial.printf("🔍 DEBUG: homingInProgress = %s, adresse = %p\n", isHomingInProgress() ? "true" : "false", &homingInProgress);
        if (isHomingInProgress()) {
            Serial.println("⚠️ Homing déjà en cours, commande ignorée");
            return;
        }
        home_slide();
    });
    
    msg.dispatch("/slide/sgthrs", [](OSCMessage &m){
        if (m.size() > 0) {
            int thr = m.getInt(0);
            if (thr < 0) thr = 0; if (thr > 255) thr = 255;
            setSlideSGThreshold((uint8_t)thr);
        }
    });
}

void OSCManager::handleOffsetRoutes(OSCMessage &msg) {
    // Routes OSC pour offsets latched
    msg.dispatch("/offset/zero", [](OSCMessage &m){
        int do_pan  = (m.size()>0) ? m.getInt(0) : 1;
        int do_tilt = (m.size()>1) ? m.getInt(1) : 1;
        if (do_pan)  pan_offset_latched  = 0;
        if (do_tilt) tilt_offset_latched = 0;
        Serial.println("🔄 Reset offsets: pan=" + String(pan_offset_latched) + ", tilt=" + String(tilt_offset_latched));
    });
    
    msg.dispatch("/offset/add", [](OSCMessage &m){
        long pan = 0, tilt = 0, zoom = 0, slide = 0;
        if (m.size() > 0) pan = m.getInt(0);
        if (m.size() > 1) tilt = m.getInt(1);
        if (m.size() > 2) zoom = m.getInt(2);
        if (m.size() > 3) slide = m.getInt(3);
        addLatchedOffsets(pan, tilt, zoom, slide);
        Serial.println("➕ Add offsets: pan=" + String(pan_offset_latched) + ", tilt=" + String(tilt_offset_latched) + ", zoom=" + String(zoom_offset_latched) + ", slide=" + String(slide_offset_latched));
    });
    
    msg.dispatch("/offset/set", [](OSCMessage &m){
        long pan = 0, tilt = 0, zoom = 0, slide = 0;
        if (m.size() > 0) pan = m.getInt(0);
        if (m.size() > 1) tilt = m.getInt(1);
        if (m.size() > 2) zoom = m.getInt(2);
        if (m.size() > 3) slide = m.getInt(3);
        setLatchedOffsets(pan, tilt, zoom, slide);
        Serial.println("🎯 Set offsets: pan=" + String(pan_offset_latched) + ", tilt=" + String(tilt_offset_latched) + ", zoom=" + String(zoom_offset_latched) + ", slide=" + String(slide_offset_latched));
    });
    
    msg.dispatch("/offset/bake", [](OSCMessage &m){
        if (isActive()){
            // Intègre l'offset actuel dans la goal_base du preset
            long pan, tilt, zoom, slide;
            getLatchedOffsets(pan, tilt, zoom, slide);
            bakeOffsetsIntoCurrentMove(pan, tilt, zoom, slide);
        }
        resetLatchedOffsets();
        Serial.println("🔄 Reset offsets after bake");
    });
    
    msg.dispatch("/offset/reset_all", [](OSCMessage &m){
        resetLatchedOffsets();
        saveOffsetBaseline();
        interp_offset_p = interp_offset_t = interp_offset_z = interp_offset_s = 0;
        Serial.println("🔄 Tous les offsets joystick réinitialisés manuellement");
    });
}

void OSCManager::handleConfigRoutes(OSCMessage &msg) {
    // Config: ranges offsets et mapping slide->pan/tilt
    msg.dispatch("/config/offset_range", [](OSCMessage &m){
        long pan = m.getInt(0);
        long tilt = m.getInt(1);
        long zoom = (m.size() > 2) ? m.getInt(2) : DEFAULT_ZOOM_SLEW_RANGE;
        long slide = (m.size() > 3) ? m.getInt(3) : DEFAULT_SLIDE_SLEW_RANGE;
        setOffsetRanges(pan, tilt, zoom, slide);
    });
    // Routes /config/pan_map et /config/tilt_map supprimées (mode follow obsolète)
}
