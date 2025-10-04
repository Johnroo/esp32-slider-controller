#include "Coordinator.h"
#include "Config.h"
#include "MotorControl.h"
#include "Joystick.h"
#include "Presets.h"
#include "MotionPlanner.h"
#include "Utils.h"

// Déclarations externes pour les variables joystick
extern JoyState joy_filt;

void Coordinator::coordinatorTick() {
    static uint32_t last_ms = millis();
    uint32_t now = millis();
    uint32_t dt_ms = now - last_ms;
    if (dt_ms == 0) return;
    last_ms = now;

    float dt = dt_ms / 1000.0f;

    // --- 1) Mouvements planifiés ---
    if (isActive()) {                 // recall preset - priorité absolue
        updateMotionPlanner();
        return; // Les recalls gardent la priorité, pas de joystick direct
    } else if (isInterpolationActive()) {  // automorph
        updateInterpolation();
        // Pas de return : le joystick reste actif comme offset
    } else if (fabs(interp_jog_cmd) > 0.001f) { // jog d'interp
        updateInterpolationJog();
        // Pas de return : le joystick reste actif comme offset
    }

    // --- 2) Joystick toujours actif (sauf pendant recall) ---
    // Pendant interpolation auto/jog : applique les offsets joystick
    // Sans interpolation : contrôle direct des moteurs
    applyJoystickOffsets(dt);
}

// Nouvelle fonction utilitaire à placer dans ce fichier (hors classe)
void applyJoystickOffsets(float dt) {
    // Déterminer le contexte : interpolation active ou contrôle direct
    bool inInterpolation = isInterpolationActive() || (fabs(interp_jog_cmd) > 0.001f);
    
    if (inInterpolation) {
        // Pendant l'interpolation : le joystick modifie les offsets persistants
        // Les offsets sont déjà gérés dans updateInterpolation() et updateInterpolationJog()
        // Ici on ne fait que logger l'activité
        if (fabs(joy_filt.pan) > 0.01f || fabs(joy_filt.tilt) > 0.01f || 
            fabs(joy_filt.zoom) > 0.01f || fabs(joy_filt.slide) > 0.01f) {
            Serial.printf("🎮 Interpolation + Joystick: P%.2f T%.2f Z%.2f S%.2f\n",
                          joy_filt.pan, joy_filt.tilt, joy_filt.zoom, joy_filt.slide);
        }
    } else {
        // Contrôle direct des moteurs (pas d'interpolation active)
        // PAN
        if (isPanActive()) {
            long p = steppers[0]->targetPos();
            p = clampL(p + (long)lround(joy_filt.pan * PAN_JOG_SPEED * dt),
                       cfg[0].min_limit, cfg[0].max_limit);
            steppers[0]->moveTo(p);
        }

        // TILT
        if (isTiltActive()) {
            long t = steppers[1]->targetPos();
            t = clampL(t + (long)lround(joy_filt.tilt * TILT_JOG_SPEED * dt),
                       cfg[1].min_limit, cfg[1].max_limit);
            steppers[1]->moveTo(t);
        }

        // ZOOM
        if (fabs(joy_filt.zoom) > 0.001f) {
            long z = steppers[2]->targetPos();
            z = clampL(z + (long)lround(joy_filt.zoom * ZOOM_JOG_SPEED * dt),
                       cfg[2].min_limit, cfg[2].max_limit);
            steppers[2]->moveTo(z);
        }

        // SLIDE
        if (isSlideActive()) {
            long s = steppers[3]->targetPos();
            s = clampL(s + (long)lround(joy_filt.slide * SLIDE_JOG_SPEED * dt),
                       cfg[3].min_limit, cfg[3].max_limit);
            steppers[3]->moveTo(s);
        }

        // Log de debug pour contrôle direct
        Serial.printf("🎮 Joystick direct: P%.2f T%.2f Z%.2f S%.2f\n",
                      joy_filt.pan, joy_filt.tilt, joy_filt.zoom, joy_filt.slide);
    }
}
