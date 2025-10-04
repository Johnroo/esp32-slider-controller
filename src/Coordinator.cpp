#include "Coordinator.h"
#include "Config.h"
#include "MotorControl.h"
#include "Joystick.h"
#include "Presets.h"
#include "MotionPlanner.h"
#include "Utils.h"

void Coordinator::coordinatorTick() {
    static uint32_t last_ms = millis();
    uint32_t now = millis();
    uint32_t dt_ms = now - last_ms;
    if (dt_ms == 0) return;
    last_ms = now;

    // 1) Mouvement synchronisé (priorité la plus haute)
    if (isActive()) {
        updateMotionPlanner();
        return; // Ignorer tous les autres modes pendant un mouvement sync
    }

    // 2) Interpolation automatique (priorité haute)
    if (interpAuto.active) {
        updateInterpolation();
        return; // Ignorer les autres modes pendant l'interpolation auto
    }

    // 3) Interpolation manuelle (jog)
    if (fabs(interp_jog_cmd) > 0.001f) {
        updateInterpolationJog();
        return; // Ignorer les autres modes pendant le jog d'interpolation
    }

    // 4) Contrôle joystick direct (mode par défaut)
    // Jog direct des axes individuels
    float dt = dt_ms / 1000.0f;
    
    // Jog Pan
    if (isPanActive()) {
        long p = steppers[0]->targetPos();
        p = clampL(p + (long)lround(joy_filt.pan * PAN_JOG_SPEED * dt), cfg[0].min_limit, cfg[0].max_limit);
        steppers[0]->moveTo(p);
    }
    
    // Jog Tilt
    if (isTiltActive()) {
        long t = steppers[1]->targetPos();
        t = clampL(t + (long)lround(joy_filt.tilt * TILT_JOG_SPEED * dt), cfg[1].min_limit, cfg[1].max_limit);
        steppers[1]->moveTo(t);
    }
    
    // Jog Slide
    if (isSlideActive()){
        long s = steppers[3]->targetPos();
        long Sgoal = clampL(s + (long)lround(slide_jog_cmd * SLIDE_JOG_SPEED * dt),
                            cfg[3].min_limit, cfg[3].max_limit);
        steppers[3]->moveTo(Sgoal);
    }
}
