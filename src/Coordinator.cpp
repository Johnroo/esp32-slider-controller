#include "Coordinator.h"
#include "Config.h"
#include "MotorControl.h"
#include "Joystick.h"
#include "Presets.h"
#include "Tracking.h"
#include "MotionPlanner.h"
#include "Utils.h"

void Coordinator::coordinatorTick() {
    static uint32_t last_ms = millis();
    uint32_t now = millis();
    uint32_t dt_ms = now - last_ms;
    if (dt_ms == 0) return;
    last_ms = now;

    // Mode interpolation automatique multi-presets (prioritaire)
    updateInterpolation();

    // Mise à jour du suivi coordonné slide-pan/tilt
    updateTracking();

    // Interpolation d'ancre min-jerk pour recall autour de l'autopan
    
    // Logique slideAB supprimée (remplacée par interpolation multi-presets)

    // 2) Jog direct Pan/Tilt/Slide (vitesse) quand pas de mouvement sync
    if (!isSynchronizedMoveActive()){
        float dt = dt_ms / 1000.0f;
        
        // Jog interpolation manuel (prioritaire)
        static float interp_fraction = 0.0f;  // fraction courante sur l'axe d'interpolation
        if (fabs(interp_jog_cmd) > 0.001f) {
            // Déterminer le segment [j, j+1] de la courbe correspondant à interp_fraction
            uint8_t j = 0;
            while (j < interpCount - 1 && interpPoints[j+1].fraction < interp_fraction) {
                j++;
            }
            float u0 = interpPoints[j].fraction;
            float u1 = interpPoints[j+1].fraction;
            if (u1 < u0) u1 = u0;  // sécurité (au cas où, mais interpPoints est trié)
            
            // Calculer les écarts en pas sur ce segment pour chaque axe
            uint8_t presetA = interpPoints[j].presetIndex;
            uint8_t presetB = interpPoints[j+1].presetIndex;
            long dP = presets[presetB].p - presets[presetA].p;
            long dT = presets[presetB].t - presets[presetA].t;
            long dZ = presets[presetB].z - presets[presetA].z;
            long dS = presets[presetB].s - presets[presetA].s;
            float frac_len = (u1 - u0 > 0.0f ? u1 - u0 : 1.0f);
            
            // Calcul de la vitesse fractionnelle maximale autorisée (steps/s limitant)
            float maxFracSpeed = INFINITY;
            if (dP != 0) maxFracSpeed = fmin(maxFracSpeed, cfg[0].max_speed * frac_len / fabs(dP));
            if (dT != 0) maxFracSpeed = fmin(maxFracSpeed, cfg[1].max_speed * frac_len / fabs(dT));
            if (dZ != 0) maxFracSpeed = fmin(maxFracSpeed, cfg[2].max_speed * frac_len / fabs(dZ));
            if (dS != 0) maxFracSpeed = fmin(maxFracSpeed, cfg[3].max_speed * frac_len / fabs(dS));
            if (maxFracSpeed == INFINITY) {
                maxFracSpeed = 0.0f; // aucun mouvement requis si tous décalages nuls
            }
            
            // Intégration de la position fractionnelle en fonction de la vitesse demandée
            float du = interp_jog_cmd * maxFracSpeed * dt;
            interp_fraction = clampF(interp_fraction + du, 0.0f, 1.0f);
            
            // Calculer la nouvelle position interpolée et l'envoyer aux moteurs
            long P, T, Z, S;
            computeInterpolatedPosition(interp_fraction, P, T, Z, S);
            steppers[0]->moveTo(P);
            steppers[1]->moveTo(T);
            steppers[2]->moveTo(Z);
            steppers[3]->moveTo(S);
            
            // On quitte pour ne pas interférer avec les autres jogs
            return;
        }
        
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

    // 2) Mouvement synchronisé
    updateMotionPlanner();
}
