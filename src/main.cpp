#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <ArduinoOTA.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include <OSCBundle.h>
#include <OSCData.h>
#include <math.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include "MotorControl.h"
#include "Homing.h"
#include "Presets.h"
#include "Tracking.h"
#include "MotionPlanner.h"
#include "Joystick.h"
#include "Utils.h"
#include "Config.h"
#include "NetworkManager.h"

//==================== Configuration ====================

//==================== NEW: Presets, offsets, mapping ====================
// Les structures Preset et variables sont maintenant dans le module Presets

// Les variables d'offsets et structures sont maintenant dans le module Joystick

// Les constantes sont maintenant dans le module Config

// Les fonctions d'offsets sont maintenant dans le module Joystick

// Les constantes de mapping sont maintenant dans le module Tracking

// La structure SyncMove et la variable sync_move sont maintenant dans le module MotionPlanner

// La structure CancelPolicy est maintenant dans le module Joystick

// Couplage Pan/Tilt ↔ Slide pendant le jog
// Les structures Follow et AnchorMorph sont maintenant dans le module Tracking

// Politique de recall du slide
enum class SlideRecallPolicy : uint8_t { KEEP_AB = 0, GOTO_THEN_RESUME = 1 };
struct RecallPolicy {
  SlideRecallPolicy slide = SlideRecallPolicy::KEEP_AB;
} recallPolicy;

//==================== Homing Slide (StallGuard) ====================
// Variables de homing maintenant dans le module Homing.h

// Mode AB infini pour le slide (déprécié - remplacé par interpolation multi-presets)
struct SlideAB {
  bool enabled = false;
  long A = 0, B = 0;       // steps
  uint32_t T_ms = 4000;    // durée d'un aller
  uint32_t t0_ms = 0;
  int dir = +1;            // +1: A->B, -1: B->A
} slideAB;

// Les structures d'interpolation et banques sont maintenant dans le module Presets

// Les variables de jog slide sont maintenant dans le module Joystick

// La variable interp_jog_cmd est maintenant dans le module Presets

// Les vitesses de jog sont maintenant dans le module Joystick

// Pins et configuration des moteurs maintenant dans MotorControl.h

// Configuration des axes maintenant dans MotorControl.cpp

// Objets moteurs maintenant dans MotorControl.cpp

// Fonction setupDriversTMC() maintenant dans MotorControl.cpp

// Homing du slide via StallGuard4 (TMC2209)
// Configuration maintenant dans Homing.h

void home_slide() {
  // Déléguer au module Homing
  homeSlide();
}


// Les fonctions utilitaires sont maintenant dans le module Presets

//==================== NEW: Joystick Pipeline ====================
// Les structures JoyCfg, JoyState et variables sont maintenant dans le module Joystick

// Les fonctions utilitaires du joystick sont maintenant dans le module Joystick

// La fonction joystick_tick() est maintenant dans le module Joystick

// La fonction pick_duration_ms_for_deltas est maintenant dans le module MotionPlanner

//==================== Bank Management Functions ====================
// La fonction saveBank est maintenant dans le module Presets

// La fonction loadBank est maintenant dans le module Presets

void saveActiveBank() {
  saveBank(activeBank);
}

void loadActiveBank() {
  loadBank(activeBank);
}

// Les fonctions de mapping et de suivi sont maintenant dans le module Tracking

// Les fonctions d'interpolation sont maintenant dans le module Presets

//==================== NEW: Tick de coordination ====================
void coordinator_tick(){
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
  
  if (slideAB.enabled && !isSynchronizedMoveActive()){
    float tau = (float)(now - slideAB.t0_ms) / (float)slideAB.T_ms;
    if (tau >= 1.0f){ slideAB.dir = -slideAB.dir; slideAB.t0_ms = now; tau = 0.0f; }
    float s = s_minjerk(tau);
    long Sgoal = (slideAB.dir > 0)
                   ? (long)lround(lerp(slideAB.A, slideAB.B, s))
                   : (long)lround(lerp(slideAB.B, slideAB.A, s));
    Sgoal = clampL(Sgoal, cfg[3].min_limit, cfg[3].max_limit);
    steppers[3]->moveTo(Sgoal);

    // Faire suivre Pan/Tilt via la map + offsets joystick
    if (follow.enabled){
      if (!follow.valid) refreshFollowAnchor();
      long pComp = panCompFromSlide(Sgoal);
      long tComp = tiltCompFromSlide(Sgoal);
      bool in_recall = isSynchronizedMoveActive() || isAnchorMorphActive();
      long Pgoal = clampL(follow.pan_anchor  + pComp + getEffectivePanOffset(in_recall),  cfg[0].min_limit, cfg[0].max_limit);
      long Tgoal = clampL(follow.tilt_anchor + tComp + getEffectiveTiltOffset(in_recall), cfg[1].min_limit, cfg[1].max_limit);
      steppers[0]->moveTo(Pgoal);
      steppers[1]->moveTo(Tgoal);
    }

    // Ne plus retourner ici — on laisse la suite traiter le joystick P/T
  }

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
      float pan_speed = PAN_JOG_SPEED * joy.pan_tilt_speed;
      p = clampL(p + (long)lround(joy_filt.pan * pan_speed * dt), cfg[0].min_limit, cfg[0].max_limit);
      steppers[0]->moveTo(p);
    }
    
    // Jog Tilt
    if (isTiltActive()) {
      long t = steppers[1]->targetPos();
      float tilt_speed = TILT_JOG_SPEED * joy.pan_tilt_speed;
      t = clampL(t + (long)lround(joy_filt.tilt * tilt_speed * dt), cfg[1].min_limit, cfg[1].max_limit);
      steppers[1]->moveTo(t);
    }
    
    // Jog Slide (+ follow map for Pan/Tilt) - bloqué pendant AB
    if (!slideAB.enabled && isSlideActive()){
      long s = steppers[3]->targetPos();
      long Sgoal = clampL(s + (long)lround(slide_jog_cmd * SLIDE_JOG_SPEED * dt),
                          cfg[3].min_limit, cfg[3].max_limit);
      steppers[3]->moveTo(Sgoal);

      // Faire suivre Pan/Tilt via la map slide->pan/tilt
      if (follow.enabled) {
        if (!follow.valid) refreshFollowAnchor();
        long pComp = panCompFromSlide(Sgoal);
        long tComp = tiltCompFromSlide(Sgoal);
        bool in_recall = isSynchronizedMoveActive() || isAnchorMorphActive();
        long Pgoal = clampL(follow.pan_anchor  + pComp + getEffectivePanOffset(in_recall),  cfg[0].min_limit, cfg[0].max_limit);
        long Tgoal = clampL(follow.tilt_anchor + tComp + getEffectiveTiltOffset(in_recall), cfg[1].min_limit, cfg[1].max_limit);
        
        // Ne PAS écraser un axe si le joystick le pilote déjà
        bool joyP = isPanActive();
        bool joyT = isTiltActive();
        
        if (!joyP) steppers[0]->moveTo(Pgoal);
        if (!joyT) steppers[1]->moveTo(Tgoal);
      }
    }
  }

  // 2) Mouvement synchronisé
  updateMotionPlanner();
}

// Variables de position maintenant dans MotorControl.cpp

//==================== OSC ====================
WiFiUDP udp;
OSCErrorCode error;
// Port OSC défini dans Config.h

//==================== Web Server ====================
AsyncWebServer webServer(80);

//==================== Setup OSC ====================
void setupOSC() {
  if (udp.begin(OSC_PORT)) {
    Serial.println("✅ OSC Server started on port " + String(OSC_PORT));
    Serial.println("📡 Waiting for OSC messages...");
  } else {
    Serial.println("❌ Failed to start OSC server on port " + String(OSC_PORT));
  }
}

//==================== Process OSC ====================
void processOSC() {
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
      // Traitement des messages OSC
      // Joystick en OSC (-1..+1)
      msg.dispatch("/pan", [](OSCMessage &m){ 
        // Annuler preset selon la politique
        if (cancel.by_joystick && isSynchronizedMoveActive()) {
          stopSynchronizedMove();
          Serial.println("⏹️ Cancel by joystick");
        }
        
        float pan = clampF(m.getFloat(0), -1.f, +1.f);
        setRawJoystickValues(pan, joy_raw.tilt, joy_raw.slide);
      });
      msg.dispatch("/tilt", [](OSCMessage &m){ 
        // Annuler preset selon la politique
        if (cancel.by_joystick && isSynchronizedMoveActive()) {
          stopSynchronizedMove();
          Serial.println("⏹️ Cancel by joystick");
        }
        
        float tilt = clampF(m.getFloat(0), -1.f, +1.f);
        setRawJoystickValues(joy_raw.pan, tilt, joy_raw.slide);
      });
      msg.dispatch("/joy/pt", [](OSCMessage &m){ 
        // Annuler preset selon la politique
        if (cancel.by_joystick && isSynchronizedMoveActive()) {
          stopSynchronizedMove();
          Serial.println("⏹️ Cancel by joystick");
        }
        
        float pan = clampF(m.getFloat(0), -1.f, +1.f);
        float tilt = clampF(m.getFloat(1), -1.f, +1.f);
        setRawJoystickValues(pan, tilt, joy_raw.slide);
      });
      msg.dispatch("/slide/jog", [](OSCMessage &m){ 
        // Annuler preset selon la politique
        if (cancel.by_joystick && isSynchronizedMoveActive()) {
          stopSynchronizedMove();
          Serial.println("⏹️ Cancel by joystick");
        }
        
        float slide = clampF(m.getFloat(0), -1.f, +1.f);
        setRawJoystickValues(joy_raw.pan, joy_raw.tilt, slide);
      });
      
      // Optionnel: réglages runtime
      msg.dispatch("/joy/config", [](OSCMessage &m){
        JoyCfg config = getJoystickConfig();
        if (m.size() > 0) config.deadzone = clampF(m.getFloat(0), 0.f, 0.5f);
        if (m.size() > 1) config.expo = clampF(m.getFloat(1), 0.f, 0.95f);
        if (m.size() > 2) config.slew_per_s = fabsf(m.getFloat(2));
        if (m.size() > 3) config.filt_hz = fabsf(m.getFloat(3));
        if (m.size() > 4) config.pan_tilt_speed = clampF(m.getFloat(4), 0.1f, 3.0f);
        if (m.size() > 5) config.slide_speed = clampF(m.getFloat(5), 0.1f, 3.0f);
        setJoystickConfig(config);
        
        Serial.printf("🎛 Joy cfg: dz=%.2f expo=%.2f slew=%.0f filt=%.1f PT=%.2fx S=%.2fx\n",
                      config.deadzone, config.expo, config.slew_per_s, config.filt_hz,
                      config.pan_tilt_speed, config.slide_speed);
      });
      
      // Configuration de la politique d'annulation
      msg.dispatch("/preset/cancel_policy", [](OSCMessage &m){
        CancelPolicy policy = getCancelPolicy();
        if (m.size() > 0) policy.by_joystick = m.getInt(0) != 0;
        if (m.size() > 1) policy.by_axis = m.getInt(1) != 0;
        setCancelPolicy(policy);
        Serial.printf("⚙️ Cancel policy: joystick=%d axis=%d\n", policy.by_joystick, policy.by_axis);
      });
      
      // Configuration du suivi Pan/Tilt ↔ Slide
      msg.dispatch("/follow/en", [](OSCMessage &m){
        if (m.getInt(0) != 0) {
          startTracking();
        } else {
          stopTracking();
        }
        Serial.printf("🎯 Follow mapping on jog: %s\n", isTrackingEnabled() ? "ON" : "OFF");
      });
      
      // Mode AB infini pour le slide (déprécié)
      msg.dispatch("/slide/ab", [](OSCMessage &m){
        slideAB.enabled = m.getInt(0) != 0;
        slideAB.t0_ms = millis();
        follow.valid = false; // re-anchor au démarrage
        Serial.printf("♾️ Slide AB %s\n", slideAB.enabled ? "ON" : "OFF");
      });
      
      msg.dispatch("/slide/ab/set", [](OSCMessage &m){
        float uA = clampF(m.getFloat(0), 0.f, 1.f);
        float uB = clampF(m.getFloat(1), 0.f, 1.f);
        float T  = fabsf(m.getFloat(2)); if (T <= 0) T = 2.f;
        slideAB.A    = (long)lround(lerp(cfg[3].min_limit, cfg[3].max_limit, uA));
        slideAB.B    = (long)lround(lerp(cfg[3].min_limit, cfg[3].max_limit, uB));
        slideAB.T_ms = (uint32_t)lround(T * 1000.f);
        slideAB.t0_ms= millis();
        slideAB.dir  = +1;
        Serial.printf("AB set A=%ld B=%ld T=%u ms\n", slideAB.A, slideAB.B, slideAB.T_ms);
      });
      
      //==================== Routes OSC pour interpolation multi-presets ====================
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
          
          // Désactiver modes concurrents
          slideAB.enabled = false;
          stopSynchronizedMove();
          follow.valid = false;
          
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
        slideAB.enabled = false;
        
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
        slideAB.enabled = false;
        follow.valid = false;
        
        // Appliquer la nouvelle consigne de vitesse de l'axe d'interpolation
        interp_jog_cmd = value;
        Serial.printf("🎛️ Interp jog speed = %.2f\n", value);
      });
      
      msg.dispatch("/axis_pan", [](OSCMessage &msg) {
        // Annuler preset selon la politique
        if (cancel.by_axis && isSynchronizedMoveActive()) {
          stopSynchronizedMove();
          Serial.println("⏹️ Manual override: cancel preset");
        }
        
        float value = clampF(msg.getFloat(0), 0.0f, 1.0f);
        long pos_val = (long)(value * (cfg[0].max_limit - cfg[0].min_limit) + cfg[0].min_limit);
        Serial.println("🔧 Moving Pan to: " + String(pos_val));
        steppers[0]->moveTo(pos_val);
        Serial.println("Axis Pan: " + String(value) + " -> " + String(pos_val));
        Serial.println("🔧 Pan stepper running: " + String(steppers[0]->isRunning()));
      });
      
      msg.dispatch("/axis_tilt", [](OSCMessage &msg) {
        // Annuler preset selon la politique
        if (cancel.by_axis && isSynchronizedMoveActive()) {
          stopSynchronizedMove();
          Serial.println("⏹️ Manual override: cancel preset");
        }
        
        float value = clampF(msg.getFloat(0), 0.0f, 1.0f);
        long pos_val = (long)(value * (cfg[1].max_limit - cfg[1].min_limit) + cfg[1].min_limit);
        Serial.println("🔧 Moving Tilt to: " + String(pos_val));
        steppers[1]->moveTo(pos_val);
        Serial.println("Axis Tilt: " + String(value) + " -> " + String(pos_val));
        Serial.println("🔧 Tilt stepper running: " + String(steppers[1]->isRunning()));
      });
      
      msg.dispatch("/axis_zoom", [](OSCMessage &msg) {
        // Annuler preset selon la politique
        if (cancel.by_axis && isSynchronizedMoveActive()) {
          stopSynchronizedMove();
          Serial.println("⏹️ Manual override: cancel preset");
        }
        
        float value = clampF(msg.getFloat(0), 0.0f, 1.0f);
        long pos_val = (long)(value * (cfg[2].max_limit - cfg[2].min_limit) + cfg[2].min_limit);
        Serial.println("🔧 Moving Zoom to: " + String(pos_val));
        steppers[2]->moveTo(pos_val);
        Serial.println("Axis Zoom: " + String(value) + " -> " + String(pos_val));
        Serial.println("🔧 Zoom stepper running: " + String(steppers[2]->isRunning()));
      });
      
      msg.dispatch("/axis_slide", [](OSCMessage &msg) {
        // Annuler preset selon la politique
        if (cancel.by_axis && isSynchronizedMoveActive()) {
          stopSynchronizedMove();
          Serial.println("⏹️ Manual override: cancel preset");
        }
        
        float value = clampF(msg.getFloat(0), 0.0f, 1.0f);
        long pos_val = (long)(value * (cfg[3].max_limit - cfg[3].min_limit) + cfg[3].min_limit);
        Serial.println("🔧 Moving Slide to: " + String(pos_val));
        steppers[3]->moveTo(pos_val);
        Serial.println("Axis Slide: " + String(value) + " -> " + String(pos_val));
        Serial.println("🔧 Slide stepper running: " + String(steppers[3]->isRunning()));
      });
      
      //==================== NEW: Routes OSC avancées ====================
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

      //==================== Motor Configuration Routes ====================
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

      //==================== Bank Management Routes ====================
      msg.dispatch("/bank/set", [](OSCMessage &m){
        if (m.size() > 0) {
          uint8_t idx = m.getInt(0);
          if (idx < 10) {
            activeBank = idx;
            loadBank(idx);
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

        long pComp = panCompFromSlide(S);
        long tComp = tiltCompFromSlide(S);

        presets[i].p = P; presets[i].t = T; presets[i].z = Z; presets[i].s = S;

        Serial.printf("\xF0\x9F\x92\xBE Store preset %d | ABS P:%ld T:%ld Z:%ld S:%ld\n",
                      i, P,T,Z,S);
      });

      // Mode preset supprimé - plus de différenciation entre ABSOLUTE et FOLLOW_SLIDE

      // Policy de recall slide
      msg.dispatch("/preset/recall_policy", [](OSCMessage &m){
        if (m.size() > 0) {
          int sp = m.getInt(0);
          recallPolicy.slide = (sp == 1) ? SlideRecallPolicy::GOTO_THEN_RESUME : SlideRecallPolicy::KEEP_AB;
          Serial.printf("Recall policy slide=%d\n", (int)recallPolicy.slide);
        }
      });

      msg.dispatch("/preset/recall", [](OSCMessage &m){
        int i = m.getInt(0);
        float Tsec = m.getFloat(1); if (Tsec <= 0) Tsec = 2.0f;
        activePreset = i;

        // Baseline des offsets au début du recall
        offset_session.pan0  = pan_offset_latched;
        offset_session.tilt0 = tilt_offset_latched;

        uint32_t Tms_req = (uint32_t)lround(Tsec*1000.0);

        // Logique simplifiée : tous les presets sont maintenant absolus
        {
          // Mouvement synchronisé normal (ABSOLUTE ou policy GOTO_THEN_RESUME)
          long base_goal[NUM_MOTORS] = { presets[i].p, presets[i].t, presets[i].z, presets[i].s };
          
          if (planSynchronizedMove(base_goal, Tms_req)) {
            Serial.printf("\xE2\x96\xBA Recall(ABS): P:%ld T:%ld Z:%ld S:%ld\n",
                          base_goal[0], base_goal[1], base_goal[2], base_goal[3]);
          }
        }
      });

      // Note: /pan, /tilt, /slide/jog sont déjà gérés plus haut dans le pipeline joystick

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

      // Config: ranges offsets et mapping slide->pan/tilt
      msg.dispatch("/config/offset_range", [](OSCMessage &m){
        setOffsetRanges(m.getInt(0), m.getInt(1));
      });
      msg.dispatch("/config/pan_map", [](OSCMessage &m){
        setPanMapping(m.getInt(0), m.getInt(1));
      });
      msg.dispatch("/config/tilt_map", [](OSCMessage &m){
        setTiltMapping(m.getInt(0), m.getInt(1));
      });
      
      //==================== NEW: Routes OSC pour offsets latched ====================
      msg.dispatch("/offset/zero", [](OSCMessage &m){
        int do_pan  = (m.size()>0) ? m.getInt(0) : 1;
        int do_tilt = (m.size()>1) ? m.getInt(1) : 1;
        if (do_pan)  pan_offset_latched  = 0;
        if (do_tilt) tilt_offset_latched = 0;
        Serial.println("🔄 Reset offsets: pan=" + String(pan_offset_latched) + ", tilt=" + String(tilt_offset_latched));
      });
      
      msg.dispatch("/offset/add", [](OSCMessage &m){
        long pan = 0, tilt = 0;
        if (m.size() > 0) pan = m.getInt(0);
        if (m.size() > 1) tilt = m.getInt(1);
        addLatchedOffsets(pan, tilt);
        Serial.println("➕ Add offsets: pan=" + String(pan_offset_latched) + ", tilt=" + String(tilt_offset_latched));
      });
      
      msg.dispatch("/offset/set", [](OSCMessage &m){
        long pan = 0, tilt = 0;
        if (m.size() > 0) pan = m.getInt(0);
        if (m.size() > 1) tilt = m.getInt(1);
        setLatchedOffsets(pan, tilt);
        Serial.println("🎯 Set offsets: pan=" + String(pan_offset_latched) + ", tilt=" + String(tilt_offset_latched));
      });
      
      msg.dispatch("/offset/bake", [](OSCMessage &m){
        if (isSynchronizedMoveActive()){
          // Intègre l'offset actuel dans la goal_base du preset
          long pan, tilt;
          getLatchedOffsets(pan, tilt);
          bakeOffsetsIntoCurrentMove(pan, tilt);
        }
        resetLatchedOffsets();
        Serial.println("🔄 Reset offsets after bake");
      });
    } else {
      Serial.println("❌ OSC Error: " + String(msg.getError()));
    }
  }
}

//==================== Web Handlers ====================
void setupWebServer() {
  webServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", "ESP32 Slider Controller - OSC Server Running");
  });
  
  webServer.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request){
    // Créer un JSON avec les positions des moteurs et les points d'interpolation
    DynamicJsonDocument doc(2048);
    
    // Positions actuelles des 4 moteurs
    doc["motors"]["pan"] = steppers[0]->targetPos();
    doc["motors"]["tilt"] = steppers[1]->targetPos();
    doc["motors"]["zoom"] = steppers[2]->targetPos();
    doc["motors"]["slide"] = steppers[3]->targetPos();
    
    // Positions en pourcentage (0-100)
    doc["motors_percent"]["pan"] = (float)(steppers[0]->targetPos() - cfg[0].min_limit) / (cfg[0].max_limit - cfg[0].min_limit) * 100.0f;
    doc["motors_percent"]["tilt"] = (float)(steppers[1]->targetPos() - cfg[1].min_limit) / (cfg[1].max_limit - cfg[1].min_limit) * 100.0f;
    doc["motors_percent"]["zoom"] = (float)(steppers[2]->targetPos() - cfg[2].min_limit) / (cfg[2].max_limit - cfg[2].min_limit) * 100.0f;
    doc["motors_percent"]["slide"] = (float)(steppers[3]->targetPos() - cfg[3].min_limit) / (cfg[3].max_limit - cfg[3].min_limit) * 100.0f;
    
    // Points d'interpolation actuels
    doc["interpolation"]["count"] = interpCount;
    JsonArray interpArray = doc["interpolation"].createNestedArray("points");
    for (int i = 0; i < interpCount; i++) {
      JsonObject point = interpArray.createNestedObject();
      point["presetIndex"] = interpPoints[i].presetIndex;
      point["fraction"] = interpPoints[i].fraction * 100.0f; // Convertir en pourcentage
    }
    
    // État des modes actifs
    doc["modes"]["interpAuto"] = interpAuto.active;
    doc["modes"]["syncMove"] = isSynchronizedMoveActive();
    doc["modes"]["slideAB"] = slideAB.enabled;
    
    // Banque active
    doc["bank"]["active"] = activeBank;
    
    // Sérialiser en JSON
    String jsonString;
    serializeJson(doc, jsonString);
    
    request->send(200, "application/json", jsonString);
  });
  
  webServer.on("/api/interpolation", HTTP_GET, [](AsyncWebServerRequest *request){
    // Route spécifique pour les points d'interpolation
    DynamicJsonDocument doc(1024);
    
    doc["interpCount"] = interpCount;
    JsonArray interpArray = doc.createNestedArray("interp");
    for (int i = 0; i < interpCount; i++) {
      JsonObject point = interpArray.createNestedObject();
      point["presetIndex"] = interpPoints[i].presetIndex;
      point["fraction"] = interpPoints[i].fraction * 100.0f; // Convertir en pourcentage
    }
    
    String jsonString;
    serializeJson(doc, jsonString);
    request->send(200, "application/json", jsonString);
  });
  
  //==================== NEW: Axes Status Endpoint ====================
  webServer.on("/api/axes/status", HTTP_GET, [](AsyncWebServerRequest *req){
    Serial.println("📡 /api/axes/status requested");
    DynamicJsonDocument doc(256);
    // Normalise les positions entre 0 et 1 selon min/max de chaque axe
    auto norm = [](long pos, long min, long max){
      return (max!=min)? (float)(pos - min) / (float)(max - min) : 0.0f;
    };
    
    long pan_pos = steppers[0]->getCurrentPosition();
    long tilt_pos = steppers[1]->getCurrentPosition();
    long zoom_pos = steppers[2]->getCurrentPosition();
    long slide_pos = steppers[3]->getCurrentPosition();
    
    doc["pan"]   = norm(pan_pos, cfg[0].min_limit, cfg[0].max_limit);
    doc["tilt"]  = norm(tilt_pos, cfg[1].min_limit, cfg[1].max_limit);
    doc["zoom"]  = norm(zoom_pos, cfg[2].min_limit, cfg[2].max_limit);
    doc["slide"] = norm(slide_pos, cfg[3].min_limit, cfg[3].max_limit);
    
    String out; 
    serializeJson(doc, out);
    Serial.printf("📊 Axes status: %s\n", out.c_str());
    req->send(200, "application/json", out);
  });
  
  // Test endpoint simple
  webServer.on("/api/test", HTTP_GET, [](AsyncWebServerRequest *req){
    Serial.println("🧪 /api/test requested");
    req->send(200, "application/json", "{\"status\":\"ok\",\"message\":\"test endpoint works\"}");
  });
  
  //==================== Bank Export Endpoint ====================
  webServer.on("/api/bank/*", HTTP_GET, [](AsyncWebServerRequest *req){
    String url = req->url();
    // Extraire l'ID de la banque de l'URL (ex: /api/bank/0 -> 0)
    int bankId = url.substring(url.lastIndexOf('/') + 1).toInt();
    Serial.printf("📤 /api/bank/%d requested\n", bankId);
    
    if (bankId < 0 || bankId >= 10) {
      req->send(400, "application/json", "{\"error\":\"Invalid bank ID\"}");
      return;
    }
    
    DynamicJsonDocument doc(4096);
    
    // Récupérer les presets de la banque
    JsonArray presetsArray = doc.createNestedArray("presets");
    for (int i = 0; i < 8; i++) {
      JsonObject preset = presetsArray.createNestedObject();
      Preset p = banks[bankId].presets[i];
      preset["p"] = p.p;
      preset["t"] = p.t;
      preset["z"] = p.z;
      preset["s"] = p.s;
    }
    
    // Récupérer les points d'interpolation de la banque
    doc["interpCount"] = banks[bankId].interpCount;
    JsonArray interpArray = doc.createNestedArray("interpPoints");
    for (int i = 0; i < banks[bankId].interpCount; i++) {
      JsonObject point = interpArray.createNestedObject();
      point["presetIndex"] = banks[bankId].interpPoints[i].presetIndex;
      point["fraction"] = banks[bankId].interpPoints[i].fraction * 100.0f; // Convertir en pourcentage
    }
    
    String jsonString;
    serializeJson(doc, jsonString);
    req->send(200, "application/json", jsonString);
  });
  
  webServer.begin();
  Serial.println("🌐 Web server started");
}

//==================== Setup ====================
void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("🚀 ESP32 Slider Controller Starting...");
  
  // Calculer les vitesses de jog basées sur la config
  float pan_speed = cfg[0].max_speed * 0.8f;   // 80% de la vitesse max
  float tilt_speed = cfg[1].max_speed * 0.8f;  // 80% de la vitesse max
  float slide_speed = cfg[3].max_speed * 0.8f;  // 80% de la vitesse max
  setJogSpeeds(pan_speed, tilt_speed, slide_speed);
  
  Serial.printf("🎯 Jog speeds: Pan=%.0f Tilt=%.0f Slide=%.0f steps/s\n", 
                pan_speed, tilt_speed, slide_speed);
  
  // Charger la banque 0 au démarrage
  loadBank(0);
  Serial.println("📂 Banque 0 chargée au démarrage");
  
  // Désactiver les anciens modes par défaut
  follow.enabled = false;
  slideAB.enabled = false;
  
  // Initialiser la configuration globale
  initConfig();
  
  // Initialiser les moteurs via le module MotorControl
  initMotors();
  
  // Initialiser le système de homing
  initHoming();
  
  // Initialiser le système de presets
  initPresets();
  
  // Initialiser le système de suivi
  initTracking();
  
  // Initialiser le planificateur de mouvements
  initMotionPlanner();
  
  // Initialiser le module joystick
  initJoystick();
  
  // Initialiser le réseau (WiFi + OTA)
  initNetwork();
  initOTA();
  
  // Web Server
  setupWebServer();
  
  // OSC
  setupOSC();

  Serial.println("🎯 System ready!");

  // Homing automatique optionnel
  if (doAutoHomeSlide) {
    Serial.println("\xF0\x9F\x8F\xA0 Homing automatique du slide au d\xC3\xA9marrage...");
    home_slide();
  }
}

//==================== Loop ====================
void loop() {
  handleOTA();
  processOSC();
  updateJoystick();    // NEW: Pipeline joystick avec lissage
  coordinator_tick();  // NEW: Orchestrateur de mouvements synchronisés
  
  // FastAccelStepper n'a pas besoin de engine.run()
  // Les moteurs se déplacent automatiquement
  
  // Mettre à jour les positions via le module MotorControl
  updateMotorPositions();

  // Log périodique (console web + série)
  static unsigned long tlog = 0;
  static unsigned long osc_log = 0;
  if (millis() - tlog > 500) {
    tlog = millis();
    String s = "t=" + String(millis()/1000.0, 2) + " jog=" + String(slide_jog_cmd, 2) +
               " | P:" + String(panPos) + " T:" + String(tiltPos) +
               " Z:" + String(zoomPos) + " S:" + String(slidePos);
    Serial.println(s);
  }
  
  // Log OSC status toutes les 5 secondes
  if (millis() - osc_log > 5000) {
    osc_log = millis();
    Serial.println("🔍 OSC listening on port " + String(OSC_PORT));
    
    // Vérifier l'état des drivers TMC
    for (int i=0; i<NUM_MOTORS; i++) {
      Serial.println("🔧 Driver " + String(i) + " toff: " + String(drivers[i]->toff()) + 
                     " tstep: " + String(drivers[i]->TSTEP()));
    }
  }
}