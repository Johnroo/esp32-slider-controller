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
#include "WebServerManager.h"
#include "OSCManager.h"

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

// Mode AB infini pour le slide supprimé (remplacé par interpolation multi-presets)

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

// Variables de position maintenant dans MotorControl.cpp

//==================== OSC ====================
// La logique OSC est maintenant gérée par le module OSCManager

//==================== Web Handlers ====================
// La configuration du serveur web est maintenant dans WebServerManager

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
  // Modes follow et slideAB supprimés (obsolètes)
  
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
  
  // Initialiser le serveur web
  initWebServer();
  
  // OSC
  OSCManager::initOSC();

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
  OSCManager::processOSC();
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