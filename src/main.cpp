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
#include "Coordinator.h"

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
// Les déclarations sont maintenant dans OSCManager.h
RecallPolicy recallPolicy;

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
// La fonction coordinator_tick() est maintenant dans le module Coordinator

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
  Coordinator::coordinatorTick();  // NEW: Orchestrateur de mouvements synchronisés
  
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