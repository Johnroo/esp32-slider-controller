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
#include "Diagnostics.h"

//==================== Configuration ====================
RecallPolicy recallPolicy;

//==================== Homing ====================

void home_slide() {
  // Déléguer au module Homing
  homeSlide();
}

//==================== Bank Management ====================

void saveActiveBank() {
  saveBank(activeBank);
}

void loadActiveBank() {
  loadBank(activeBank);
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
  
  // Initialiser les modules
  initConfig();
  initMotors();
  initHoming();
  initPresets();
  initTracking();
  initMotionPlanner();
  initJoystick();
  initNetwork();
  initOTA();
  initWebServer();
  OSCManager::initOSC();
  Diagnostics::initDiagnostics();
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
  updateJoystick();
  Coordinator::coordinatorTick();
  updateMotorPositions();
  Diagnostics::updateDiagnostics();
}