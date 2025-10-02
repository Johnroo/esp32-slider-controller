/**
 * @file Homing.cpp
 * @brief Implémentation du module de homing
 */

#include "Homing.h"

//==================== Variables globales ====================
bool doAutoHomeSlide = false;          // lancer automatiquement au démarrage si true
uint8_t slide_sg_threshold = 100;      // SGTHRS par défaut (sensibilité moyenne)

// Variables d'état
volatile bool stall_detected = false;  // Flag global de détection de stall
volatile bool homing_in_progress = false; // Flag indiquant qu'un homing est en cours

//==================== Fonctions d'initialisation ====================
void initHoming() {
  Serial.println("🏠 Initialisation du module Homing...");
  
  // Configuration StallGuard pour tous les drivers
  for (int i = 0; i < NUM_MOTORS; i++) {
    setupStallGuard(i, cfg[i].sgt);
  }
  
  Serial.println("✅ Module Homing initialisé");
}

void setupStallGuard(int driverIndex, uint8_t threshold) {
  if (driverIndex < 0 || driverIndex >= NUM_MOTORS) {
    Serial.printf("❌ Index driver invalide: %d\n", driverIndex);
    return;
  }
  
  auto driver = drivers[driverIndex];
  driver->SGTHRS(threshold);
  
  Serial.printf("✅ StallGuard configuré pour driver %d (seuil: %d)\n", 
                driverIndex, threshold);
}

//==================== Fonctions de homing ====================
void homeSlide() {
  int i = SLIDE_INDEX;
  
  if (!steppers[i]) {
    Serial.println("❌ Stepper slide non initialisé");
    return;
  }
  
  Serial.println("🏠 HOMING SLIDE START (StallGuard4)");
  
  // Configuration pour StallGuard4 (StealthChop requis)
  drivers[i]->en_spreadCycle(false);  // StealthChop pour StallGuard4
  drivers[i]->SGTHRS(slide_sg_threshold);   // Sensibilité StallGuard
  
  // Configuration vitesse/accel pour homing
  steppers[i]->setAcceleration(HOMING_ACCEL);
  steppers[i]->setSpeedInHz(HOMING_SPEED);
  
  Serial.printf("⚙️ Config: speed=%d, accel=%d, SGTHRS=%d, SG_DETECT=%d\n",
                HOMING_SPEED, HOMING_ACCEL, slide_sg_threshold, SG_DETECT);
  
  // === HOMING BUTÉE INFÉRIEURE ===
  Serial.println("⬇️ Homing butée inférieure...");
  stall_detected = false;
  unsigned long t0 = millis();
  
  steppers[i]->runBackward();
  
  // Attendre détection de stall ou timeout
  while (millis() - t0 < HOMING_TIMEOUT && !stall_detected) {
    delay(10);
    
    // Détecter stall après 500ms de mouvement
    if (millis() - t0 > 500) {
      uint16_t sg_result = drivers[i]->SG_RESULT();
      if (sg_result < SG_DETECT) {
        Serial.printf("🛑 Stall détecté (INF): SG_RESULT=%d\n", sg_result);
        stall_detected = true;
      }
    }
  }
  
  if (!stall_detected) {
    Serial.println("⚠️ Timeout INF - pas de stall détecté");
    steppers[i]->forceStop();
    return;
  }
  
  steppers[i]->forceStop();
  long minPos = steppers[i]->getCurrentPosition();
  Serial.printf("📍 Position butée INF: %ld\n", minPos);
  
  // Recul
  Serial.printf("↩️ Recul de %d pas...\n", BACKOFF_STEPS);
  steppers[i]->move(BACKOFF_STEPS);
  while (steppers[i]->isRunning()) delay(2);
  
  // === HOMING BUTÉE SUPÉRIEURE ===
  Serial.println("⬆️ Homing butée supérieure...");
  stall_detected = false;
  t0 = millis();
  
  steppers[i]->runForward();
  
  // Attendre détection de stall ou timeout
  while (millis() - t0 < HOMING_TIMEOUT && !stall_detected) {
    delay(10);
    
    // Détecter stall après 500ms de mouvement
    if (millis() - t0 > 500) {
      uint16_t sg_result = drivers[i]->SG_RESULT();
      if (sg_result < SG_DETECT) {
        Serial.printf("🛑 Stall détecté (SUP): SG_RESULT=%d\n", sg_result);
        stall_detected = true;
      }
    }
  }
  
  if (!stall_detected) {
    Serial.println("⚠️ Timeout SUP - pas de stall détecté");
    steppers[i]->forceStop();
    return;
  }
  
  steppers[i]->forceStop();
  long maxPos = steppers[i]->getCurrentPosition();
  Serial.printf("📍 Position butée SUP: %ld\n", maxPos);
  
  // === CENTRAGE ET MISE À ZÉRO ===
  long center = (minPos + maxPos) / 2;
  Serial.printf("🎯 Centrage vers position: %ld\n", center);
  
  steppers[i]->moveTo(center);
  while (steppers[i]->isRunning()) delay(2);
  
  // Mise à zéro de la position
  steppers[i]->setCurrentPosition(0);
  
  // Restaurer configuration normale
  steppers[i]->setAcceleration(cfg[i].max_accel);
  steppers[i]->setSpeedInHz(cfg[i].max_speed);
  
  // Mettre à jour les limites dans la configuration
  cfg[i].min_limit = minPos - center;
  cfg[i].max_limit = maxPos - center;
  
  Serial.printf("✅ Homing terminé - Limites: [%ld, %ld]\n", 
                cfg[i].min_limit, cfg[i].max_limit);
  Serial.println("🏠 HOMING SLIDE END");
}

void homeAllAxes() {
  Serial.println("🏠 Début homing de tous les axes...");
  
  // Pour l'instant, seul le slide est homé automatiquement
  // Les autres axes (PAN, TILT, ZOOM) restent en position actuelle
  homeSlide();
  
  Serial.println("✅ Homing de tous les axes terminé");
}

//==================== Fonctions utilitaires ====================
bool isAutoHomeEnabled() {
  return doAutoHomeSlide;
}

void setAutoHomeEnabled(bool enabled) {
  doAutoHomeSlide = enabled;
  Serial.printf("🏠 Homing automatique %s\n", enabled ? "activé" : "désactivé");
}

uint8_t getSlideSGThreshold() {
  return slide_sg_threshold;
}

void setSlideSGThreshold(uint8_t threshold) {
  slide_sg_threshold = threshold;
  drivers[SLIDE_INDEX]->SGTHRS(slide_sg_threshold);
  cfg[SLIDE_INDEX].sgt = slide_sg_threshold;
  Serial.printf("⚙️ Nouvelle SGTHRS (slide) = %d\n", slide_sg_threshold);
}

//==================== Fonctions de détection de stall ====================
bool isStallDetected(int driverIndex) {
  if (driverIndex < 0 || driverIndex >= NUM_MOTORS) {
    return false;
  }
  
  uint16_t sg_result = drivers[driverIndex]->SG_RESULT();
  return sg_result < SG_DETECT;
}

void IRAM_ATTR stallGuardCallback() {
  // Cette fonction peut être appelée par une interruption StallGuard
  // Pour l'instant, on utilise la détection par polling dans homeSlide()
  stall_detected = true;
}
