/**
 * @file Homing.cpp
 * @brief Implémentation du module de homing avec StallGuard
 * @author Laurent Eyen
 * @date 2024
 */

#include "Homing.h"

//==================== Variables globales ====================
bool doAutoHomeSlide = true;     // lancer automatiquement au démarrage si true
uint8_t slide_sg_threshold = 100; // SGTHRS par défaut (sensibilité moyenne)

//==================== Fonctions du module ====================

/**
 * @brief Initialise le système de homing (StallGuard)
 */
void initHoming() {
  Serial.println("🏠 Initialisation du système de homing...");
  
  // Configuration du seuil StallGuard pour le slide
  if (drivers[SLIDE_INDEX]) {
    drivers[SLIDE_INDEX]->SGTHRS(slide_sg_threshold);
    cfg[SLIDE_INDEX].sgt = slide_sg_threshold;
    Serial.printf("✅ StallGuard configuré (seuil: %d)\n", slide_sg_threshold);
  } else {
    Serial.println("❌ Driver slide non disponible pour StallGuard");
  }
  
  Serial.println("✅ Système de homing initialisé");
}

/**
 * @brief Effectue le homing complet du slide
 */
void homeSlide() {
  int i = SLIDE_INDEX;
  if (!steppers[i]) return;

  Serial.println("🏠 HOMING SLIDE START (StallGuard4)");

  // Désactiver tous les modes automatiques
  // slideAB.enabled = false;  // TODO: Déplacer dans le module approprié
  // sync_move.active = false; // TODO: Déplacer dans le module approprié
  // follow.enabled = false;   // TODO: Déplacer dans le module approprié

  // Sauvegarder la configuration actuelle (TODO: utiliser MotorControl)
  // int original_current = cfg[i].current_ma;
  // int original_accel = cfg[i].max_accel;
  // int original_speed = cfg[i].max_speed;

  // Configuration pour le homing
  steppers[i]->setSpeedInHz(HOMING_SPEED);
  steppers[i]->setAcceleration(HOMING_ACCEL);
  
  // Réduire le courant pour éviter les dégâts en cas de collision
  drivers[i]->rms_current(800); // Réduire à 800mA pour le homing
  
  Serial.printf("🔧 Configuration homing: vitesse=%d, accel=%d, courant=800mA\n", 
                HOMING_SPEED, HOMING_ACCEL);

  // ------------------ PHASE INF (vers butée inférieure) ------------------
  Serial.println("◀️ Vers butée INF...");
  steppers[i]->runBackward();
  uint32_t t0 = millis();
  uint32_t last_sg_log = 0;
  bool stall_detected = false;
  
  while (millis() - t0 < HOMING_TIMEOUT && !stall_detected) {
    uint16_t sg = drivers[i]->SG_RESULT();
    uint32_t now = millis();
    
    // Log SG_RESULT toutes les 300ms pour debug
    if (now - last_sg_log > 300) {
      Serial.printf("SG_RESULT: %u (seuil: %u)\n", sg, SG_DETECT);
      last_sg_log = now;
    }
    
    // Détecter stall après 500ms de mouvement
    if (millis() - t0 > 500 && sg < SG_DETECT) {
      Serial.printf("💥 INF Stall détecté (SG=%u < %u)\n", sg, SG_DETECT);
      steppers[i]->forceStop();
      stall_detected = true;
    }
    delay(2);
  }
  
  if (!stall_detected) {
    Serial.println("⚠️ Timeout INF - pas de stall détecté");
  }
  
  long minPos = steppers[i]->getCurrentPosition();
  Serial.printf("📍 Position INF: %ld\n", minPos);

  // Recul pour se dégager de la butée
  Serial.printf("↩️ Recul de %d pas...\n", BACKOFF_STEPS);
  steppers[i]->move(BACKOFF_STEPS);
  while (steppers[i]->isRunning()) delay(2);

  // ------------------ PHASE SUP (vers butée supérieure) ------------------
  Serial.println("▶️ Vers butée SUP...");
  steppers[i]->runForward();
  t0 = millis();
  last_sg_log = 0;
  stall_detected = false;
  
  while (millis() - t0 < HOMING_TIMEOUT && !stall_detected) {
    uint16_t sg = drivers[i]->SG_RESULT();
    uint32_t now = millis();
    
    // Log SG_RESULT toutes les 300ms pour debug
    if (now - last_sg_log > 300) {
      Serial.printf("SG_RESULT: %u (seuil: %u)\n", sg, SG_DETECT);
      last_sg_log = now;
    }
    
    // Détecter stall après 500ms de mouvement
    if (millis() - t0 > 500 && sg < SG_DETECT) {
      Serial.printf("💥 SUP Stall détecté (SG=%u < %u)\n", sg, SG_DETECT);
      steppers[i]->forceStop();
      stall_detected = true;
    }
    delay(2);
  }
  
  if (!stall_detected) {
    Serial.println("⚠️ Timeout SUP - pas de stall détecté");
  }
  
  long maxPos = steppers[i]->getCurrentPosition();
  Serial.printf("📍 Position SUP: %ld\n", maxPos);

  // ------------------ CALCUL CENTRE ET LIMITES ------------------
  long center = (minPos + maxPos) / 2;
  long range = maxPos - minPos;
  
  Serial.printf("📏 Course détectée: %ld pas (%.1f mm)\n", range, range * 0.1); // Assumant 0.1mm/pas
  
  // Aller au centre
  Serial.printf("🎯 Aller au centre: %ld\n", center);
  steppers[i]->moveTo(center);
  while (steppers[i]->isRunning()) delay(2);
  
  // Définir position 0 au centre
  steppers[i]->setCurrentPosition(0);
  
  // Calculer limites relatives au centre (TODO: utiliser MotorControl)
  // cfg[i].min_limit = minPos - center;
  // cfg[i].max_limit = maxPos - center;
  
  Serial.printf("✅ Homing terminé - Centre: %ld, Course: %ld pas\n", center, range);
  
  // Restaurer la configuration normale
  steppers[i]->setSpeedInHz(cfg[i].max_speed);
  steppers[i]->setAcceleration(cfg[i].max_accel);
  drivers[i]->rms_current(cfg[i].current_ma);
  
  Serial.println("🏠 HOMING SLIDE COMPLETED");
}

/**
 * @brief Configure le seuil StallGuard pour le slide
 */
void setSlideSGThreshold(uint8_t threshold) {
  slide_sg_threshold = threshold;
  if (drivers[SLIDE_INDEX]) {
    drivers[SLIDE_INDEX]->SGTHRS(slide_sg_threshold);
    cfg[SLIDE_INDEX].sgt = slide_sg_threshold;
    Serial.printf("🔧 Nouvelle SGTHRS (slide) = %d\n", slide_sg_threshold);
  }
}

/**
 * @brief Obtient le seuil StallGuard actuel du slide
 */
uint8_t getSlideSGThreshold() {
  return slide_sg_threshold;
}
