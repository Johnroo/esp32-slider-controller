/**
 * @file Homing.cpp
 * @brief Implémentation du module de homing avec StallGuard
 * @author Laurent Eyen
 * @date 2024
 */

#include "Homing.h"

//==================== Variables globales ====================
bool doAutoHomeSlide = false;     // lancer automatiquement au démarrage si true
bool homingInProgress = false;    // flag pour éviter les homing multiples

//==================== Fonctions du module ====================

/**
 * @brief Initialise le système de homing (StallGuard)
 */
void initHoming() {
  Serial.println("🏠 Initialisation du système de homing...");
  
  // Configuration du seuil StallGuard pour le slide
  if (drivers[SLIDE_INDEX]) {
    drivers[SLIDE_INDEX]->SGTHRS(SG_DETECT);
    cfg[SLIDE_INDEX].sgt = SG_DETECT;
    Serial.printf("✅ StallGuard configuré (seuil: %d)\n", SG_DETECT);
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

  // Protection contre les homing multiples
  if (homingInProgress) {
    Serial.println("⚠️ Homing déjà en cours, ignoré");
    return;
  }
  
  // Libération forcée du flag au cas où il serait bloqué
  homingInProgress = false;
  
  homingInProgress = true;
  Serial.printf("🏠 HOMING SLIDE START (StallGuard4) - Flag: %s\n", homingInProgress ? "true" : "false");
  Serial.printf("🔍 DEBUG: homingInProgress = %s, adresse = %p\n", homingInProgress ? "true" : "false", &homingInProgress);
  
  // Timeout de sécurité pour éviter les homing bloqués
  unsigned long homingStartTime = millis();

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
  
  // Configuration pour StallGuard4 (SpreadCycle requis pour le homing)
  drivers[i]->en_spreadCycle(true);   // SpreadCycle pour StallGuard4
  
  // Réduire le courant pour éviter les dégâts en cas de collision
  drivers[i]->rms_current(1600); // boost à 1800mA pour le homing
  
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
    
    // Vérifier timeout global
    if (millis() - homingStartTime > HOMING_TIMEOUT * 2) {
      Serial.println("⏰ TIMEOUT homing global - libération forcée du flag");
      homingInProgress = false;
      return;
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
  
  // Délai de stabilisation pour éviter les sauts de pas
  Serial.printf("⏳ Stabilisation %dms...\n", BACKOFF_DELAY);
  delay(BACKOFF_DELAY);

  // ------------------ PHASE SUP (vers butée supérieure) ------------------
  Serial.println("▶️ Vers butée SUP...");
  
  // Reconfigurer la vitesse et accélération pour la phase supérieure
  steppers[i]->setSpeedInHz(HOMING_SPEED);
  steppers[i]->setAcceleration(HOMING_ACCEL);
  Serial.printf("🔧 Reconfiguration phase SUP: vitesse=%d, accel=%d\n", HOMING_SPEED, HOMING_ACCEL);
  
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
    
    // Vérifier timeout global
    if (millis() - homingStartTime > HOMING_TIMEOUT * 2) {
      Serial.println("⏰ TIMEOUT homing global - libération forcée du flag");
      homingInProgress = false;
      return;
    }
    
    delay(2);
  }
  
  if (!stall_detected) {
    Serial.println("⚠️ Timeout SUP - pas de stall détecté");
  }
  
  long maxPos = steppers[i]->getCurrentPosition();
  Serial.printf("📍 Position SUP: %ld\n", maxPos);

  // Petit recul de la butée supérieure
  Serial.printf("↩️ Petit recul de %d pas...\n", BACKOFF_STEPS/2);
  steppers[i]->move(-BACKOFF_STEPS/2);
  while (steppers[i]->isRunning()) delay(2);
  delay(BACKOFF_DELAY/2); // Stabilisation plus courte

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
  
  // Calculer limites relatives au centre
  long minLimitRaw = minPos - center;
  long maxLimitRaw = maxPos - center;
  // Appliquer une marge de sécurité pour éviter les butées
  cfg[i].min_limit = minLimitRaw + SAFETY_STEPS;
  cfg[i].max_limit = maxLimitRaw - SAFETY_STEPS;
  Serial.printf("🛟 Safety margin appliquée (%d pas) -> min=%ld, max=%ld (raw min=%ld, raw max=%ld)\n",
                SAFETY_STEPS, cfg[i].min_limit, cfg[i].max_limit, minLimitRaw, maxLimitRaw);
 
  Serial.printf("✅ Homing terminé - Centre: %ld, Course: %ld pas\n", center, range);
  Serial.printf("📏 Limites mises à jour: min=%ld, max=%ld\n", cfg[i].min_limit, cfg[i].max_limit);
  
  // Restaurer la configuration normale
  steppers[i]->setSpeedInHz(cfg[i].max_speed);
  steppers[i]->setAcceleration(cfg[i].max_accel);
  drivers[i]->rms_current(cfg[i].current_ma);
  
  // IMPORTANT: Passer en StealthChop pour un meilleur fonctionnement du slider
  drivers[i]->en_spreadCycle(false);  // StealthChop pour le fonctionnement normal
  Serial.println("🔄 Mode StealthChop activé pour le fonctionnement normal");
  
  // Libérer le flag de protection
  homingInProgress = false;
  Serial.printf("🏠 HOMING SLIDE COMPLETED - Flag: %s\n", homingInProgress ? "true" : "false");
  Serial.printf("🔍 DEBUG: homingInProgress = %s, adresse = %p\n", homingInProgress ? "true" : "false", &homingInProgress);
}

/**
 * @brief Configure le seuil StallGuard pour le slide
 */
void setSlideSGThreshold(uint8_t threshold) {
  if (drivers[SLIDE_INDEX]) {
    drivers[SLIDE_INDEX]->SGTHRS(threshold);
    cfg[SLIDE_INDEX].sgt = threshold;
    Serial.printf("🔧 Nouvelle SGTHRS (slide) = %d\n", threshold);
  }
}

/**
 * @brief Obtient le seuil StallGuard actuel du slide
 */
uint8_t getSlideSGThreshold() {
  return SG_DETECT;
}

/**
 * @brief Vérifie si un homing est en cours
 */
bool isHomingInProgress() {
  return homingInProgress;
}
