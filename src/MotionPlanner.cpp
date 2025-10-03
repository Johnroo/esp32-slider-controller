/**
 * @file MotionPlanner.cpp
 * @brief Implémentation du planificateur de mouvements synchronisés
 * @author Laurent Eyen
 * @date 2024
 */

#include "MotionPlanner.h"
#include "MotorControl.h"
#include "Tracking.h"
#include "Presets.h"

//==================== Variables globales ====================
SyncMove sync_move;

//==================== Variables statiques ====================
static uint32_t defaultMoveDuration = 2000; // 2 secondes par défaut

//==================== Fonctions du module ====================

/**
 * @brief Initialise le planificateur de mouvements
 */
void initMotionPlanner() {
  Serial.println("🎬 Initialisation du planificateur de mouvements...");
  
  // Initialiser la structure
  sync_move.active = false;
  sync_move.t0_ms = 0;
  sync_move.T_ms = defaultMoveDuration;
  
  for (int i = 0; i < NUM_MOTORS; i++) {
    sync_move.start[i] = 0;
    sync_move.goal_base[i] = 0;
  }
  
  Serial.println("✅ Planificateur de mouvements initialisé");
}

/**
 * @brief Met à jour l'exécution des mouvements planifiés
 */
void updateMotionPlanner() {
  if (!sync_move.active) return;
  
  uint32_t now = millis();
  float tau = (float)(now - sync_move.t0_ms) / (float)sync_move.T_ms;
  
  if (tau >= 1.0f) {
    // Fin de mouvement
    sync_move.active = false;
    follow.valid = false;   // re-anchor next time
    
    // Rephase AB après GOTO_THEN_RESUME pour éviter un saut
    // Note: Cette logique sera gérée dans le module principal si nécessaire
    
    Serial.println("✅ Mouvement synchronisé terminé");
    return;
  }
  
  // Profil minimum-jerk
  float s = s_minjerk(tau);
  
  // Slide de référence (pour couplage)
  long slide_ref = (long)lround( sync_move.start[3] + (sync_move.goal_base[3] - sync_move.start[3]) * s );
  slide_ref = clampL(slide_ref, cfg[3].min_limit, cfg[3].max_limit);
  
  // Compensations en fonction du slide + offsets joystick (toujours actifs)
  long pan_comp  = panCompFromSlide(slide_ref);
  long tilt_comp = tiltCompFromSlide(slide_ref);
  
  // TODO: Implémenter les offsets correctement
  long pan_goal  = sync_move.goal_base[0] + pan_comp; // + eff_pan_offset();
  long tilt_goal = sync_move.goal_base[1] + tilt_comp; // + eff_tilt_offset();
  long zoom_goal = sync_move.goal_base[2];
  long slide_goal= sync_move.goal_base[3];
  
  // Cibles "à l'instant" suivant s(t)
  long P = (long)lround( sync_move.start[0] + (pan_goal  - sync_move.start[0]) * s );
  long T = (long)lround( sync_move.start[1] + (tilt_goal - sync_move.start[1]) * s );
  long Z = (long)lround( sync_move.start[2] + (zoom_goal - sync_move.start[2]) * s );
  long S = (long)lround( sync_move.start[3] + (slide_goal- sync_move.start[3]) * s );
  
  // Clip limites
  P = clampL(P, cfg[0].min_limit, cfg[0].max_limit);
  T = clampL(T, cfg[1].min_limit, cfg[1].max_limit);
  Z = clampL(Z, cfg[2].min_limit, cfg[2].max_limit);
  S = clampL(S, cfg[3].min_limit, cfg[3].max_limit);
  
  // Envoyer aux moteurs
  steppers[0]->moveTo(P);
  steppers[1]->moveTo(T);
  steppers[2]->moveTo(Z);
  steppers[3]->moveTo(S);
}

/**
 * @brief Planifie un mouvement synchronisé vers des positions cibles
 */
bool planSynchronizedMove(const long targetPositions[NUM_MOTORS], uint32_t durationMs) {
  if (sync_move.active) {
    Serial.println("⚠️ Mouvement synchronisé déjà en cours");
    return false;
  }
  
  // Enregistrer les positions de départ
  for (int i = 0; i < NUM_MOTORS; i++) {
    sync_move.start[i] = steppers[i]->getCurrentPosition();
    sync_move.goal_base[i] = clampL(targetPositions[i], cfg[i].min_limit, cfg[i].max_limit);
  }
  
  // Calculer la durée optimale
  sync_move.T_ms = calculateOptimalDuration(sync_move.start, sync_move.goal_base, durationMs);
  sync_move.t0_ms = millis();
  sync_move.active = true;
  
  Serial.printf("🎬 Mouvement synchronisé planifié: P:%ld T:%ld Z:%ld S:%ld en %u ms\n",
                sync_move.goal_base[0], sync_move.goal_base[1],
                sync_move.goal_base[2], sync_move.goal_base[3], sync_move.T_ms);
  
  return true;
}

/**
 * @brief Planifie un mouvement synchronisé vers une position slide
 */
bool planSlideMove(long slidePosition, float durationSec) {
  long targetPositions[NUM_MOTORS];
  
  // Obtenir les positions actuelles
  for (int i = 0; i < NUM_MOTORS; i++) {
    targetPositions[i] = steppers[i]->getCurrentPosition();
  }
  
  // Modifier seulement la position du slide
  targetPositions[3] = slidePosition;
  
  uint32_t durationMs = (uint32_t)lround(durationSec * 1000.0f);
  return planSynchronizedMove(targetPositions, durationMs);
}

/**
 * @brief Arrête le mouvement synchronisé en cours
 */
void stopSynchronizedMove() {
  if (sync_move.active) {
    sync_move.active = false;
    Serial.println("⏹️ Mouvement synchronisé arrêté");
  }
}

/**
 * @brief Vérifie si un mouvement synchronisé est en cours
 */
bool isSynchronizedMoveActive() {
  return sync_move.active;
}

/**
 * @brief Calcule la durée optimale pour un mouvement
 */
uint32_t calculateOptimalDuration(const long startPositions[NUM_MOTORS], 
                                  const long targetPositions[NUM_MOTORS], 
                                  uint32_t requestedDurationMs) {
  double T = requestedDurationMs / 1000.0;
  
  for (;;) {
    bool ok = true;
    for (int i = 0; i < NUM_MOTORS; i++) {
      double d = fabs((double)targetPositions[i] - (double)startPositions[i]);
      double v_max = cfg[i].max_speed;
      double t_needed = d / v_max;
      
      if (t_needed > T) {
        T = t_needed;
        ok = false;
      }
    }
    if (ok) break;
  }
  
  return (uint32_t)lround(T * 1000.0);
}

/**
 * @brief Intègre les offsets dans la cible du mouvement en cours
 */
void bakeOffsetsIntoCurrentMove(long panOffset, long tiltOffset) {
  if (sync_move.active) {
    sync_move.goal_base[0] = clampL(sync_move.goal_base[0] + panOffset, cfg[0].min_limit, cfg[0].max_limit);
    sync_move.goal_base[1] = clampL(sync_move.goal_base[1] + tiltOffset, cfg[1].min_limit, cfg[1].max_limit);
    Serial.printf("🍞 Offsets intégrés: pan=%ld, tilt=%ld\n", sync_move.goal_base[0], sync_move.goal_base[1]);
  }
}

/**
 * @brief Obtient l'état du mouvement synchronisé
 */
void getSynchronizedMoveStatus(bool &isActive, float &progress, uint32_t &remainingTimeMs) {
  isActive = sync_move.active;
  
  if (!isActive) {
    progress = 0.0f;
    remainingTimeMs = 0;
    return;
  }
  
  uint32_t now = millis();
  uint32_t elapsed = now - sync_move.t0_ms;
  
  if (elapsed >= sync_move.T_ms) {
    progress = 1.0f;
    remainingTimeMs = 0;
  } else {
    progress = (float)elapsed / (float)sync_move.T_ms;
    remainingTimeMs = sync_move.T_ms - elapsed;
  }
}

/**
 * @brief Obtient les positions de départ du mouvement en cours
 */
void getStartPositions(long positions[NUM_MOTORS]) {
  for (int i = 0; i < NUM_MOTORS; i++) {
    positions[i] = sync_move.start[i];
  }
}

/**
 * @brief Obtient les positions cibles du mouvement en cours
 */
void getTargetPositions(long positions[NUM_MOTORS]) {
  for (int i = 0; i < NUM_MOTORS; i++) {
    positions[i] = sync_move.goal_base[i];
  }
}

/**
 * @brief Définit la durée par défaut pour les mouvements synchronisés
 */
void setDefaultMoveDuration(uint32_t durationMs) {
  defaultMoveDuration = durationMs;
  Serial.printf("⏱️ Durée par défaut définie: %u ms\n", durationMs);
}

/**
 * @brief Obtient la durée par défaut pour les mouvements synchronisés
 */
uint32_t getDefaultMoveDuration() {
  return defaultMoveDuration;
}
