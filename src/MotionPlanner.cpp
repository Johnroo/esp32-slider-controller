/**
 * @file MotionPlanner.cpp
 * @brief Implémentation du module de planification des mouvements synchronisés
 */

#include "MotionPlanner.h"
#include "Tracking.h"
#include "Presets.h"
#include <ArduinoJson.h>

//==================== Variables globales ====================
SyncMove sync_move;

// Configuration par défaut
static float max_speeds[NUM_MOTORS] = {1000.0f, 1000.0f, 1000.0f, 1000.0f}; // steps/s
static float max_accelerations[NUM_MOTORS] = {2000.0f, 2000.0f, 2000.0f, 2000.0f}; // steps/s²

//==================== Fonctions utilitaires ====================
static inline long clampL(long v, long vmin, long vmax){ return v < vmin ? vmin : (v > vmax ? vmax : v); }

// Fonction de courbe minimum jerk (déplacée de main.cpp)
static inline float s_minjerk(float tau) {
  return tau * tau * (3.0f - 2.0f * tau);
}

//==================== Fonctions d'initialisation ====================
void initMotionPlanner() {
  Serial.println("🎬 Initialisation du module MotionPlanner...");
  
  // Initialiser la structure sync_move
  sync_move.active = false;
  sync_move.t0_ms = 0;
  sync_move.T_ms = 0;
  
  for(int i = 0; i < NUM_MOTORS; i++) {
    sync_move.start[i] = 0;
    sync_move.goal_base[i] = 0;
  }
  
  Serial.println("✅ Module MotionPlanner initialisé");
}

void resetMotionPlanner() {
  sync_move.active = false;
  sync_move.t0_ms = 0;
  sync_move.T_ms = 0;
  Serial.println("🔄 Planificateur réinitialisé");
}

//==================== Fonctions de planification ====================
uint32_t pickDurationMsForDeltas(const long start[NUM_MOTORS], const long goal[NUM_MOTORS], uint32_t T_req_ms) {
  double T = T_req_ms / 1000.0;
  
  for(;;) {
    bool ok = true;
    
    for(int i = 0; i < NUM_MOTORS; i++) {
      double d = fabs((double)goal[i] - (double)start[i]);
      if (d > 0) {
        double v_max = max_speeds[i];
        double a_max = max_accelerations[i];
        
        // Calcul du temps minimum avec accélération
        double t_accel = v_max / a_max;
        double d_accel = 0.5 * a_max * t_accel * t_accel;
        
        double t_total;
        if (d <= 2 * d_accel) {
          // Mouvement triangulaire (pas de phase constante)
          t_total = 2 * sqrt(d / a_max);
        } else {
          // Mouvement trapézoïdal
          double d_constant = d - 2 * d_accel;
          double t_constant = d_constant / v_max;
          t_total = 2 * t_accel + t_constant;
        }
        
        if (t_total > T) {
          T = t_total;
          ok = false;
        }
      }
    }
    
    if (ok) break;
  }
  
  return (uint32_t)lround(T * 1000.0);
}

bool planSynchronizedMove(const long target_positions[NUM_MOTORS], uint32_t duration_ms) {
  if (sync_move.active) {
    Serial.println("⚠️ Mouvement déjà en cours, impossible de planifier un nouveau mouvement");
    return false;
  }
  
  // Enregistrer les positions de départ
  for(int ax = 0; ax < NUM_MOTORS; ax++) {
    sync_move.start[ax] = steppers[ax]->getCurrentPosition();
    sync_move.goal_base[ax] = clampL(target_positions[ax], cfg[ax].min_limit, cfg[ax].max_limit);
  }
  
  // Calculer la durée optimale
  sync_move.T_ms = pickDurationMsForDeltas(sync_move.start, sync_move.goal_base, duration_ms);
  sync_move.t0_ms = millis();
  sync_move.active = true;
  
  Serial.printf("🎬 Mouvement synchronisé planifié: P:%ld T:%ld Z:%ld S:%ld en %u ms\n",
                sync_move.goal_base[0], sync_move.goal_base[1],
                sync_move.goal_base[2], sync_move.goal_base[3], sync_move.T_ms);
  
  return true;
}

bool planRelativeMove(const long relative_positions[NUM_MOTORS], uint32_t duration_ms) {
  long target_positions[NUM_MOTORS];
  
  for(int i = 0; i < NUM_MOTORS; i++) {
    target_positions[i] = steppers[i]->getCurrentPosition() + relative_positions[i];
  }
  
  return planSynchronizedMove(target_positions, duration_ms);
}

bool planSingleAxisMove(uint8_t axis, long target_position, uint32_t duration_ms) {
  if (axis >= NUM_MOTORS) {
    Serial.printf("❌ Axe invalide: %d\n", axis);
    return false;
  }
  
  long target_positions[NUM_MOTORS];
  
  // Garder les positions actuelles pour les autres axes
  for(int i = 0; i < NUM_MOTORS; i++) {
    target_positions[i] = steppers[i]->getCurrentPosition();
  }
  
  // Modifier seulement l'axe spécifié
  target_positions[axis] = target_position;
  
  return planSynchronizedMove(target_positions, duration_ms);
}

//==================== Fonctions de contrôle du planificateur ====================
bool startPlannedMove() {
  if (!sync_move.active) {
    Serial.println("⚠️ Aucun mouvement planifié à démarrer");
    return false;
  }
  
  sync_move.t0_ms = millis();
  Serial.println("▶️ Démarrage du mouvement planifié");
  return true;
}

void stopPlannedMove() {
  sync_move.active = false;
  Serial.println("⏹️ Arrêt du mouvement planifié");
}

void pausePlannedMove() {
  if (sync_move.active) {
    // Pour l'instant, on arrête complètement
    // TODO: Implémenter une vraie pause avec sauvegarde de l'état
    sync_move.active = false;
    Serial.println("⏸️ Pause du mouvement planifié");
  }
}

void resumePlannedMove() {
  if (!sync_move.active) {
    Serial.println("⚠️ Aucun mouvement en pause à reprendre");
  }
}

//==================== Fonctions d'état ====================
bool isMoveActive() {
  return sync_move.active;
}

bool isMovePaused() {
  // Pour l'instant, pas de pause implémentée
  return false;
}

float getMoveProgress() {
  if (!sync_move.active || sync_move.T_ms == 0) {
    return 0.0f;
  }
  
  uint32_t elapsed = millis() - sync_move.t0_ms;
  return min(1.0f, (float)elapsed / (float)sync_move.T_ms);
}

uint32_t getRemainingTime() {
  if (!sync_move.active) {
    return 0;
  }
  
  uint32_t elapsed = millis() - sync_move.t0_ms;
  return (elapsed >= sync_move.T_ms) ? 0 : (sync_move.T_ms - elapsed);
}

uint32_t getTotalDuration() {
  return sync_move.T_ms;
}

//==================== Fonctions de mise à jour ====================
bool updateMotionPlanner(uint32_t now_ms) {
  if (!sync_move.active) {
    return false;
  }
  
  float tau = (float)(now_ms - sync_move.t0_ms) / (float)sync_move.T_ms;
  
  if (tau >= 1.0f) {
    // Fin du mouvement
    sync_move.active = false;
    Serial.println("✅ Mouvement synchronisé terminé");
    
    // S'assurer que tous les moteurs sont à leur position finale
    for(int i = 0; i < NUM_MOTORS; i++) {
      moveMotorTo(i, sync_move.goal_base[i]);
    }
    
    return false;
  }
  
  // Utiliser la courbe minimum jerk pour une interpolation plus douce
  float s = s_minjerk(tau);
  
  // Calculer les positions interpolées
  long pan_goal = sync_move.goal_base[0];
  long tilt_goal = sync_move.goal_base[1];
  long zoom_goal = sync_move.goal_base[2];
  long slide_goal = sync_move.goal_base[3];
  
  // Appliquer les compensations slide->pan/tilt si le tracking est actif
  if (isTrackingEnabled()) {
    // Slide de référence (pour couplage)
    long slide_ref = (long)lround(sync_move.start[3] + (sync_move.goal_base[3] - sync_move.start[3]) * s);
    slide_ref = clampL(slide_ref, cfg[3].min_limit, cfg[3].max_limit);
    
    // Compensations en fonction du slide + offsets joystick (toujours actifs)
    long pan_comp = panCompFromSlide(slide_ref);
    long tilt_comp = tiltCompFromSlide(slide_ref);
    
    // Appliquer les offsets (en mode recall)
    pan_goal = sync_move.goal_base[0] + pan_comp + active_pan_offset(true);
    tilt_goal = sync_move.goal_base[1] + tilt_comp + active_tilt_offset(true);
  }
  
  // Cibles "à l'instant" suivant s(t)
  long P = (long)lround(sync_move.start[0] + (pan_goal - sync_move.start[0]) * s);
  long T = (long)lround(sync_move.start[1] + (tilt_goal - sync_move.start[1]) * s);
  long Z = (long)lround(sync_move.start[2] + (zoom_goal - sync_move.start[2]) * s);
  long S = (long)lround(sync_move.start[3] + (slide_goal - sync_move.start[3]) * s);
  
  // Appliquer les limites
  P = clampL(P, cfg[0].min_limit, cfg[0].max_limit);
  T = clampL(T, cfg[1].min_limit, cfg[1].max_limit);
  Z = clampL(Z, cfg[2].min_limit, cfg[2].max_limit);
  S = clampL(S, cfg[3].min_limit, cfg[3].max_limit);
  
  // Envoyer aux moteurs
  moveMotorTo(0, P); // PAN
  moveMotorTo(1, T); // TILT
  moveMotorTo(2, Z); // ZOOM
  moveMotorTo(3, S); // SLIDE
  
  return true;
}

bool updateMotionPlanner() {
  return updateMotionPlanner(millis());
}

//==================== Fonctions de configuration ====================
void setMaxSpeeds(const float max_speeds_config[NUM_MOTORS]) {
  for(int i = 0; i < NUM_MOTORS; i++) {
    max_speeds[i] = max_speeds_config[i];
  }
  Serial.println("⚙️ Vitesses maximales mises à jour");
}

void setMaxAccelerations(const float max_accelerations_config[NUM_MOTORS]) {
  for(int i = 0; i < NUM_MOTORS; i++) {
    max_accelerations[i] = max_accelerations_config[i];
  }
  Serial.println("⚙️ Accélérations maximales mises à jour");
}

void getMaxSpeeds(float max_speeds_out[NUM_MOTORS]) {
  for(int i = 0; i < NUM_MOTORS; i++) {
    max_speeds_out[i] = max_speeds[i];
  }
}

void getMaxAccelerations(float max_accelerations_out[NUM_MOTORS]) {
  for(int i = 0; i < NUM_MOTORS; i++) {
    max_accelerations_out[i] = max_accelerations[i];
  }
}

//==================== Fonctions utilitaires ====================
void getStartPositions(long start_positions[NUM_MOTORS]) {
  for(int i = 0; i < NUM_MOTORS; i++) {
    start_positions[i] = sync_move.start[i];
  }
}

void getTargetPositions(long target_positions[NUM_MOTORS]) {
  for(int i = 0; i < NUM_MOTORS; i++) {
    target_positions[i] = sync_move.goal_base[i];
  }
}

double getTotalDistance() {
  double total = 0.0;
  for(int i = 0; i < NUM_MOTORS; i++) {
    total += abs(sync_move.goal_base[i] - sync_move.start[i]);
  }
  return total;
}

long getAxisDistance(uint8_t axis) {
  if (axis >= NUM_MOTORS) return 0;
  return abs(sync_move.goal_base[axis] - sync_move.start[axis]);
}

String getPlannerStatus() {
  DynamicJsonDocument doc(512);
  
  doc["active"] = sync_move.active;
  doc["progress"] = getMoveProgress();
  doc["duration_ms"] = sync_move.T_ms;
  doc["elapsed_ms"] = sync_move.active ? (millis() - sync_move.t0_ms) : 0;
  doc["remaining_ms"] = getRemainingTime();
  
  JsonArray start = doc.createNestedArray("start");
  JsonArray target = doc.createNestedArray("target");
  
  for(int i = 0; i < NUM_MOTORS; i++) {
    start.add(sync_move.start[i]);
    target.add(sync_move.goal_base[i]);
  }
  
  String result;
  serializeJson(doc, result);
  return result;
}
