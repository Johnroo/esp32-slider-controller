/**
 * @file Tracking.cpp
 * @brief Implémentation du module de suivi coordonné slide-pan/tilt
 * @author Laurent Eyen
 * @date 2024
 */

#include "Tracking.h"
#include "MotorControl.h"
#include "Utils.h"

//==================== Constantes de configuration ====================
// Utilise les constantes par défaut du module Config
long PAN_AT_SLIDE_MIN  = DEFAULT_PAN_AT_SLIDE_MIN;
long PAN_AT_SLIDE_MAX  = DEFAULT_PAN_AT_SLIDE_MAX;
long TILT_AT_SLIDE_MIN = DEFAULT_TILT_AT_SLIDE_MIN;
long TILT_AT_SLIDE_MAX = DEFAULT_TILT_AT_SLIDE_MAX;

//==================== Variables globales ====================
Follow follow;
AnchorMorph anchor_morph;

// Les fonctions utilitaires sont définies dans Presets.cpp

//==================== Fonctions du module ====================

/**
 * @brief Initialise le système de suivi
 */
void initTracking() {
  Serial.println("🎯 Initialisation du système de suivi...");
  
  // Initialiser les structures
  follow.enabled = false;
  follow.valid = false;
  follow.pan_anchor = 0;
  follow.tilt_anchor = 0;
  
  anchor_morph.active = false;
  anchor_morph.p0 = 0;
  anchor_morph.t0 = 0;
  anchor_morph.p1 = 0;
  anchor_morph.t1 = 0;
  anchor_morph.t0_ms = 0;
  anchor_morph.T_ms = 0;
  
  Serial.println("✅ Système de suivi initialisé");
}

/**
 * @brief Met à jour le suivi coordonné slide-pan/tilt
 */
void updateTracking() {
  uint32_t now = millis();
  
  // Interpolation d'ancre min-jerk pour recall autour de l'autopan
  if (anchor_morph.active) {
    float tau = (float)(now - anchor_morph.t0_ms) / (float)anchor_morph.T_ms;
    if (tau >= 1.0f) { 
      tau = 1.0f; 
      anchor_morph.active = false; 
    }
    float s = s_minjerk(tau);
    follow.pan_anchor  = lround( lerp((float)anchor_morph.p0, (float)anchor_morph.p1, s) );
    follow.tilt_anchor = lround( lerp((float)anchor_morph.t0, (float)anchor_morph.t1, s) );
    follow.valid = true; // ancres imposées
  }
}

/**
 * @brief Calcule la compensation pan à partir de la position slide
 */
long panCompFromSlide(long slide) {
  float u = (float)(slide - cfg[3].min_limit) / (float)(cfg[3].max_limit - cfg[3].min_limit);
  return (long) lround(lerp(PAN_AT_SLIDE_MIN, PAN_AT_SLIDE_MAX, clampF(u, 0, 1)));
}

/**
 * @brief Calcule la compensation tilt à partir de la position slide
 */
long tiltCompFromSlide(long slide) {
  float u = (float)(slide - cfg[3].min_limit) / (float)(cfg[3].max_limit - cfg[3].min_limit);
  return (long) lround(lerp(TILT_AT_SLIDE_MIN, TILT_AT_SLIDE_MAX, clampF(u, 0, 1)));
}

/**
 * @brief Rafraîchit l'ancre de suivi
 */
void refreshFollowAnchor() {
  long s = steppers[3]->targetPos();
  follow.pan_anchor  = steppers[0]->targetPos() - panCompFromSlide(s);
  follow.tilt_anchor = steppers[1]->targetPos() - tiltCompFromSlide(s);
  follow.valid = true;
}

/**
 * @brief Active le mode de suivi
 */
void startTracking() {
  follow.enabled = true;
  follow.valid = false; // Force le rafraîchissement de l'ancre
  Serial.println("🎯 Suivi activé");
}

/**
 * @brief Désactive le mode de suivi
 */
void stopTracking() {
  follow.enabled = false;
  follow.valid = false;
  Serial.println("⏹️ Suivi désactivé");
}

/**
 * @brief Vérifie si le suivi est activé
 */
bool isTrackingEnabled() {
  return follow.enabled;
}

/**
 * @brief Démarre l'interpolation d'ancre
 */
void startAnchorMorph(long p0, long t0, long p1, long t1, uint32_t duration) {
  anchor_morph.active = true;
  anchor_morph.p0 = p0;
  anchor_morph.t0 = t0;
  anchor_morph.p1 = p1;
  anchor_morph.t1 = t1;
  anchor_morph.t0_ms = millis();
  anchor_morph.T_ms = duration;
  
  Serial.printf("🔄 Interpolation d'ancre démarrée (T=%u ms)\n", duration);
}

/**
 * @brief Arrête l'interpolation d'ancre
 */
void stopAnchorMorph() {
  anchor_morph.active = false;
  Serial.println("⏹️ Interpolation d'ancre arrêtée");
}

/**
 * @brief Vérifie si l'interpolation d'ancre est active
 */
bool isAnchorMorphActive() {
  return anchor_morph.active;
}

/**
 * @brief Définit les paramètres de mapping pan
 */
void setPanMapping(long min, long max) {
  PAN_AT_SLIDE_MIN = min;
  PAN_AT_SLIDE_MAX = max;
  Serial.printf("🎯 Mapping Pan mis à jour: min=%ld, max=%ld\n", min, max);
}

/**
 * @brief Définit les paramètres de mapping tilt
 */
void setTiltMapping(long min, long max) {
  TILT_AT_SLIDE_MIN = min;
  TILT_AT_SLIDE_MAX = max;
  Serial.printf("🎯 Mapping Tilt mis à jour: min=%ld, max=%ld\n", min, max);
}

/**
 * @brief Obtient les paramètres de mapping pan
 */
void getPanMapping(long &min, long &max) {
  min = PAN_AT_SLIDE_MIN;
  max = PAN_AT_SLIDE_MAX;
}

/**
 * @brief Obtient les paramètres de mapping tilt
 */
void getTiltMapping(long &min, long &max) {
  min = TILT_AT_SLIDE_MIN;
  max = TILT_AT_SLIDE_MAX;
}
