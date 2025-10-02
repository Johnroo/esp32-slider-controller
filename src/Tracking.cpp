/**
 * @file Tracking.cpp
 * @brief Implémentation du module de tracking slide-pan/tilt
 */

#include "Tracking.h"

//==================== Variables globales ====================
long PAN_AT_SLIDE_MIN = +800;   // Position PAN à gauche du slide
long PAN_AT_SLIDE_MAX = -800;   // Position PAN à droite du slide
long TILT_AT_SLIDE_MIN = 0;     // Position TILT à gauche du slide
long TILT_AT_SLIDE_MAX = 0;     // Position TILT à droite du slide

Follow follow;
AnchorMorph anchor_morph;

//==================== Fonctions utilitaires ====================
static inline long clampL(long v, long vmin, long vmax){ return v < vmin ? vmin : (v > vmax ? vmax : v); }
static inline float clampF(float v, float vmin, float vmax){ return v < vmin ? vmin : (v > vmax ? vmax : v); }
static inline float lerp(float a, float b, float u){ return a + (b - a) * u; }

//==================== Fonctions d'initialisation ====================
void initTracking() {
  Serial.println("🎯 Initialisation du module Tracking...");
  
  // Initialiser les structures
  follow.enabled = false;
  follow.valid = false;
  follow.pan_anchor = 0;
  follow.tilt_anchor = 0;
  
  anchor_morph.active = false;
  anchor_morph.t0_ms = 0;
  anchor_morph.T_ms = 1000;
  anchor_morph.p0 = anchor_morph.p1 = 0;
  anchor_morph.t0 = anchor_morph.t1 = 0;
  
  Serial.println("✅ Module Tracking initialisé");
}

void setSlideMapping(long pan_min, long pan_max, long tilt_min, long tilt_max) {
  PAN_AT_SLIDE_MIN = pan_min;
  PAN_AT_SLIDE_MAX = pan_max;
  TILT_AT_SLIDE_MIN = tilt_min;
  TILT_AT_SLIDE_MAX = tilt_max;
  
  Serial.printf("🎯 Mapping slide->pan/tilt: PAN[%ld,%ld] TILT[%ld,%ld]\n", 
                pan_min, pan_max, tilt_min, tilt_max);
}

//==================== Fonctions de mapping ====================
long panCompFromSlide(long slide) {
  float u = (float)(slide - cfg[3].min_limit) / (float)(cfg[3].max_limit - cfg[3].min_limit);
  return (long) lround(lerp(PAN_AT_SLIDE_MIN, PAN_AT_SLIDE_MAX, clampF(u,0,1)));
}

long tiltCompFromSlide(long slide) {
  float u = (float)(slide - cfg[3].min_limit) / (float)(cfg[3].max_limit - cfg[3].min_limit);
  return (long) lround(lerp(TILT_AT_SLIDE_MIN, TILT_AT_SLIDE_MAX, clampF(u,0,1)));
}

//==================== Fonctions de gestion des ancres ====================
void refreshAnchor() {
  long s = slidePos; // Position actuelle du slide
  follow.pan_anchor = panPos - panCompFromSlide(s);
  follow.tilt_anchor = tiltPos - tiltCompFromSlide(s);
  follow.valid = true;
  
  Serial.printf("🎯 Ancres actualisées: PAN=%ld TILT=%ld (slide=%ld)\n", 
                follow.pan_anchor, follow.tilt_anchor, s);
}

void setAnchor(long pan_anchor, long tilt_anchor) {
  follow.pan_anchor = pan_anchor;
  follow.tilt_anchor = tilt_anchor;
  follow.valid = true;
  
  Serial.printf("🎯 Ancres définies: PAN=%ld TILT=%ld\n", pan_anchor, tilt_anchor);
}

void invalidateAnchor() {
  follow.valid = false;
  Serial.println("🎯 Ancres invalidées");
}

bool isAnchorValid() {
  return follow.valid;
}

//==================== Fonctions de contrôle du suivi ====================
void startTracking() {
  follow.enabled = true;
  follow.valid = false; // Force le recalcul des ancres
  Serial.println("🎯 Suivi coordonné activé");
}

void stopTracking() {
  follow.enabled = false;
  Serial.println("🎯 Suivi coordonné désactivé");
}

bool isTrackingEnabled() {
  return follow.enabled;
}

void setTrackingEnabled(bool enabled) {
  follow.enabled = enabled;
  if (!enabled) {
    follow.valid = false;
  }
  Serial.printf("🎯 Suivi coordonné %s\n", enabled ? "activé" : "désactivé");
}

//==================== Fonctions de morphing d'ancres ====================
void startAnchorMorph(uint32_t duration_ms, long pan_start, long pan_end, long tilt_start, long tilt_end) {
  anchor_morph.active = true;
  anchor_morph.t0_ms = millis();
  anchor_morph.T_ms = duration_ms;
  anchor_morph.p0 = pan_start;
  anchor_morph.p1 = pan_end;
  anchor_morph.t0 = tilt_start;
  anchor_morph.t1 = tilt_end;
  
  Serial.printf("🎯 Morphing d'ancres démarré: %.1fs\n", duration_ms / 1000.0f);
}

void stopAnchorMorph() {
  anchor_morph.active = false;
  Serial.println("🎯 Morphing d'ancres arrêté");
}

bool isAnchorMorphActive() {
  return anchor_morph.active;
}

//==================== Fonctions de mise à jour ====================
bool updateTracking(long slide_position, bool in_recall) {
  if (!follow.enabled) {
    return false;
  }
  
  uint32_t now = millis();
  
  // Gestion du morphing d'ancres
  if (anchor_morph.active) {
    float tau = (float)(now - anchor_morph.t0_ms) / (float)anchor_morph.T_ms;
    if (tau >= 1.0f) { 
      tau = 1.0f; 
      anchor_morph.active = false; 
    }
    
    float s = tau * tau * (3.0f - 2.0f * tau); // smoothstep
    follow.pan_anchor = lround(lerp((float)anchor_morph.p0, (float)anchor_morph.p1, s));
    follow.tilt_anchor = lround(lerp((float)anchor_morph.t0, (float)anchor_morph.t1, s));
    follow.valid = true; // ancres imposées
  }
  
  // Actualiser les ancres si nécessaire
  if (!follow.valid) {
    refreshAnchor();
  }
  
  // Calculer les compensations
  long pComp = panCompFromSlide(slide_position);
  long tComp = tiltCompFromSlide(slide_position);
  
  // Appliquer les offsets (fonctions externes)
  extern long active_pan_offset(bool recall_phase);
  extern long active_tilt_offset(bool recall_phase);
  
  long Pgoal = clampL(follow.pan_anchor + pComp + active_pan_offset(in_recall), 
                      cfg[0].min_limit, cfg[0].max_limit);
  long Tgoal = clampL(follow.tilt_anchor + tComp + active_tilt_offset(in_recall), 
                      cfg[1].min_limit, cfg[1].max_limit);
  
  // Envoyer aux moteurs
  moveMotorTo(0, Pgoal); // PAN
  moveMotorTo(1, Tgoal); // TILT
  
  return true;
}

bool updateTrackingForJog(long slide_goal, bool in_recall) {
  if (!follow.enabled) {
    return false;
  }
  
  // Actualiser les ancres si nécessaire
  if (!follow.valid) {
    refreshAnchor();
  }
  
  // Calculer les compensations
  long pComp = panCompFromSlide(slide_goal);
  long tComp = tiltCompFromSlide(slide_goal);
  
  // Appliquer les offsets (fonctions externes)
  extern long active_pan_offset(bool recall_phase);
  extern long active_tilt_offset(bool recall_phase);
  
  long Pgoal = clampL(follow.pan_anchor + pComp + active_pan_offset(in_recall), 
                      cfg[0].min_limit, cfg[0].max_limit);
  long Tgoal = clampL(follow.tilt_anchor + tComp + active_tilt_offset(in_recall), 
                      cfg[1].min_limit, cfg[1].max_limit);
  
  // Envoyer aux moteurs
  moveMotorTo(0, Pgoal); // PAN
  moveMotorTo(1, Tgoal); // TILT
  
  return true;
}

//==================== Fonctions utilitaires ====================
long getPanAnchor() {
  return follow.pan_anchor;
}

long getTiltAnchor() {
  return follow.tilt_anchor;
}

void getSlideMapping(long &pan_min, long &pan_max, long &tilt_min, long &tilt_max) {
  pan_min = PAN_AT_SLIDE_MIN;
  pan_max = PAN_AT_SLIDE_MAX;
  tilt_min = TILT_AT_SLIDE_MIN;
  tilt_max = TILT_AT_SLIDE_MAX;
}
