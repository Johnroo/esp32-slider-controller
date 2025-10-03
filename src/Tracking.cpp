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
// Variables de mapping slide->pan/tilt supprimées (mode follow obsolète)

//==================== Variables globales ====================
AnchorMorph anchor_morph;

// Les fonctions utilitaires sont définies dans Presets.cpp

//==================== Fonctions du module ====================

/**
 * @brief Initialise le système de suivi
 */
void initTracking() {
  Serial.println("🎯 Initialisation du système de suivi...");
  
  // Initialiser la structure anchor_morph
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
    // Note: Les ancres follow ont été supprimées, cette logique n'est plus utilisée
  }
}

// Fonctions follow obsolètes supprimées (panCompFromSlide, tiltCompFromSlide, refreshFollowAnchor, startTracking, stopTracking, isTrackingEnabled)

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

// Implémentations des fonctions de mapping slide->pan/tilt supprimées (mode follow obsolète)
