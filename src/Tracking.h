/**
 * @file Tracking.h
 * @brief Module de gestion du suivi coordonné slide-pan/tilt
 * @author Laurent Eyen
 * @date 2024
 */

#ifndef TRACKING_H
#define TRACKING_H

#include <Arduino.h>
#include "Config.h"

//==================== Constantes de configuration ====================
// Mapping slide->pan/tilt supprimé (mode follow obsolète)

//==================== Structures de données ====================
struct AnchorMorph {
  bool active = false;
  long p0 = 0, t0 = 0;
  long p1 = 0, t1 = 0;
  uint32_t t0_ms = 0, T_ms = 0;
};

//==================== Variables globales ====================
extern AnchorMorph anchor_morph;

//==================== Fonctions du module ====================

/**
 * @brief Initialise le système de suivi
 */
void initTracking();

/**
 * @brief Met à jour le suivi coordonné slide-pan/tilt
 * @details Doit être appelée dans la loop principale
 */
void updateTracking();

// Fonctions follow obsolètes supprimées (panCompFromSlide, tiltCompFromSlide, refreshFollowAnchor, etc.)

/**
 * @brief Démarre l'interpolation d'ancre
 * @param p0 Position pan initiale
 * @param t0 Position tilt initiale
 * @param p1 Position pan finale
 * @param t1 Position tilt finale
 * @param duration Durée en millisecondes
 */
void startAnchorMorph(long p0, long t0, long p1, long t1, uint32_t duration);

/**
 * @brief Arrête l'interpolation d'ancre
 */
void stopAnchorMorph();

/**
 * @brief Vérifie si l'interpolation d'ancre est active
 * @return true si active, false sinon
 */
bool isAnchorMorphActive();

// Fonctions de mapping slide->pan/tilt supprimées (mode follow obsolète)

#endif // TRACKING_H
