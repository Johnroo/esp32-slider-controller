/**
 * @file Tracking.h
 * @brief Module de gestion du suivi coordonné slide-pan/tilt
 * @author Laurent Eyen
 * @date 2024
 */

#ifndef TRACKING_H
#define TRACKING_H

#include <Arduino.h>

//==================== Constantes de configuration ====================
// Mapping linéaire slide -> compensation pan/tilt (en steps)
extern long PAN_AT_SLIDE_MIN;   // ex: +800 à gauche
extern long PAN_AT_SLIDE_MAX;   // ex: -800 à droite
extern long TILT_AT_SLIDE_MIN;  // ex: 0
extern long TILT_AT_SLIDE_MAX;  // ex: 0

//==================== Structures de données ====================
struct Follow {
  bool enabled = true;
  bool valid   = false;
  long pan_anchor  = 0;
  long tilt_anchor = 0;
};

struct AnchorMorph {
  bool active = false;
  long p0 = 0, t0 = 0;
  long p1 = 0, t1 = 0;
  uint32_t t0_ms = 0, T_ms = 0;
};

//==================== Variables globales ====================
extern Follow follow;
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

/**
 * @brief Calcule la compensation pan à partir de la position slide
 * @param slide Position actuelle du slide
 * @return Compensation pan en steps
 */
long panCompFromSlide(long slide);

/**
 * @brief Calcule la compensation tilt à partir de la position slide
 * @param slide Position actuelle du slide
 * @return Compensation tilt en steps
 */
long tiltCompFromSlide(long slide);

/**
 * @brief Rafraîchit l'ancre de suivi
 * @details Calcule les ancres pan/tilt basées sur la position actuelle
 */
void refreshFollowAnchor();

/**
 * @brief Active le mode de suivi
 */
void startTracking();

/**
 * @brief Désactive le mode de suivi
 */
void stopTracking();

/**
 * @brief Vérifie si le suivi est activé
 * @return true si activé, false sinon
 */
bool isTrackingEnabled();

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

/**
 * @brief Définit les paramètres de mapping pan
 * @param min Compensation pan au slide minimum
 * @param max Compensation pan au slide maximum
 */
void setPanMapping(long min, long max);

/**
 * @brief Définit les paramètres de mapping tilt
 * @param min Compensation tilt au slide minimum
 * @param max Compensation tilt au slide maximum
 */
void setTiltMapping(long min, long max);

/**
 * @brief Obtient les paramètres de mapping pan
 * @param min Référence pour la compensation pan minimum
 * @param max Référence pour la compensation pan maximum
 */
void getPanMapping(long &min, long &max);

/**
 * @brief Obtient les paramètres de mapping tilt
 * @param min Référence pour la compensation tilt minimum
 * @param max Référence pour la compensation tilt maximum
 */
void getTiltMapping(long &min, long &max);

#endif // TRACKING_H
