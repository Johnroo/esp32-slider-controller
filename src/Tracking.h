/**
 * @file Tracking.h
 * @brief Module de gestion du suivi coordonné slide-pan/tilt
 * 
 * Ce module encapsule toute la logique de tracking :
 * - Mapping linéaire slide -> compensation pan/tilt
 * - Gestion des ancres de suivi
 * - Modes de suivi (follow, anchor morph)
 * - Compensation automatique des mouvements
 */

#ifndef TRACKING_H
#define TRACKING_H

#include <Arduino.h>
#include "MotorControl.h"

// Les fonctions active_pan_offset et active_tilt_offset sont définies dans main.cpp

//==================== Constantes de mapping ====================
extern long PAN_AT_SLIDE_MIN;   // Position PAN à gauche du slide
extern long PAN_AT_SLIDE_MAX;   // Position PAN à droite du slide
extern long TILT_AT_SLIDE_MIN;  // Position TILT à gauche du slide
extern long TILT_AT_SLIDE_MAX;  // Position TILT à droite du slide

//==================== Structures de données ====================

/**
 * @brief Structure de suivi coordonné
 */
struct Follow {
  bool enabled = false;     // Suivi activé/désactivé
  bool valid = false;       // Ancre valide (calculée)
  long pan_anchor = 0;      // Position de référence PAN
  long tilt_anchor = 0;     // Position de référence TILT
};

/**
 * @brief Structure de morphing d'ancres
 */
struct AnchorMorph {
  bool active = false;      // Morphing en cours
  uint32_t t0_ms = 0;      // Temps de début
  uint32_t T_ms = 1000;    // Durée du morphing
  long p0 = 0, p1 = 0;     // Ancres PAN début/fin
  long t0 = 0, t1 = 0;     // Ancres TILT début/fin
};

//==================== Variables globales (extern) ====================
extern Follow follow;           // Structure de suivi
extern AnchorMorph anchor_morph; // Morphing d'ancres

//==================== Fonctions d'initialisation ====================

/**
 * @brief Initialise le module de tracking
 */
void initTracking();

/**
 * @brief Définit les paramètres de mapping slide->pan/tilt
 * @param pan_min Position PAN à gauche du slide
 * @param pan_max Position PAN à droite du slide
 * @param tilt_min Position TILT à gauche du slide
 * @param tilt_max Position TILT à droite du slide
 */
void setSlideMapping(long pan_min, long pan_max, long tilt_min, long tilt_max);

//==================== Fonctions de mapping ====================

/**
 * @brief Calcule la compensation PAN basée sur la position du slide
 * @param slide Position actuelle du slide
 * @return Compensation PAN en steps
 */
long panCompFromSlide(long slide);

/**
 * @brief Calcule la compensation TILT basée sur la position du slide
 * @param slide Position actuelle du slide
 * @return Compensation TILT en steps
 */
long tiltCompFromSlide(long slide);

//==================== Fonctions de gestion des ancres ====================

/**
 * @brief Actualise les ancres de suivi
 * 
 * Calcule les positions de référence PAN/TILT basées sur
 * les positions actuelles et les compensations slide
 */
void refreshAnchor();

/**
 * @brief Définit les ancres manuellement
 * @param pan_anchor Position de référence PAN
 * @param tilt_anchor Position de référence TILT
 */
void setAnchor(long pan_anchor, long tilt_anchor);

/**
 * @brief Invalide les ancres (force leur recalcul)
 */
void invalidateAnchor();

/**
 * @brief Vérifie si les ancres sont valides
 * @return true si les ancres sont valides
 */
bool isAnchorValid();

//==================== Fonctions de contrôle du suivi ====================

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
 * @return true si le suivi est activé
 */
bool isTrackingEnabled();

/**
 * @brief Active/désactive le mode de suivi
 * @param enabled true pour activer, false pour désactiver
 */
void setTrackingEnabled(bool enabled);

//==================== Fonctions de morphing d'ancres ====================

/**
 * @brief Démarre un morphing d'ancres
 * @param duration_ms Durée du morphing en millisecondes
 * @param pan_start Ancre PAN de départ
 * @param pan_end Ancre PAN d'arrivée
 * @param tilt_start Ancre TILT de départ
 * @param tilt_end Ancre TILT d'arrivée
 */
void startAnchorMorph(uint32_t duration_ms, long pan_start, long pan_end, long tilt_start, long tilt_end);

/**
 * @brief Arrête le morphing d'ancres
 */
void stopAnchorMorph();

/**
 * @brief Vérifie si un morphing d'ancres est actif
 * @return true si un morphing est en cours
 */
bool isAnchorMorphActive();

//==================== Fonctions de mise à jour ====================

/**
 * @brief Met à jour le tracking
 * 
 * Cette fonction doit être appelée régulièrement dans la boucle principale.
 * Elle gère :
 * - Le morphing d'ancres
 * - Le calcul des positions cibles PAN/TILT basées sur le slide
 * - L'application des commandes aux moteurs
 * 
 * @param slide_position Position actuelle du slide
 * @param in_recall Indique si on est en mode recall/interpolation
 * @return true si des mouvements ont été appliqués
 */
bool updateTracking(long slide_position, bool in_recall = false);

/**
 * @brief Met à jour le tracking pour le jog du slide
 * 
 * Version spécialisée pour le jog du slide qui applique
 * immédiatement les compensations PAN/TILT
 * 
 * @param slide_goal Position cible du slide
 * @param in_recall Indique si on est en mode recall/interpolation
 * @return true si des mouvements ont été appliqués
 */
bool updateTrackingForJog(long slide_goal, bool in_recall = false);

//==================== Fonctions utilitaires ====================

/**
 * @brief Obtient la position de référence PAN
 * @return Position de référence PAN
 */
long getPanAnchor();

/**
 * @brief Obtient la position de référence TILT
 * @return Position de référence TILT
 */
long getTiltAnchor();

/**
 * @brief Obtient les paramètres de mapping actuels
 * @param pan_min Position PAN à gauche du slide (sortie)
 * @param pan_max Position PAN à droite du slide (sortie)
 * @param tilt_min Position TILT à gauche du slide (sortie)
 * @param tilt_max Position TILT à droite du slide (sortie)
 */
void getSlideMapping(long &pan_min, long &pan_max, long &tilt_min, long &tilt_max);

#endif // TRACKING_H
