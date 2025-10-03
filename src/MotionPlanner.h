/**
 * @file MotionPlanner.h
 * @brief Module de planification des mouvements synchronisés
 * @author Laurent Eyen
 * @date 2024
 */

#ifndef MOTIONPLANNER_H
#define MOTIONPLANNER_H

#include <Arduino.h>
#include "Utils.h"
#include "Config.h"

// Les constantes sont maintenant dans le module Config

//==================== Structures de données ====================
struct SyncMove {
  bool  active = false;
  uint32_t t0_ms = 0;
  uint32_t T_ms  = 2000;      // durée demandée
  long start[NUM_MOTORS];
  long goal_base[NUM_MOTORS]; // cible sans offsets/couplages
};

//==================== Variables globales ====================
extern SyncMove sync_move;

//==================== Fonctions du module ====================

/**
 * @brief Initialise le planificateur de mouvements
 */
void initMotionPlanner();

/**
 * @brief Met à jour l'exécution des mouvements planifiés
 * @details Doit être appelée dans la loop principale
 */
void updateMotionPlanner();

/**
 * @brief Planifie un mouvement synchronisé vers des positions cibles
 * @param targetPositions Positions cibles pour chaque axe
 * @param durationMs Durée demandée en millisecondes
 * @return true si le mouvement a été planifié, false sinon
 */
bool planSynchronizedMove(const long targetPositions[NUM_MOTORS], uint32_t durationMs);

/**
 * @brief Planifie un mouvement synchronisé vers une position slide
 * @param slidePosition Position cible du slide
 * @param durationSec Durée en secondes
 * @return true si le mouvement a été planifié, false sinon
 */
bool planSlideMove(long slidePosition, float durationSec);

/**
 * @brief Arrête le mouvement synchronisé en cours
 */
void stopSynchronizedMove();

/**
 * @brief Vérifie si un mouvement synchronisé est en cours
 * @return true si actif, false sinon
 */
bool isSynchronizedMoveActive();

/**
 * @brief Calcule la durée optimale pour un mouvement
 * @param startPositions Positions de départ
 * @param targetPositions Positions cibles
 * @param requestedDurationMs Durée demandée en millisecondes
 * @return Durée calculée en millisecondes
 */
uint32_t calculateOptimalDuration(const long startPositions[NUM_MOTORS], 
                                  const long targetPositions[NUM_MOTORS], 
                                  uint32_t requestedDurationMs);

/**
 * @brief Intègre les offsets dans la cible du mouvement en cours
 * @param panOffset Offset pan à intégrer
 * @param tiltOffset Offset tilt à intégrer
 */
void bakeOffsetsIntoCurrentMove(long panOffset, long tiltOffset);

/**
 * @brief Obtient l'état du mouvement synchronisé
 * @param isActive Référence pour l'état actif
 * @param progress Référence pour le progrès (0.0-1.0)
 * @param remainingTimeMs Référence pour le temps restant en ms
 */
void getSynchronizedMoveStatus(bool &isActive, float &progress, uint32_t &remainingTimeMs);

/**
 * @brief Obtient les positions de départ du mouvement en cours
 * @param positions Tableau pour stocker les positions
 */
void getStartPositions(long positions[NUM_MOTORS]);

/**
 * @brief Obtient les positions cibles du mouvement en cours
 * @param positions Tableau pour stocker les positions
 */
void getTargetPositions(long positions[NUM_MOTORS]);

// Les fonctions setDefaultMoveDuration et getDefaultMoveDuration 
// sont maintenant dans le module Config

#endif // MOTIONPLANNER_H
