/**
 * @file MotionPlanner.h
 * @brief Module de planification des mouvements synchronisés
 * 
 * Ce module encapsule toute la logique de planification des mouvements :
 * - Calcul de durées synchronisées pour plusieurs axes
 * - Gestion des mouvements coordonnés
 * - Planification des séquences de mouvements
 * - Coordination avec les autres modules (Tracking, Presets)
 */

#ifndef MOTIONPLANNER_H
#define MOTIONPLANNER_H

#include <Arduino.h>
#include "MotorControl.h"

// Déclarations des fonctions externes nécessaires
long active_pan_offset(bool recall_phase);
long active_tilt_offset(bool recall_phase);

//==================== Structures de données ====================

/**
 * @brief Structure de mouvement synchronisé
 */
struct SyncMove {
  bool active = false;              // Mouvement en cours
  uint32_t t0_ms = 0;              // Temps de début du mouvement
  uint32_t T_ms = 0;               // Durée totale du mouvement
  long start[NUM_MOTORS];          // Positions de départ pour chaque axe
  long goal_base[NUM_MOTORS];      // Positions cibles pour chaque axe
};

//==================== Variables globales (extern) ====================
extern SyncMove sync_move;

//==================== Fonctions d'initialisation ====================

/**
 * @brief Initialise le module de planification des mouvements
 */
void initMotionPlanner();

/**
 * @brief Réinitialise le planificateur
 */
void resetMotionPlanner();

//==================== Fonctions de planification ====================

/**
 * @brief Calcule la durée optimale pour un mouvement synchronisé
 * 
 * Cette fonction calcule la durée minimale requise pour que tous les axes
 * puissent atteindre leurs positions cibles en respectant leurs limites
 * de vitesse et d'accélération.
 * 
 * @param start Positions de départ pour chaque axe
 * @param goal Positions cibles pour chaque axe
 * @param T_req_ms Durée requise en millisecondes
 * @return Durée calculée en millisecondes
 */
uint32_t pickDurationMsForDeltas(const long start[NUM_MOTORS], const long goal[NUM_MOTORS], uint32_t T_req_ms);

/**
 * @brief Planifie un mouvement synchronisé vers des positions absolues
 * 
 * @param target_positions Positions cibles pour chaque axe [PAN, TILT, ZOOM, SLIDE]
 * @param duration_ms Durée souhaitée en millisecondes
 * @return true si le mouvement a été planifié avec succès
 */
bool planSynchronizedMove(const long target_positions[NUM_MOTORS], uint32_t duration_ms);

/**
 * @brief Planifie un mouvement synchronisé vers des positions relatives
 * 
 * @param relative_positions Déplacements relatifs pour chaque axe [PAN, TILT, ZOOM, SLIDE]
 * @param duration_ms Durée souhaitée en millisecondes
 * @return true si le mouvement a été planifié avec succès
 */
bool planRelativeMove(const long relative_positions[NUM_MOTORS], uint32_t duration_ms);

/**
 * @brief Planifie un mouvement d'un seul axe
 * 
 * @param axis Index de l'axe (0=PAN, 1=TILT, 2=ZOOM, 3=SLIDE)
 * @param target_position Position cible pour l'axe
 * @param duration_ms Durée souhaitée en millisecondes
 * @return true si le mouvement a été planifié avec succès
 */
bool planSingleAxisMove(uint8_t axis, long target_position, uint32_t duration_ms);

//==================== Fonctions de contrôle du planificateur ====================

/**
 * @brief Démarre l'exécution du mouvement planifié
 * 
 * @return true si le mouvement a été démarré
 */
bool startPlannedMove();

/**
 * @brief Arrête le mouvement en cours
 */
void stopPlannedMove();

/**
 * @brief Met en pause le mouvement en cours
 */
void pausePlannedMove();

/**
 * @brief Reprend un mouvement en pause
 */
void resumePlannedMove();

//==================== Fonctions d'état ====================

/**
 * @brief Vérifie si un mouvement est actuellement en cours
 * @return true si un mouvement est actif
 */
bool isMoveActive();

/**
 * @brief Vérifie si un mouvement est en pause
 * @return true si un mouvement est en pause
 */
bool isMovePaused();

/**
 * @brief Obtient le pourcentage d'avancement du mouvement en cours
 * @return Pourcentage d'avancement (0.0 à 1.0)
 */
float getMoveProgress();

/**
 * @brief Obtient le temps restant estimé du mouvement en cours
 * @return Temps restant en millisecondes
 */
uint32_t getRemainingTime();

/**
 * @brief Obtient la durée totale du mouvement en cours
 * @return Durée totale en millisecondes
 */
uint32_t getTotalDuration();

//==================== Fonctions de mise à jour ====================

/**
 * @brief Met à jour le planificateur de mouvements
 * 
 * Cette fonction doit être appelée régulièrement dans la boucle principale.
 * Elle gère :
 * - L'avancement du mouvement en cours
 * - Le calcul des positions interpolées
 * - L'application des commandes aux moteurs
 * - La gestion des compensations (tracking, offsets)
 * 
 * @param now_ms Timestamp actuel en millisecondes
 * @return true si un mouvement est en cours
 */
bool updateMotionPlanner(uint32_t now_ms);

/**
 * @brief Met à jour le planificateur avec le timestamp actuel
 * 
 * Version simplifiée qui utilise millis() automatiquement
 * 
 * @return true si un mouvement est en cours
 */
bool updateMotionPlanner();

//==================== Fonctions de configuration ====================

/**
 * @brief Définit les limites de vitesse pour la planification
 * 
 * @param max_speeds Vitesses maximales pour chaque axe en steps/s
 */
void setMaxSpeeds(const float max_speeds[NUM_MOTORS]);

/**
 * @brief Définit les limites d'accélération pour la planification
 * 
 * @param max_accelerations Accélérations maximales pour chaque axe en steps/s²
 */
void setMaxAccelerations(const float max_accelerations[NUM_MOTORS]);

/**
 * @brief Obtient les vitesses maximales configurées
 * 
 * @param max_speeds Tableau de sortie pour les vitesses maximales
 */
void getMaxSpeeds(float max_speeds[NUM_MOTORS]);

/**
 * @brief Obtient les accélérations maximales configurées
 * 
 * @param max_accelerations Tableau de sortie pour les accélérations maximales
 */
void getMaxAccelerations(float max_accelerations[NUM_MOTORS]);

//==================== Fonctions utilitaires ====================

/**
 * @brief Obtient les positions de départ du mouvement en cours
 * 
 * @param start_positions Tableau de sortie pour les positions de départ
 */
void getStartPositions(long start_positions[NUM_MOTORS]);

/**
 * @brief Obtient les positions cibles du mouvement en cours
 * 
 * @param target_positions Tableau de sortie pour les positions cibles
 */
void getTargetPositions(long target_positions[NUM_MOTORS]);

/**
 * @brief Calcule la distance totale à parcourir
 * 
 * @return Distance totale en steps
 */
double getTotalDistance();

/**
 * @brief Calcule la distance pour un axe spécifique
 * 
 * @param axis Index de l'axe
 * @return Distance en steps
 */
long getAxisDistance(uint8_t axis);

/**
 * @brief Obtient l'état détaillé du planificateur
 * 
 * @return String JSON contenant l'état du planificateur
 */
String getPlannerStatus();

#endif // MOTIONPLANNER_H
