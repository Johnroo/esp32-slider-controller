/**
 * @file MotorControl.h
 * @brief Module de contrôle des moteurs stepper et drivers TMC2209
 * 
 * Ce module encapsule toute la logique de gestion des moteurs :
 * - Initialisation FastAccelStepper
 * - Configuration des drivers TMC2209
 * - Gestion des positions et mouvements
 * - Interfaces de contrôle des moteurs
 */

#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <Arduino.h>
#include <FastAccelStepper.h>
#include <TMCStepper.h>
#include "Config.h"

// Les constantes de configuration sont maintenant dans le module Config

//==================== Configuration des moteurs ====================
struct MotorConfig {
  int max_speed;
  int max_accel;
  int min_limit;
  int max_limit;
  int current_ma;    // Courant RMS en mA
  int microsteps;    // Nombre de microsteps
  bool spreadcycle;
  int sgt;  // StallGuard threshold
};

extern MotorConfig cfg[NUM_MOTORS];

//==================== Objets moteurs ====================
extern FastAccelStepperEngine engine;
extern FastAccelStepper* steppers[NUM_MOTORS];

extern TMC2209Stepper driver_pan;
extern TMC2209Stepper driver_tilt;
extern TMC2209Stepper driver_zoom;
extern TMC2209Stepper driver_slide;
extern TMC2209Stepper* drivers[NUM_MOTORS];

//==================== Variables de position ====================
extern volatile long panPos;
extern volatile long tiltPos;
extern volatile long zoomPos;
extern volatile long slidePos;

//==================== Fonctions d'initialisation ====================
/**
 * @brief Initialise tous les moteurs et drivers TMC2209
 * 
 * Cette fonction doit être appelée dans setup() pour initialiser :
 * - L'engine FastAccelStepper
 * - Les drivers TMC2209
 * - La configuration des steppers
 */
void initMotors();

/**
 * @brief Configure les drivers TMC2209
 * 
 * Initialise la communication UART et configure tous les paramètres
 * des drivers TMC2209 (courant, microsteps, StallGuard, etc.)
 */
void setupDriversTMC();

//==================== Fonctions de contrôle ====================
/**
 * @brief Déplace un moteur à une position spécifique
 * @param motor_id ID du moteur (0=PAN, 1=TILT, 2=ZOOM, 3=SLIDE)
 * @param position Position cible en steps
 */
void moveMotorTo(int motor_id, long position);

/**
 * @brief Définit la vitesse d'un moteur
 * @param motor_id ID du moteur (0=PAN, 1=TILT, 2=ZOOM, 3=SLIDE)
 * @param speed Vitesse en Hz
 */
void setMotorSpeed(int motor_id, int speed);

/**
 * @brief Met à jour les positions des moteurs
 * 
 * Cette fonction doit être appelée régulièrement dans loop()
 * pour mettre à jour les variables de position globales
 */
void updateMotorPositions();

//==================== Fonctions utilitaires ====================
/**
 * @brief Obtient la position actuelle d'un moteur
 * @param motor_id ID du moteur (0=PAN, 1=TILT, 2=ZOOM, 3=SLIDE)
 * @return Position actuelle en steps
 */
long getMotorPosition(int motor_id);

/**
 * @brief Vérifie si un moteur est en mouvement
 * @param motor_id ID du moteur (0=PAN, 1=TILT, 2=ZOOM, 3=SLIDE)
 * @return true si le moteur est en mouvement
 */
bool isMotorMoving(int motor_id);

/**
 * @brief Arrête tous les moteurs d'urgence
 */
void emergencyStop();

#endif // MOTORCONTROL_H
