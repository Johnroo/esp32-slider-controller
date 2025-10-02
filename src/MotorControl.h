/**
 * @file MotorControl.h
 * @brief Module de contrôle des moteurs stepper avec FastAccelStepper et TMC2209
 * 
 * Ce module encapsule toute la logique de gestion des moteurs :
 * - Initialisation de FastAccelStepper
 * - Configuration des pilotes TMC2209
 * - Gestion des objets moteurs
 * - Fonctions utilitaires
 */

#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <Arduino.h>
#include <FastAccelStepper.h>
#include <TMCStepper.h>

//==================== Configuration moteurs ====================
#define NUM_MOTORS 4

// Pins moteurs (définis dans MotorControl.cpp)
extern const int STEP_PINS[NUM_MOTORS];
extern const int DIR_PINS[NUM_MOTORS];
extern const int ENABLE_PINS[NUM_MOTORS];

// UART TMC2209
#define UART_RX 16
#define UART_TX 17
#define R_SENSE 0.11f  // R_sense = 110mΩ

// Adresses TMC2209
#define ADDR_PAN  0
#define ADDR_TILT 1
#define ADDR_ZOOM 2
#define ADDR_SLIDE 3

// Configuration par défaut des moteurs
struct MotorConfig {
  int current_ma;
  int microsteps;
  bool spreadcycle;
  int sgt;          // StallGuard threshold
  int max_speed;
  int max_accel;
  long min_limit;
  long max_limit;
};

//==================== Variables globales moteurs ====================
extern FastAccelStepperEngine engine;
extern FastAccelStepper* steppers[NUM_MOTORS];

// Drivers TMC2209
extern TMC2209Stepper driver_pan;
extern TMC2209Stepper driver_tilt;
extern TMC2209Stepper driver_zoom;
extern TMC2209Stepper driver_slide;
extern TMC2209Stepper* drivers[NUM_MOTORS];

// Configuration des moteurs
extern MotorConfig cfg[NUM_MOTORS];

//==================== Fonctions d'initialisation ====================
/**
 * @brief Initialise le module de contrôle des moteurs
 * 
 * Cette fonction regroupe toute l'initialisation matérielle :
 * - Initialisation de FastAccelStepper
 * - Configuration des drivers TMC2209
 * - Connexion des steppers aux pins
 * - Configuration des paramètres de base
 */
void initMotors();

/**
 * @brief Configure les drivers TMC2209
 * 
 * Initialise la communication UART et configure tous les paramètres
 * des drivers TMC2209 (courant, microsteps, StallGuard, etc.)
 */
void setupDriversTMC();

//==================== Fonctions utilitaires moteurs ====================
/**
 * @brief Vérifie si un moteur est en cours de déplacement
 * @param motorIndex Index du moteur (0=PAN, 1=TILT, 2=ZOOM, 3=SLIDE)
 * @return true si le moteur est en mouvement
 */
bool isMotorRunning(int motorIndex);

/**
 * @brief Arrête immédiatement un moteur
 * @param motorIndex Index du moteur à arrêter
 */
void stopMotor(int motorIndex);

/**
 * @brief Arrête immédiatement tous les moteurs
 */
void stopAllMotors();

/**
 * @brief Obtient la position actuelle d'un moteur
 * @param motorIndex Index du moteur
 * @return Position actuelle en steps
 */
long getMotorPosition(int motorIndex);

/**
 * @brief Obtient la position cible d'un moteur
 * @param motorIndex Index du moteur
 * @return Position cible en steps
 */
long getMotorTarget(int motorIndex);

/**
 * @brief Définit la vitesse d'un moteur
 * @param motorIndex Index du moteur
 * @param speed Vitesse en Hz
 */
void setMotorSpeed(int motorIndex, int speed);

/**
 * @brief Définit l'accélération d'un moteur
 * @param motorIndex Index du moteur
 * @param acceleration Accélération en steps/s²
 */
void setMotorAcceleration(int motorIndex, int acceleration);

/**
 * @brief Déplace un moteur vers une position
 * @param motorIndex Index du moteur
 * @param target Position cible en steps
 */
void moveMotorTo(int motorIndex, long target);

/**
 * @brief Déplace un moteur relativement
 * @param motorIndex Index du moteur
 * @param steps Nombre de steps à déplacer (+ ou -)
 */
void moveMotor(int motorIndex, long steps);

//==================== Fonctions de position ====================
/**
 * @brief Met à jour les variables globales de position
 * 
 * Cette fonction lit les positions actuelles de tous les moteurs
 * et met à jour les variables globales panPos, tiltPos, zoomPos, slidePos
 */
void updateMotorPositions();

// Variables globales de position (déclarées extern pour compatibilité)
extern volatile long panPos;
extern volatile long tiltPos;
extern volatile long zoomPos;
extern volatile long slidePos;

#endif // MOTORCONTROL_H
