/**
 * @file Config.h
 * @brief Configuration centralisée - Constantes et paramètres globaux
 * @author Laurent Eyen
 * @date 2024
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

//==================== Constantes du système ====================

// Nombre de moteurs
#define NUM_MOTORS 4

// Index des axes
#define SLIDE_INDEX 3

//==================== Configuration matérielle ====================

// Pins STEP/DIR/EN (définis dans Config.cpp)
// ⚠️ ATTENTION: Ces pins sont critiques pour le matériel - ne pas modifier sans vérification!
extern const int STEP_PINS[NUM_MOTORS];
extern const int DIR_PINS[NUM_MOTORS];
extern const int ENABLE_PINS[NUM_MOTORS];

// UART TMC2209
#define UART_TX 17
#define UART_RX 16

// Adresses TMC2209
#define ADDR_PAN  0
#define ADDR_TILT 1
#define ADDR_ZOOM 2
#define ADDR_SLIDE 3

// R_SENSE pour TMC2209
#define R_SENSE 0.11f

//==================== Configuration des moteurs ====================

// Vitesses par défaut (steps/s)
#define DEFAULT_PAN_MAX_SPEED    20000
#define DEFAULT_TILT_MAX_SPEED   20000
#define DEFAULT_ZOOM_MAX_SPEED   20000
#define DEFAULT_SLIDE_MAX_SPEED  10000

// Accélérations par défaut (steps/s²)
#define DEFAULT_PAN_ACCEL    12000
#define DEFAULT_TILT_ACCEL   12000
#define DEFAULT_ZOOM_ACCEL   8000
#define DEFAULT_SLIDE_ACCEL  12000

// Courants par défaut (mA)
#define DEFAULT_PAN_CURRENT    1700
#define DEFAULT_TILT_CURRENT   1700
#define DEFAULT_ZOOM_CURRENT   400
#define DEFAULT_SLIDE_CURRENT  1600

// Microstepping par défaut
#define DEFAULT_PAN_MICROSTEPS    16
#define DEFAULT_TILT_MICROSTEPS   16
#define DEFAULT_ZOOM_MICROSTEPS   16
#define DEFAULT_SLIDE_MICROSTEPS  8

// Limites par défaut (steps)
#define DEFAULT_PAN_MIN_LIMIT    -27106
#define DEFAULT_PAN_MAX_LIMIT     27106
#define DEFAULT_TILT_MIN_LIMIT   -2439
#define DEFAULT_TILT_MAX_LIMIT    2439
#define DEFAULT_ZOOM_MIN_LIMIT   -20000
#define DEFAULT_ZOOM_MAX_LIMIT    20000
#define DEFAULT_SLIDE_MIN_LIMIT  -20000
#define DEFAULT_SLIDE_MAX_LIMIT   20000

//==================== Configuration du homing ====================

// Paramètres de homing
#define HOMING_SPEED    9000     // steps/s
#define HOMING_ACCEL    60000    // accel élevée pour atteindre vitesse rapidement
#define SG_DETECT       20       // seuil SG_RESULT pour détecter stall
#define HOMING_TIMEOUT  20000    // ms
#define BACKOFF_STEPS   1000     // pas de recul après détection
#define BACKOFF_DELAY   500      // ms de délai après recul pour stabilisation
#define SAFETY_STEPS    500      // marge de sécurité à l'intérieur des butées

//==================== Configuration des presets ====================

// Limites des presets et banques
#define MAX_PRESETS 8
#define MAX_INTERP_POINTS 6
#define MAX_BANKS 10

// Durée par défaut des mouvements synchronisés (ms)
#define DEFAULT_MOVE_DURATION 2000

//==================== Configuration du joystick ====================

// Ranges d'offsets par défaut
// Plages de slew limiting pour le filtrage joystick (plus grande = réponse plus rapide)
#define DEFAULT_PAN_SLEW_RANGE  800
#define DEFAULT_TILT_SLEW_RANGE 800

// Vitesses de jog par défaut (steps/s)
#define DEFAULT_PAN_JOG_SPEED   18000.0f
#define DEFAULT_TILT_JOG_SPEED  3000.0f
#define DEFAULT_SLIDE_JOG_SPEED 6000.0f

// Configuration joystick par défaut
#define DEFAULT_JOYSTICK_DEADZONE     0.06f
#define DEFAULT_JOYSTICK_EXPO         0.35f
#define DEFAULT_JOYSTICK_SLEW_PER_S   8000.0f
#define DEFAULT_JOYSTICK_FILT_HZ      60.0f
#define DEFAULT_JOYSTICK_PAN_TILT_SPEED 3.0f
#define DEFAULT_JOYSTICK_SLIDE_SPEED    1.0f

// Anti-bruit pour les offsets
#define OFFSET_DEADBAND_STEPS 2

//==================== Configuration réseau ====================

// Port OSC
#define OSC_PORT 8000

// Port web server
#define WEB_SERVER_PORT 80

//==================== Configuration du suivi ====================

// Mapping slide -> pan/tilt supprimé (mode follow obsolète)

//==================== Variables globales configurables ====================

// Ranges d'offsets (modifiables via OSC)
extern long PAN_OFFSET_RANGE;
extern long TILT_OFFSET_RANGE;

// Vitesses de jog (calculées dans setup)
extern float PAN_JOG_SPEED;
extern float TILT_JOG_SPEED;
extern float SLIDE_JOG_SPEED;

// Durée par défaut des mouvements
extern uint32_t DEFAULT_MOVE_DURATION_MS;

//==================== Fonctions de configuration ====================

/**
 * @brief Initialise la configuration globale
 */
void initConfig();

/**
 * @brief Obtient les vitesses de jog actuelles
 * @param pan Référence pour la vitesse pan
 * @param tilt Référence pour la vitesse tilt
 * @param slide Référence pour la vitesse slide
 */
void getJogSpeeds(float &pan, float &tilt, float &slide);

/**
 * @brief Définit les vitesses de jog
 * @param pan Vitesse pan
 * @param tilt Vitesse tilt
 * @param slide Vitesse slide
 */
void setJogSpeeds(float pan, float tilt, float slide);

/**
 * @brief Obtient les ranges d'offsets actuels
 * @param pan Référence pour le range pan
 * @param tilt Référence pour le range tilt
 */
void getOffsetRanges(long &pan, long &tilt);

/**
 * @brief Définit les ranges d'offsets
 * @param pan Range pan
 * @param tilt Range tilt
 */
void setOffsetRanges(long pan, long tilt);

/**
 * @brief Obtient la durée par défaut des mouvements
 * @return Durée en millisecondes
 */
uint32_t getDefaultMoveDuration();

/**
 * @brief Définit la durée par défaut des mouvements
 * @param durationMs Durée en millisecondes
 */
void setDefaultMoveDuration(uint32_t durationMs);

/**
 * @brief Affiche la configuration actuelle
 */
void printConfig();

#endif // CONFIG_H
