/**
 * @file Presets.h
 * @brief Module de gestion des presets et interpolation multi-presets
 * 
 * Ce module encapsule toute la logique des presets :
 * - Structures de données des presets et points d'interpolation
 * - Gestion des banques de presets (save/load)
 * - Interpolation multi-presets avec courbes personnalisées
 * - Modes d'interpolation (automatique, manuel, jog)
 * - Sérialisation JSON pour export/import
 */

#ifndef PRESETS_H
#define PRESETS_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include "MotorControl.h"

//==================== Constantes ====================
#define NUM_PRESETS 8          // Nombre de presets par banque
#define NUM_BANKS 10           // Nombre de banques disponibles
#define MAX_INTERP_POINTS 6    // Nombre maximum de points d'interpolation

//==================== Structures de données ====================

/**
 * @brief Structure représentant un preset de position
 */
struct Preset {
  long p;  // Position PAN (steps)
  long t;  // Position TILT (steps)
  long z;  // Position ZOOM (steps)
  long s;  // Position SLIDE (steps)
};

/**
 * @brief Point d'interpolation sur l'axe multi-presets
 */
struct InterpPoint {
  uint8_t presetIndex;  // Index du preset (0-7)
  float fraction;       // Fraction sur l'axe d'interpolation (0.0-1.0)
};

/**
 * @brief Banque complète de presets et d'interpolation
 */
struct Bank {
  Preset presets[NUM_PRESETS];      // 8 presets de position
  InterpPoint interpPoints[MAX_INTERP_POINTS];  // Points d'interpolation
  uint8_t interpCount;              // Nombre de points d'interpolation actifs
};

/**
 * @brief Mode d'interpolation automatique
 */
struct InterpAuto {
  bool active;           // Interpolation automatique active
  uint32_t t0_ms;       // Temps de démarrage
  uint32_t T_ms;        // Durée totale du cycle
  int8_t dir;           // Direction (+1 ou -1)
};

//==================== Variables globales (extern) ====================
extern Preset presets[NUM_PRESETS];           // Presets actifs
extern int activePreset;                      // Preset actuellement actif (-1 = aucun)
extern Bank banks[NUM_BANKS];                 // Banques de presets
extern uint8_t activeBank;                    // Banque active (0-9)
extern InterpPoint interpPoints[MAX_INTERP_POINTS];  // Points d'interpolation actifs
extern uint8_t interpCount;                   // Nombre de points d'interpolation
extern InterpAuto interpAuto;                 // Mode interpolation automatique
extern float interp_jog_cmd;                  // Commande de jog manuel (-1..+1)

//==================== Fonctions de gestion des presets ====================

/**
 * @brief Initialise le module Presets
 */
void initPresets();

/**
 * @brief Sauvegarde un preset avec la position actuelle des moteurs
 * @param index Index du preset (0-7)
 */
void savePreset(uint8_t index);

/**
 * @brief Charge un preset et positionne les moteurs
 * @param index Index du preset (0-7)
 */
void loadPreset(uint8_t index);

/**
 * @brief Obtient un preset sans le charger
 * @param index Index du preset (0-7)
 * @return Référence vers le preset
 */
const Preset& getPreset(uint8_t index);

/**
 * @brief Définit un preset avec des valeurs spécifiques
 * @param index Index du preset (0-7)
 * @param p Position PAN
 * @param t Position TILT
 * @param z Position ZOOM
 * @param s Position SLIDE
 */
void setPreset(uint8_t index, long p, long t, long z, long s);

/**
 * @brief Obtient l'index du preset actuellement actif
 * @return Index du preset actif (-1 si aucun)
 */
int getActivePreset();

/**
 * @brief Définit le preset actif
 * @param index Index du preset (0-7) ou -1 pour aucun
 */
void setActivePreset(int index);

//==================== Fonctions de gestion des banques ====================

/**
 * @brief Sauvegarde une banque complète
 * @param bankIndex Index de la banque (0-9)
 */
void saveBank(uint8_t bankIndex);

/**
 * @brief Charge une banque complète
 * @param bankIndex Index de la banque (0-9)
 */
void loadBank(uint8_t bankIndex);

/**
 * @brief Obtient l'index de la banque active
 * @return Index de la banque active (0-9)
 */
uint8_t getActiveBank();

/**
 * @brief Définit la banque active
 * @param bankIndex Index de la banque (0-9)
 */
void setActiveBank(uint8_t bankIndex);

/**
 * @brief Sauvegarde la banque active
 */
void saveActiveBank();

/**
 * @brief Charge la banque active
 */
void loadActiveBank();

//==================== Fonctions d'interpolation ====================

/**
 * @brief Calcule une position interpolée entre les presets
 * @param fraction Fraction sur l'axe d'interpolation (0.0-1.0)
 * @param P Position PAN résultante (sortie)
 * @param T Position TILT résultante (sortie)
 * @param Z Position ZOOM résultante (sortie)
 * @param S Position SLIDE résultante (sortie)
 */
void computeInterpolatedPosition(float fraction, long &P, long &T, long &Z, long &S);

/**
 * @brief Définit les points d'interpolation
 * @param points Tableau des points d'interpolation
 * @param count Nombre de points
 */
void setInterpolationPoints(const InterpPoint* points, uint8_t count);

/**
 * @brief Obtient les points d'interpolation actuels
 * @param points Tableau de sortie pour les points
 * @return Nombre de points
 */
uint8_t getInterpolationPoints(InterpPoint* points);

/**
 * @brief Obtient le nombre de points d'interpolation
 * @return Nombre de points d'interpolation actifs
 */
uint8_t getInterpolationCount();

//==================== Fonctions de contrôle d'interpolation ====================

/**
 * @brief Démarre l'interpolation automatique
 * @param duration_ms Durée du cycle en millisecondes
 */
void startAutoInterpolation(uint32_t duration_ms);

/**
 * @brief Arrête l'interpolation automatique
 */
void stopAutoInterpolation();

/**
 * @brief Vérifie si l'interpolation automatique est active
 * @return true si l'interpolation automatique est en cours
 */
bool isAutoInterpolationActive();

/**
 * @brief Va à une position spécifique sur l'axe d'interpolation
 * @param fraction Fraction cible (0.0-1.0)
 */
void gotoInterpolationPosition(float fraction);

/**
 * @brief Définit la vitesse de jog manuel sur l'axe d'interpolation
 * @param speed Vitesse normalisée (-1.0 à +1.0)
 */
void setInterpolationJogSpeed(float speed);

/**
 * @brief Met à jour l'interpolation (à appeler régulièrement)
 */
void updateInterpolation();

//==================== Fonctions de sérialisation ====================

/**
 * @brief Sérialise une banque en JSON
 * @param bankIndex Index de la banque à sérialiser
 * @param jsonString Chaîne de sortie JSON
 * @return true si la sérialisation a réussi
 */
bool serializeBankToJSON(uint8_t bankIndex, String &jsonString);

/**
 * @brief Désérialise une banque depuis JSON
 * @param bankIndex Index de la banque à charger
 * @param jsonString Chaîne JSON d'entrée
 * @return true si la désérialisation a réussi
 */
bool deserializeBankFromJSON(uint8_t bankIndex, const String &jsonString);

#endif // PRESETS_H
