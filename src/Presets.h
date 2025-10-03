/**
 * @file Presets.h
 * @brief Module de gestion des presets de mouvement et interpolation
 * @author Laurent Eyen
 * @date 2024
 */

#ifndef PRESETS_H
#define PRESETS_H

#include <Arduino.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include "Config.h"

// Les constantes des presets sont maintenant dans le module Config

//==================== Structures de données ====================
struct Preset {
  long p, t, z, s;          // positions absolues
};

struct InterpPoint { 
  uint8_t presetIndex; 
  float fraction; 
};

struct Bank {
  Preset presets[MAX_PRESETS];
  InterpPoint interpPoints[MAX_INTERP_POINTS];
  uint8_t interpCount;
};

struct InterpAuto {
  bool active = false;
  uint32_t t0_ms = 0;
  uint32_t T_ms = 5000;   // durée du parcours 0->100%
  int dir = +1;           // sens courant (+1 = 0→100%, -1 = 100%→0%)
};

//==================== Variables globales ====================
extern Preset presets[MAX_PRESETS];
extern int activePreset;
extern InterpPoint interpPoints[MAX_INTERP_POINTS];
extern uint8_t interpCount;
extern Bank banks[MAX_BANKS];
extern uint8_t activeBank;
extern Preferences nvs;
extern InterpAuto interpAuto;
extern float interp_jog_cmd;

//==================== Fonctions du module ====================

/**
 * @brief Initialise le système de presets
 */
void initPresets();

/**
 * @brief Définit un preset avec les positions actuelles
 * @param index Index du preset (0-7)
 */
void setPreset(uint8_t index);

/**
 * @brief Obtient un preset
 * @param index Index du preset (0-7)
 * @return Référence vers le preset
 */
Preset& getPreset(uint8_t index);

/**
 * @brief Sauvegarde un preset dans la banque active
 * @param index Index du preset (0-7)
 */
void savePreset(uint8_t index);

/**
 * @brief Charge un preset depuis la banque active
 * @param index Index du preset (0-7)
 */
void loadPreset(uint8_t index);

/**
 * @brief Sauvegarde une banque complète
 * @param idx Index de la banque (0-9)
 */
void saveBank(uint8_t idx);

/**
 * @brief Charge une banque complète
 * @param idx Index de la banque (0-9)
 */
void loadBank(uint8_t idx);

/**
 * @brief Met à jour l'interpolation automatique
 * @details Doit être appelée dans la loop principale
 */
void updateInterpolation();

/**
 * @brief Calcule une position interpolée
 * @param u Fraction d'interpolation (0.0-1.0)
 * @param P Référence pour la position Pan
 * @param T Référence pour la position Tilt
 * @param Z Référence pour la position Zoom
 * @param S Référence pour la position Slide
 */
void computeInterpolatedPosition(float u, long &P, long &T, long &Z, long &S);

/**
 * @brief Démarre l'interpolation automatique
 * @param duration Durée en secondes
 */
void startInterpolation(float duration);

/**
 * @brief Arrête l'interpolation automatique
 */
void stopInterpolation();

/**
 * @brief Vérifie si l'interpolation est active
 * @return true si active, false sinon
 */
bool isInterpolationActive();

/**
 * @brief Définit la commande de jog d'interpolation
 * @param cmd Commande normalisée (-1.0 à +1.0)
 */
void setInterpJogCommand(float cmd);

/**
 * @brief Calcule la durée optimale pour un mouvement
 * @param deltaP Écart Pan
 * @param deltaT Écart Tilt
 * @param deltaZ Écart Zoom
 * @param deltaS Écart Slide
 * @return Durée en millisecondes
 */
uint32_t pickDurationMsForDeltas(long deltaP, long deltaT, long deltaZ, long deltaS);

#endif // PRESETS_H
