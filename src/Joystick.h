/**
 * @file Joystick.h
 * @brief Module de gestion complète du joystick
 * @author Laurent Eyen
 * @date 2024
 */

#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <Arduino.h>
#include "Utils.h"
#include "Config.h"

// Les constantes du joystick sont maintenant dans le module Config

//==================== Structures de données ====================
struct JoyCfg { 
  float deadzone = 0.06f;
  float expo = 0.35f;
  float slew_per_s = 8000.0f;
  float filt_hz = 60.0f;
  float pan_tilt_speed = 6.0f;
  float slide_speed = 1.0f;
};

struct JoyState { 
  float pan = 0;
  float tilt = 0;
  float slide = 0;
};

struct CancelPolicy { 
  bool by_joystick = false;  // joystick n'annule pas les presets (offsets actifs)
  bool by_axis = true;       // Direct Axis annule les presets (contrôle direct)
};

// Baseline d'offset au recall (Δoffset = latched - baseline)
struct OffsetSession {
  long pan0 = 0;
  long tilt0 = 0;
};

//==================== Variables globales ====================
extern JoyCfg joy;
extern JoyState joy_raw;      // alimenté par l'OSC
extern JoyState joy_cmd;      // après deadzone/expo
extern JoyState joy_filt;     // après filtrage
extern CancelPolicy cancel;
extern OffsetSession offset_session;

// Offsets joystick (en steps)
extern volatile long pan_offset_steps;
extern volatile long tilt_offset_steps;

// Offsets joystick latched (s'accumulent, persistent)
extern volatile long pan_offset_latched;
extern volatile long tilt_offset_latched;

// Les variables globales sont maintenant dans le module Config

// Jog slide
extern float slide_jog_cmd;    // -1..+1

//==================== Fonctions du module ====================

/**
 * @brief Initialise le module joystick
 */
void initJoystick();

/**
 * @brief Met à jour le traitement du joystick
 * @details Doit être appelée dans la loop principale
 */
void updateJoystick();

/**
 * @brief Applique deadzone et exposition à une valeur
 * @param x Valeur d'entrée (-1..+1)
 * @param dz Deadzone (0..0.5)
 * @param expo Exposition (0..0.95)
 * @return Valeur traitée
 */
float applyDeadzoneExpo(float x, float dz, float expo);

/**
 * @brief Applique un filtre IIR 1-pole
 * @param y Valeur filtrée précédente
 * @param x Valeur d'entrée
 * @param fc Fréquence de coupure (Hz)
 * @param dt Pas de temps (s)
 * @return Valeur filtrée
 */
float iir1Pole(float y, float x, float fc, float dt);

/**
 * @brief Applique une limitation de slew rate
 * @param y Valeur actuelle
 * @param x Valeur cible
 * @param rate Taux de changement max (unités/s)
 * @param dt Pas de temps (s)
 * @return Valeur limitée
 */
float slewLimit(float y, float x, float rate, float dt);

/**
 * @brief Obtient les valeurs brutes du joystick
 * @param pan Référence pour la valeur pan
 * @param tilt Référence pour la valeur tilt
 * @param slide Référence pour la valeur slide
 */
void getRawJoystickValues(float &pan, float &tilt, float &slide);

/**
 * @brief Obtient les valeurs filtrées du joystick
 * @param pan Référence pour la valeur pan
 * @param tilt Référence pour la valeur tilt
 * @param slide Référence pour la valeur slide
 */
void getFilteredJoystickValues(float &pan, float &tilt, float &slide);

/**
 * @brief Définit les valeurs brutes du joystick (via OSC)
 * @param pan Valeur pan (-1..+1)
 * @param tilt Valeur tilt (-1..+1)
 * @param slide Valeur slide (-1..+1)
 */
void setRawJoystickValues(float pan, float tilt, float slide);

/**
 * @brief Obtient les offsets actuels
 * @param pan Référence pour l'offset pan
 * @param tilt Référence pour l'offset tilt
 */
void getCurrentOffsets(long &pan, long &tilt);

/**
 * @brief Obtient les offsets latched
 * @param pan Référence pour l'offset pan latched
 * @param tilt Référence pour l'offset tilt latched
 */
void getLatchedOffsets(long &pan, long &tilt);

/**
 * @brief Définit les offsets latched
 * @param pan Offset pan latched
 * @param tilt Offset tilt latched
 */
void setLatchedOffsets(long pan, long tilt);

/**
 * @brief Ajoute des offsets aux offsets latched
 * @param pan Offset pan à ajouter
 * @param tilt Offset tilt à ajouter
 */
void addLatchedOffsets(long pan, long tilt);

/**
 * @brief Remet à zéro les offsets latched
 */
void resetLatchedOffsets();

/**
 * @brief Obtient la commande de jog slide
 * @return Commande de jog slide (-1..+1)
 */
float getSlideJogCommand();

/**
 * @brief Définit la commande de jog slide
 * @param cmd Commande de jog slide (-1..+1)
 */
void setSlideJogCommand(float cmd);

/**
 * @brief Obtient la configuration du joystick
 * @return Référence vers la configuration
 */
JoyCfg& getJoystickConfig();

/**
 * @brief Définit la configuration du joystick
 * @param config Nouvelle configuration
 */
void setJoystickConfig(const JoyCfg& config);

/**
 * @brief Obtient la politique d'annulation
 * @return Référence vers la politique
 */
CancelPolicy& getCancelPolicy();

/**
 * @brief Définit la politique d'annulation
 * @param policy Nouvelle politique
 */
void setCancelPolicy(const CancelPolicy& policy);

// Les fonctions de gestion des ranges et vitesses sont maintenant dans le module Config

/**
 * @brief Vérifie si le joystick pan est actif
 * @return true si actif, false sinon
 */
bool isPanActive();

/**
 * @brief Vérifie si le joystick tilt est actif
 * @return true si actif, false sinon
 */
bool isTiltActive();

/**
 * @brief Vérifie si le joystick slide est actif
 * @return true si actif, false sinon
 */
bool isSlideActive();

/**
 * @brief Obtient l'offset pan effectif selon le contexte
 * @param recallPhase true si en phase de recall
 * @return Offset pan effectif
 */
long getEffectivePanOffset(bool recallPhase);

/**
 * @brief Obtient l'offset tilt effectif selon le contexte
 * @param recallPhase true si en phase de recall
 * @return Offset tilt effectif
 */
long getEffectiveTiltOffset(bool recallPhase);

/**
 * @brief Sauvegarde la baseline d'offsets pour une session
 */
void saveOffsetBaseline();

/**
 * @brief Restaure la baseline d'offsets pour une session
 */
void restoreOffsetBaseline();

#endif // JOYSTICK_H
