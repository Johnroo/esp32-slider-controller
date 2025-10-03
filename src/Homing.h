/**
 * @file Homing.h
 * @brief Module de gestion du homing des moteurs avec StallGuard (TMC2209)
 * @author Laurent Eyen
 * @date 2024
 */

#ifndef HOMING_H
#define HOMING_H

#include <Arduino.h>
#include "MotorControl.h"
#include "Config.h"

// Les constantes de homing sont maintenant dans le module Config

//==================== Variables globales ====================
extern bool doAutoHomeSlide;     // lancer automatiquement au démarrage si true
extern bool homingInProgress;    // flag pour éviter les homing multiples

//==================== Fonctions du module ====================
/**
 * @brief Initialise le système de homing (StallGuard)
 * @details Configure les interruptions et les paramètres StallGuard
 */
void initHoming();

/**
 * @brief Effectue le homing complet du slide
 * @details Utilise StallGuard pour détecter les butées et calibrer l'axe
 */
void homeSlide();

/**
 * @brief Configure le seuil StallGuard pour le slide
 * @param threshold Nouveau seuil (0-255)
 */
void setSlideSGThreshold(uint8_t threshold);

/**
 * @brief Obtient le seuil StallGuard actuel du slide
 * @return Seuil actuel (0-255)
 */
uint8_t getSlideSGThreshold();

/**
 * @brief Vérifie si un homing est en cours
 * @return true si homing en cours, false sinon
 */
bool isHomingInProgress();

#endif // HOMING_H
