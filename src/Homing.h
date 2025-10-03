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

//==================== Configuration Homing ====================
#define SLIDE_INDEX     3
#define HOMING_SPEED    9000     // steps/s (2000-4000 range pour SG4)
#define HOMING_ACCEL    20000    // accel élevée pour atteindre vitesse rapidement
#define SG_DETECT       200      // seuil SG_RESULT pour détecter stall (commencer à ~100)
#define HOMING_TIMEOUT  20000    // ms
#define BACKOFF_STEPS   300      // pas de recul après détection

//==================== Variables globales ====================
extern bool doAutoHomeSlide;     // lancer automatiquement au démarrage si true
extern uint8_t slide_sg_threshold; // SGTHRS par défaut (sensibilité moyenne)

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

#endif // HOMING_H
