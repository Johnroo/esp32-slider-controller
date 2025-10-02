/**
 * @file Homing.h
 * @brief Module de gestion du homing (référencement) des moteurs avec StallGuard
 * 
 * Ce module encapsule toute la logique de homing :
 * - Configuration StallGuard pour TMC2209
 * - Procédures de référencement automatique
 * - Gestion des interruptions de stall
 * - Homing du slide avec détection de butée
 */

#ifndef HOMING_H
#define HOMING_H

#include <Arduino.h>
#include <TMCStepper.h>
#include "MotorControl.h"

//==================== Configuration Homing ====================
#define SLIDE_INDEX     3           // Index du moteur slide dans le tableau

// Paramètres de homing
#define HOMING_SPEED    9000        // steps/s (2000-4000 range pour SG4)
#define HOMING_ACCEL    90000       // accel élevée pour atteindre vitesse rapidement
#define SG_DETECT       100         // seuil SG_RESULT pour détecter stall (commencer à ~100)
#define HOMING_TIMEOUT  20000       // ms
#define BACKOFF_STEPS   300         // pas de recul après détection

// Variables globales de configuration
extern bool doAutoHomeSlide;        // lancer automatiquement au démarrage si true
extern uint8_t slide_sg_threshold;  // SGTHRS par défaut (sensibilité moyenne)

//==================== Fonctions d'initialisation ====================
/**
 * @brief Initialise le module de homing
 * 
 * Configure StallGuard pour tous les drivers TMC2209
 * et prépare les paramètres nécessaires au homing
 */
void initHoming();

/**
 * @brief Configure StallGuard pour un driver spécifique
 * @param driverIndex Index du driver dans le tableau drivers[]
 * @param threshold Seuil StallGuard (0-255)
 */
void setupStallGuard(int driverIndex, uint8_t threshold);

//==================== Fonctions de homing ====================
/**
 * @brief Lance le homing du slide
 * 
 * Procédure complète de homing du slide :
 * 1. Configuration StallGuard
 * 2. Déplacement vers butée inférieure
 * 3. Déplacement vers butée supérieure
 * 4. Centrage
 * 5. Mise à zéro de la position
 */
void homeSlide();

/**
 * @brief Lance le homing de tous les axes
 * 
 * Pour l'instant, seul le slide est homé automatiquement.
 * Les autres axes (PAN, TILT, ZOOM) restent en position actuelle.
 */
void homeAllAxes();

//==================== Fonctions utilitaires ====================
/**
 * @brief Vérifie si le homing automatique est activé
 * @return true si le homing automatique doit être lancé au démarrage
 */
bool isAutoHomeEnabled();

/**
 * @brief Active/désactive le homing automatique
 * @param enabled true pour activer, false pour désactiver
 */
void setAutoHomeEnabled(bool enabled);

/**
 * @brief Obtient le seuil StallGuard actuel du slide
 * @return Seuil StallGuard (0-255)
 */
uint8_t getSlideSGThreshold();

/**
 * @brief Définit le seuil StallGuard du slide
 * @param threshold Nouveau seuil (0-255)
 */
void setSlideSGThreshold(uint8_t threshold);

//==================== Fonctions de détection de stall ====================
/**
 * @brief Vérifie si un stall est détecté sur un driver
 * @param driverIndex Index du driver à vérifier
 * @return true si un stall est détecté
 */
bool isStallDetected(int driverIndex);

/**
 * @brief Callback d'interruption StallGuard
 * 
 * Cette fonction est appelée automatiquement par l'interruption
 * StallGuard quand un stall est détecté.
 * Doit être marquée IRAM_ATTR pour ESP32.
 */
void IRAM_ATTR stallGuardCallback();

//==================== Variables d'état ====================
extern volatile bool stall_detected;    // Flag global de détection de stall
extern volatile bool homing_in_progress; // Flag indiquant qu'un homing est en cours

#endif // HOMING_H
