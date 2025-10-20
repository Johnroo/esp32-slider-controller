/**
 * @file NetworkManager.h
 * @brief Module de gestion réseau - WiFi et OTA
 * @author Laurent Eyen
 * @date 2024
 */

#ifndef NETWORKMANAGER_H
#define NETWORKMANAGER_H

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>
#include <ESPmDNS.h>

//==================== Fonctions du module ====================

/**
 * @brief Initialise la connexion WiFi avec portail captif et configuration
 * @return true si connexion réussie, false sinon
 */
bool initNetwork();

/**
 * @brief Initialise mDNS avec le hostname configuré
 * @return true si mDNS démarré avec succès
 */
bool initMDNS();

/**
 * @brief Initialise OTA (Over-The-Air updates)
 */
void initOTA();

/**
 * @brief Gère les mises à jour OTA (à appeler dans loop())
 */
void handleOTA();

/**
 * @brief Lance le portail captif de configuration réseau
 */
void startConfigPortal();

/**
 * @brief Réinitialise la configuration WiFi et redémarre en mode AP
 */
void resetWiFiAndRestart();

#endif // NETWORKMANAGER_H
