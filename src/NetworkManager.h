/**
 * @file NetworkManager.h
 * @brief Module de gestion réseau - WiFi et OTA
 * @author Laurent Eyen
 * @date 2024
 */

#ifndef NETWORKMANAGER_H
#define NETWORKMANAGER_H

#include <Arduino.h>
#include <WiFiManager.h>
#include <ArduinoOTA.h>
#include <WiFi.h>

//==================== Fonctions du module ====================

/**
 * @brief Initialise la connexion WiFi et affiche l'IP
 */
void initNetwork();

/**
 * @brief Initialise OTA (Over-The-Air updates)
 */
void initOTA();

/**
 * @brief Gère les mises à jour OTA (à appeler dans loop())
 */
void handleOTA();

#endif // NETWORKMANAGER_H
