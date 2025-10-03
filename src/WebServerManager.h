/**
 * @file WebServerManager.h
 * @brief Module de gestion du serveur web AsyncWebServer
 * @author Laurent Eyen
 * @date 2024
 */

#ifndef WEBSERVERMANAGER_H
#define WEBSERVERMANAGER_H

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

//==================== Configuration ====================

// Port du serveur web (défini dans Config.h)

//==================== Variables globales ====================

// Serveur web AsyncWebServer
extern AsyncWebServer webServer;

//==================== Fonctions du module ====================

/**
 * @brief Initialise le serveur web et configure toutes les routes
 */
void initWebServer();

#endif // WEBSERVERMANAGER_H
