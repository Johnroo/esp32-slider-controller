/**
 * @file NetworkManager.cpp
 * @brief Implémentation du module de gestion réseau
 * @author Laurent Eyen
 * @date 2024
 */

#include "NetworkManager.h"

//==================== Fonctions du module ====================

/**
 * @brief Initialise la connexion WiFi et affiche l'IP
 */
void initNetwork() {
  Serial.println("🌐 Initialisation du WiFi...");
  
  // WiFi Manager
  WiFiManager wm;
  wm.autoConnect("ESP32-Slider");
  
  Serial.println("📡 WiFi connected: " + WiFi.localIP().toString());
}

/**
 * @brief Initialise OTA (Over-The-Air updates)
 */
void initOTA() {
  Serial.println("🔄 Initialisation OTA...");
  
  ArduinoOTA.begin();
  
  Serial.println("✅ OTA initialisé");
}

/**
 * @brief Gère les mises à jour OTA (à appeler dans loop())
 */
void handleOTA() {
  ArduinoOTA.handle();
}
