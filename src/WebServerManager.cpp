/**
 * @file WebServerManager.cpp
 * @brief Implémentation du module de gestion du serveur web
 * @author Laurent Eyen
 * @date 2024
 */

#include "WebServerManager.h"
#include "Config.h"
#include "MotorControl.h"
#include "Presets.h"
#include "MotionPlanner.h"
#include "NetworkManager.h"

//==================== Variables globales ====================

// Serveur web AsyncWebServer
AsyncWebServer webServer(WEB_SERVER_PORT);

//==================== Fonctions du module ====================

/**
 * @brief Initialise le serveur web et configure toutes les routes
 */
void initWebServer() {
  Serial.println("🌐 Configuration du serveur web...");
  
  webServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", "ESP32 Slider Controller - OSC Server Running");
  });
  
  webServer.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request){
    // Créer un JSON avec les positions des moteurs et les points d'interpolation
    DynamicJsonDocument doc(2048);
    
    // Positions actuelles des 4 moteurs
    doc["motors"]["pan"] = steppers[0]->targetPos();
    doc["motors"]["tilt"] = steppers[1]->targetPos();
    doc["motors"]["zoom"] = steppers[2]->targetPos();
    doc["motors"]["slide"] = steppers[3]->targetPos();
    
    // Positions en pourcentage (0-100)
    doc["motors_percent"]["pan"] = (float)(steppers[0]->targetPos() - cfg[0].min_limit) / (cfg[0].max_limit - cfg[0].min_limit) * 100.0f;
    doc["motors_percent"]["tilt"] = (float)(steppers[1]->targetPos() - cfg[1].min_limit) / (cfg[1].max_limit - cfg[1].min_limit) * 100.0f;
    doc["motors_percent"]["zoom"] = (float)(steppers[2]->targetPos() - cfg[2].min_limit) / (cfg[2].max_limit - cfg[2].min_limit) * 100.0f;
    doc["motors_percent"]["slide"] = (float)(steppers[3]->targetPos() - cfg[3].min_limit) / (cfg[3].max_limit - cfg[3].min_limit) * 100.0f;
    
    // Points d'interpolation actuels
    doc["interpolation"]["count"] = interpCount;
    JsonArray interpArray = doc["interpolation"].createNestedArray("points");
    for (int i = 0; i < interpCount; i++) {
      JsonObject point = interpArray.createNestedObject();
      point["presetIndex"] = interpPoints[i].presetIndex;
      point["fraction"] = interpPoints[i].fraction * 100.0f; // Convertir en pourcentage
    }
    
    // État des modes actifs
    doc["modes"]["interpAuto"] = interpAuto.active;
    doc["modes"]["syncMove"] = isActive();
    doc["modes"]["slideAB"] = false; // Mode slideAB supprimé dans la refactorisation
    
    // Banque active
    doc["bank"]["active"] = activeBank;
    
    // Sérialiser en JSON
    String jsonString;
    serializeJson(doc, jsonString);
    
    request->send(200, "application/json", jsonString);
  });
  
  webServer.on("/api/interpolation", HTTP_GET, [](AsyncWebServerRequest *request){
    // Route spécifique pour les points d'interpolation
    DynamicJsonDocument doc(1024);
    
    doc["interpCount"] = interpCount;
    JsonArray interpArray = doc.createNestedArray("interp");
    for (int i = 0; i < interpCount; i++) {
      JsonObject point = interpArray.createNestedObject();
      point["presetIndex"] = interpPoints[i].presetIndex;
      point["fraction"] = interpPoints[i].fraction * 100.0f; // Convertir en pourcentage
    }
    
    String jsonString;
    serializeJson(doc, jsonString);
    request->send(200, "application/json", jsonString);
  });
  
  //==================== NEW: Axes Status Endpoint ====================
  webServer.on("/api/axes/status", HTTP_GET, [](AsyncWebServerRequest *req){
    Serial.println("📡 /api/axes/status requested");
    DynamicJsonDocument doc(256);
    // Normalise les positions entre 0 et 1 selon min/max de chaque axe
    auto norm = [](long pos, long min, long max){
      return (max!=min)? (float)(pos - min) / (float)(max - min) : 0.0f;
    };
    
    long pan_pos = steppers[0]->getCurrentPosition();
    long tilt_pos = steppers[1]->getCurrentPosition();
    long zoom_pos = steppers[2]->getCurrentPosition();
    long slide_pos = steppers[3]->getCurrentPosition();
    
    doc["pan"]   = norm(pan_pos, cfg[0].min_limit, cfg[0].max_limit);
    doc["tilt"]  = norm(tilt_pos, cfg[1].min_limit, cfg[1].max_limit);
    doc["zoom"]  = norm(zoom_pos, cfg[2].min_limit, cfg[2].max_limit);
    doc["slide"] = norm(slide_pos, cfg[3].min_limit, cfg[3].max_limit);
    
    String out; 
    serializeJson(doc, out);
    Serial.printf("📊 Axes status: %s\n", out.c_str());
    req->send(200, "application/json", out);
  });
  
  // Test endpoint simple
  webServer.on("/api/test", HTTP_GET, [](AsyncWebServerRequest *req){
    Serial.println("🧪 /api/test requested");
    req->send(200, "application/json", "{\"status\":\"ok\",\"message\":\"test endpoint works\"}");
  });
  
  //==================== Bank Export Endpoint ====================
  webServer.on("/api/bank/*", HTTP_GET, [](AsyncWebServerRequest *req){
    String url = req->url();
    // Extraire l'ID de la banque de l'URL (ex: /api/bank/0 -> 0)
    int bankId = url.substring(url.lastIndexOf('/') + 1).toInt();
    Serial.printf("📤 /api/bank/%d requested\n", bankId);
    
    if (bankId < 0 || bankId >= 10) {
      req->send(400, "application/json", "{\"error\":\"Invalid bank ID\"}");
      return;
    }
    
    DynamicJsonDocument doc(4096);
    
    // Récupérer les presets de la banque
    JsonArray presetsArray = doc.createNestedArray("presets");
    for (int i = 0; i < 8; i++) {
      JsonObject preset = presetsArray.createNestedObject();
      Preset p = banks[bankId].presets[i];
      preset["p"] = p.p;
      preset["t"] = p.t;
      preset["z"] = p.z;
      preset["s"] = p.s;
    }
    
    // Récupérer les points d'interpolation de la banque
    doc["interpCount"] = banks[bankId].interpCount;
    JsonArray interpArray = doc.createNestedArray("interpPoints");
    for (int i = 0; i < banks[bankId].interpCount; i++) {
      JsonObject point = interpArray.createNestedObject();
      point["presetIndex"] = banks[bankId].interpPoints[i].presetIndex;
      point["fraction"] = banks[bankId].interpPoints[i].fraction * 100.0f; // Convertir en pourcentage
    }
    
    String jsonString;
    serializeJson(doc, jsonString);
    req->send(200, "application/json", jsonString);
  });
  
  //==================== Network Configuration Routes ====================
  
  // Route pour obtenir les informations réseau
  webServer.on("/api/network/info", HTTP_GET, [](AsyncWebServerRequest *request){
    DynamicJsonDocument doc(512);
    doc["hostname"] = networkConfig.hostname;
    doc["ip"] = WiFi.localIP().toString();
    doc["gateway"] = WiFi.gatewayIP().toString();
    doc["subnet"] = WiFi.subnetMask().toString();
    doc["dns"] = WiFi.dnsIP().toString();
    doc["useDHCP"] = networkConfig.useDHCP;
    doc["staticIP"] = networkConfig.staticIP;
    doc["staticGateway"] = networkConfig.gateway;
    doc["staticSubnet"] = networkConfig.subnet;
    doc["staticDNS"] = networkConfig.dns;
    
    String jsonString;
    serializeJson(doc, jsonString);
    request->send(200, "application/json", jsonString);
  });
  
  // Route pour réinitialiser la configuration réseau
  webServer.on("/api/network/reset", HTTP_POST, [](AsyncWebServerRequest *request){
    DynamicJsonDocument doc(128);
    doc["success"] = true;
    doc["message"] = "Réseau réinitialisé, redémarrage en mode configuration...";
    
    String jsonString;
    serializeJson(doc, jsonString);
    request->send(200, "application/json", jsonString);
    
    delay(500);
    resetWiFiAndRestart();
  });
  
  // Route pour redémarrer l'ESP32
  webServer.on("/api/system/restart", HTTP_POST, [](AsyncWebServerRequest *request){
    DynamicJsonDocument doc(128);
    doc["success"] = true;
    doc["message"] = "Redémarrage...";
    
    String jsonString;
    serializeJson(doc, jsonString);
    request->send(200, "application/json", jsonString);
    
    delay(500);
    ESP.restart();
  });
  
  webServer.begin();
  Serial.println("🌐 Web server started");
}
