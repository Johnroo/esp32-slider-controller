/**
 * @file NetworkManager.cpp
 * @brief Implémentation du module de gestion réseau avec portail captif
 * @author Laurent Eyen
 * @date 2024
 */

#include "NetworkManager.h"
#include "Config.h"
#include <WiFiManager.h>

// Instance globale WiFiManager
static WiFiManager wifiManager;

// Variables pour retry logic
static int connectionRetries = 0;
static const int MAX_RETRIES = 3;

// Callback pour sauvegarder les paramètres personnalisés
static bool shouldSaveConfig = false;

void saveConfigCallback() {
  Serial.println("💾 Configuration WiFi modifiée, sauvegarde nécessaire");
  shouldSaveConfig = true;
}

/**
 * @brief Initialise la connexion WiFi avec portail captif et configuration
 */
bool initNetwork() {
  Serial.println("🌐 Initialisation du réseau...");
  
  // Charger la configuration réseau depuis NVS
  loadNetworkConfig();
  
  // Configurer IP statique si nécessaire
  if (!networkConfig.useDHCP) {
    Serial.println("🔧 Configuration IP statique...");
    IPAddress ip, gateway, subnet, dns;
    
    if (ip.fromString(networkConfig.staticIP) &&
        gateway.fromString(networkConfig.gateway) &&
        subnet.fromString(networkConfig.subnet) &&
        dns.fromString(networkConfig.dns)) {
      
      WiFi.config(ip, gateway, subnet, dns);
      Serial.printf("📍 IP statique configurée: %s\n", networkConfig.staticIP);
    } else {
      Serial.println("⚠️ Erreur parsing IP statique, passage en DHCP");
      networkConfig.useDHCP = true;
    }
  }
  
  // Configurer le hostname WiFi
  WiFi.setHostname(networkConfig.hostname);
  
  // Définir le callback de sauvegarde
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  
  // Créer les paramètres personnalisés pour le portail
  WiFiManagerParameter customHostname("hostname", "Hostname (sans .local)", networkConfig.hostname, 31);
  WiFiManagerParameter customUseDHCP("dhcp", "Utiliser DHCP (1=Oui, 0=Non)", networkConfig.useDHCP ? "1" : "0", 1);
  WiFiManagerParameter customStaticIP("staticip", "IP Statique", networkConfig.staticIP, 15);
  WiFiManagerParameter customGateway("gateway", "Passerelle", networkConfig.gateway, 15);
  WiFiManagerParameter customSubnet("subnet", "Masque sous-réseau", networkConfig.subnet, 15);
  WiFiManagerParameter customDNS("dns", "Serveur DNS", networkConfig.dns, 15);
  
  // Ajouter les paramètres au WiFiManager
  wifiManager.addParameter(&customHostname);
  wifiManager.addParameter(&customUseDHCP);
  wifiManager.addParameter(&customStaticIP);
  wifiManager.addParameter(&customGateway);
  wifiManager.addParameter(&customSubnet);
  wifiManager.addParameter(&customDNS);
  
  // Configurer le timeout du portail captif
  wifiManager.setConfigPortalTimeout(180); // 3 minutes
  wifiManager.setConnectTimeout(15);       // 15 secondes pour la connexion
  wifiManager.setConnectRetries(3);        // 3 tentatives max
  
  // Mode non-bloquant si on veut juste se connecter sans portail
  wifiManager.setConfigPortalBlocking(false);
  
  // Tenter la connexion automatique
  bool connected = false;
  
  Serial.println("📡 Tentative de connexion WiFi...");
  
  // Essayer autoConnect non-bloquant d'abord
  connected = wifiManager.autoConnect("Slider-Setup");
  
  if (connected) {
    Serial.println("✅ WiFi connecté!");
    Serial.println("📍 IP: " + WiFi.localIP().toString());
    Serial.println("📍 Gateway: " + WiFi.gatewayIP().toString());
    Serial.println("📍 Subnet: " + WiFi.subnetMask().toString());
    Serial.println("📍 DNS: " + WiFi.dnsIP().toString());
    
    // Si les paramètres ont été modifiés, les sauvegarder
    if (shouldSaveConfig) {
      Serial.println("💾 Sauvegarde des nouveaux paramètres...");
      
      // Récupérer les valeurs entrées par l'utilisateur
      strncpy(networkConfig.hostname, customHostname.getValue(), 31);
      networkConfig.hostname[31] = '\0';
      
      networkConfig.useDHCP = (String(customUseDHCP.getValue()) == "1");
      
      strncpy(networkConfig.staticIP, customStaticIP.getValue(), 15);
      networkConfig.staticIP[15] = '\0';
      strncpy(networkConfig.gateway, customGateway.getValue(), 15);
      networkConfig.gateway[15] = '\0';
      strncpy(networkConfig.subnet, customSubnet.getValue(), 15);
      networkConfig.subnet[15] = '\0';
      strncpy(networkConfig.dns, customDNS.getValue(), 15);
      networkConfig.dns[15] = '\0';
      
      // Sauvegarder dans NVS
      saveNetworkConfig();
      
      shouldSaveConfig = false;
    }
    
    connectionRetries = 0;  // Reset compteur
    return true;
    
  } else {
    Serial.println("❌ Échec de connexion WiFi");
    connectionRetries++;
    
    if (connectionRetries >= MAX_RETRIES) {
      Serial.println("⚠️ Trop d'échecs de connexion, lancement du portail captif...");
      startConfigPortal();
      return false;
    }
    
    return false;
  }
}

/**
 * @brief Initialise mDNS avec le hostname configuré
 */
bool initMDNS() {
  Serial.println("🔍 Initialisation mDNS...");
  
  if (MDNS.begin(networkConfig.hostname)) {
    Serial.printf("✅ mDNS démarré: %s.local\n", networkConfig.hostname);
    Serial.printf("📍 Accessible à: http://%s.local\n", networkConfig.hostname);
    
    // Ajouter les services
    MDNS.addService("http", "tcp", WEB_SERVER_PORT);
    MDNS.addService("osc", "udp", OSC_PORT);
    
    Serial.println("📋 Services mDNS annoncés:");
    Serial.printf("  - HTTP sur port %d\n", WEB_SERVER_PORT);
    Serial.printf("  - OSC sur port %d\n", OSC_PORT);
    
    return true;
  } else {
    Serial.println("❌ Erreur démarrage mDNS");
    return false;
  }
}

/**
 * @brief Initialise OTA (Over-The-Air updates)
 */
void initOTA() {
  Serial.println("🔄 Initialisation OTA...");
  
  // Utiliser le hostname pour OTA
  ArduinoOTA.setHostname(networkConfig.hostname);
  
  // Callbacks OTA pour debug
  ArduinoOTA.onStart([]() {
    String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
    Serial.println("🔄 Début mise à jour OTA: " + type);
  });
  
  ArduinoOTA.onEnd([]() {
    Serial.println("\n✅ Mise à jour OTA terminée");
  });
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("⏳ Progression: %u%%\r", (progress / (total / 100)));
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("❌ Erreur OTA[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  
  ArduinoOTA.begin();
  
  Serial.printf("✅ OTA initialisé (hostname: %s)\n", networkConfig.hostname);
}

/**
 * @brief Gère les mises à jour OTA (à appeler dans loop())
 */
void handleOTA() {
  ArduinoOTA.handle();
}

/**
 * @brief Lance le portail captif de configuration réseau
 */
void startConfigPortal() {
  Serial.println("🔧 Lancement du portail de configuration...");
  Serial.println("📱 Connectez-vous au réseau WiFi: Slider-Setup");
  Serial.println("🌐 Puis allez sur: http://192.168.4.1");
  
  // Créer les paramètres pour le portail
  WiFiManagerParameter customHostname("hostname", "Hostname (sans .local)", networkConfig.hostname, 31);
  WiFiManagerParameter customUseDHCP("dhcp", "DHCP (1=Oui, 0=Non)", networkConfig.useDHCP ? "1" : "0", 1);
  WiFiManagerParameter customStaticIP("staticip", "IP Statique", networkConfig.staticIP, 15);
  WiFiManagerParameter customGateway("gateway", "Passerelle", networkConfig.gateway, 15);
  WiFiManagerParameter customSubnet("subnet", "Masque", networkConfig.subnet, 15);
  WiFiManagerParameter customDNS("dns", "DNS", networkConfig.dns, 15);
  
  // Ajouter les paramètres
  wifiManager.addParameter(&customHostname);
  wifiManager.addParameter(&customUseDHCP);
  wifiManager.addParameter(&customStaticIP);
  wifiManager.addParameter(&customGateway);
  wifiManager.addParameter(&customSubnet);
  wifiManager.addParameter(&customDNS);
  
  // Définir le callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  
  // Démarrer le portail (bloquant jusqu'à configuration)
  if (wifiManager.startConfigPortal("Slider-Setup")) {
    Serial.println("✅ Configuration WiFi réussie!");
    
    // Sauvegarder les nouveaux paramètres
    strncpy(networkConfig.hostname, customHostname.getValue(), 31);
    networkConfig.hostname[31] = '\0';
    
    networkConfig.useDHCP = (String(customUseDHCP.getValue()) == "1");
    
    strncpy(networkConfig.staticIP, customStaticIP.getValue(), 15);
    networkConfig.staticIP[15] = '\0';
    strncpy(networkConfig.gateway, customGateway.getValue(), 15);
    networkConfig.gateway[15] = '\0';
    strncpy(networkConfig.subnet, customSubnet.getValue(), 15);
    networkConfig.subnet[15] = '\0';
    strncpy(networkConfig.dns, customDNS.getValue(), 15);
    networkConfig.dns[15] = '\0';
    
    saveNetworkConfig();
    
    Serial.println("🔄 Redémarrage pour appliquer la configuration...");
    delay(1000);
    ESP.restart();
  } else {
    Serial.println("❌ Échec configuration WiFi");
  }
}

/**
 * @brief Réinitialise la configuration WiFi et redémarre en mode AP
 */
void resetWiFiAndRestart() {
  Serial.println("🔄 Réinitialisation configuration réseau...");
  
  // Effacer les credentials WiFi du WiFiManager
  wifiManager.resetSettings();
  
  // Déconnecter et effacer la config WiFi de l'ESP32
  WiFi.disconnect(true, true);  // disconnect + erase
  
  // Effacer la config réseau NVS
  resetNetworkConfig();
  
  Serial.println("🔄 Redémarrage en mode configuration...");
  Serial.println("📱 L'ESP32 va créer le point d'accès: Slider-Setup");
  
  delay(2000);
  ESP.restart();
}
