/**
 * @file Config.cpp
 * @brief Implémentation de la configuration centralisée
 * @author Laurent Eyen
 * @date 2024
 */

#include "Config.h"
#include <Preferences.h>

//==================== Variables globales configurables ====================

// Ranges d'offsets (modifiables via OSC)
long PAN_OFFSET_RANGE = DEFAULT_PAN_SLEW_RANGE;
long TILT_OFFSET_RANGE = DEFAULT_TILT_SLEW_RANGE;
long ZOOM_OFFSET_RANGE = DEFAULT_ZOOM_SLEW_RANGE;
long SLIDE_OFFSET_RANGE = DEFAULT_SLIDE_SLEW_RANGE;

// Vitesses de jog (calculées dans setup)
float PAN_JOG_SPEED = DEFAULT_PAN_JOG_SPEED;
float TILT_JOG_SPEED = DEFAULT_TILT_JOG_SPEED;
float ZOOM_JOG_SPEED = DEFAULT_ZOOM_JOG_SPEED;
float SLIDE_JOG_SPEED = DEFAULT_SLIDE_JOG_SPEED;

// Durée par défaut des mouvements
uint32_t DEFAULT_MOVE_DURATION_MS = DEFAULT_MOVE_DURATION;

// Configuration réseau
NetworkConfig networkConfig = {
  DEFAULT_HOSTNAME,  // hostname
  true,              // useDHCP
  "192.168.1.100",   // staticIP
  "192.168.1.1",     // gateway
  "255.255.255.0",   // subnet
  "8.8.8.8"          // dns
};

//==================== Configuration matérielle ====================

// Pins STEP/DIR/EN pour chaque moteur (ATTENTION: Ne pas modifier sans vérification!)
const int STEP_PINS[NUM_MOTORS] = {18, 21, 23, 26};
const int DIR_PINS[NUM_MOTORS] = {19, 22, 25, 27};
const int ENABLE_PINS[NUM_MOTORS] = {13, 14, 32, 33};

//==================== Fonctions de configuration ====================

/**
 * @brief Initialise la configuration globale
 */
void initConfig() {
  Serial.println("⚙️ Initialisation de la configuration...");
  
  // Initialiser les ranges d'offsets
  PAN_OFFSET_RANGE = DEFAULT_PAN_SLEW_RANGE;
  TILT_OFFSET_RANGE = DEFAULT_TILT_SLEW_RANGE;
  ZOOM_OFFSET_RANGE = DEFAULT_ZOOM_SLEW_RANGE;
  SLIDE_OFFSET_RANGE = DEFAULT_SLIDE_SLEW_RANGE;
  
  // Initialiser les vitesses de jog
  PAN_JOG_SPEED = DEFAULT_PAN_JOG_SPEED;
  TILT_JOG_SPEED = DEFAULT_TILT_JOG_SPEED;
  ZOOM_JOG_SPEED = DEFAULT_ZOOM_JOG_SPEED;
  SLIDE_JOG_SPEED = DEFAULT_SLIDE_JOG_SPEED;
  
  // Initialiser la durée par défaut
  DEFAULT_MOVE_DURATION_MS = DEFAULT_MOVE_DURATION;
  
  Serial.println("✅ Configuration initialisée");
}

/**
 * @brief Obtient les vitesses de jog actuelles
 */
void getJogSpeeds(float &pan, float &tilt, float &zoom, float &slide) {
  pan = PAN_JOG_SPEED;
  tilt = TILT_JOG_SPEED;
  zoom = ZOOM_JOG_SPEED;
  slide = SLIDE_JOG_SPEED;
}

/**
 * @brief Définit les vitesses de jog
 */
void setJogSpeeds(float pan, float tilt, float zoom, float slide) {
  PAN_JOG_SPEED = pan;
  TILT_JOG_SPEED = tilt;
  ZOOM_JOG_SPEED = zoom;
  SLIDE_JOG_SPEED = slide;
  
  Serial.printf("🎯 Vitesses de jog mises à jour: Pan=%.0f Tilt=%.0f Zoom=%.0f Slide=%.0f steps/s\n", 
                PAN_JOG_SPEED, TILT_JOG_SPEED, ZOOM_JOG_SPEED, SLIDE_JOG_SPEED);
}

/**
 * @brief Obtient les ranges d'offsets actuels
 */
void getOffsetRanges(long &pan, long &tilt, long &zoom, long &slide) {
  pan = PAN_OFFSET_RANGE;
  tilt = TILT_OFFSET_RANGE;
  zoom = ZOOM_OFFSET_RANGE;
  slide = SLIDE_OFFSET_RANGE;
}

/**
 * @brief Définit les ranges d'offsets
 */
void setOffsetRanges(long pan, long tilt, long zoom, long slide) {
  PAN_OFFSET_RANGE = pan;
  TILT_OFFSET_RANGE = tilt;
  ZOOM_OFFSET_RANGE = zoom;
  SLIDE_OFFSET_RANGE = slide;
  
  Serial.printf("📏 Ranges d'offsets mis à jour: Pan=%ld Tilt=%ld Zoom=%ld Slide=%ld\n", 
                PAN_OFFSET_RANGE, TILT_OFFSET_RANGE, ZOOM_OFFSET_RANGE, SLIDE_OFFSET_RANGE);
}

/**
 * @brief Obtient la durée par défaut des mouvements
 */
uint32_t getDefaultMoveDuration() {
  return DEFAULT_MOVE_DURATION_MS;
}

/**
 * @brief Définit la durée par défaut des mouvements
 */
void setDefaultMoveDuration(uint32_t durationMs) {
  DEFAULT_MOVE_DURATION_MS = durationMs;
  
  Serial.printf("⏱️ Durée par défaut des mouvements: %u ms\n", DEFAULT_MOVE_DURATION_MS);
}

/**
 * @brief Affiche la configuration actuelle
 */
void printConfig() {
  Serial.println("\n📋 Configuration actuelle:");
  Serial.println("==========================");
  
  Serial.printf("🔧 Matériel:\n");
  Serial.printf("  - Nombre de moteurs: %d\n", NUM_MOTORS);
  Serial.printf("  - UART TX/RX: %d/%d\n", UART_TX, UART_RX);
  Serial.printf("  - R_SENSE: %.2f\n", R_SENSE);
  
  Serial.printf("🎯 Vitesses de jog: Pan=%.0f Tilt=%.0f Zoom=%.0f Slide=%.0f steps/s\n", 
                PAN_JOG_SPEED, TILT_JOG_SPEED, ZOOM_JOG_SPEED, SLIDE_JOG_SPEED);
  
  Serial.printf("📏 Ranges d'offsets: Pan=%ld Tilt=%ld Zoom=%ld Slide=%ld\n", 
                PAN_OFFSET_RANGE, TILT_OFFSET_RANGE, ZOOM_OFFSET_RANGE, SLIDE_OFFSET_RANGE);
  
  Serial.printf("⏱️ Durée par défaut: %u ms\n", DEFAULT_MOVE_DURATION_MS);
  
  Serial.printf("🏠 Homing:\n");
  Serial.printf("  - Vitesse: %d steps/s\n", HOMING_SPEED);
  Serial.printf("  - Accélération: %d steps/s²\n", HOMING_ACCEL);
  Serial.printf("  - Seuil StallGuard: %d\n", SG_DETECT);
  Serial.printf("  - Timeout: %d ms\n", HOMING_TIMEOUT);
  Serial.printf("  - Recul: %d pas\n", BACKOFF_STEPS);
  Serial.printf("  - Délai: %d ms\n", BACKOFF_DELAY);
  Serial.printf("  - Marge sécurité: %d pas\n", SAFETY_STEPS);
  
  Serial.printf("📊 Presets:\n");
  Serial.printf("  - Max presets: %d\n", MAX_PRESETS);
  Serial.printf("  - Max points interp: %d\n", MAX_INTERP_POINTS);
  Serial.printf("  - Max banques: %d\n", MAX_BANKS);
  
  Serial.printf("🎮 Joystick:\n");
  Serial.printf("  - Deadzone: %.2f\n", DEFAULT_JOYSTICK_DEADZONE);
  Serial.printf("  - Expo: %.2f\n", DEFAULT_JOYSTICK_EXPO);
  Serial.printf("  - Slew rate: %.0f/s\n", DEFAULT_JOYSTICK_SLEW_PER_S);
  Serial.printf("  - Filtre: %.1f Hz\n", DEFAULT_JOYSTICK_FILT_HZ);
  
  Serial.printf("🌐 Réseau:\n");
  Serial.printf("  - Port OSC: %d\n", OSC_PORT);
  Serial.printf("  - Port Web: %d\n", WEB_SERVER_PORT);
  Serial.printf("  - Hostname: %s.local\n", networkConfig.hostname);
  Serial.printf("  - Mode: %s\n", networkConfig.useDHCP ? "DHCP" : "Static IP");
  if (!networkConfig.useDHCP) {
    Serial.printf("  - IP: %s\n", networkConfig.staticIP);
    Serial.printf("  - Gateway: %s\n", networkConfig.gateway);
    Serial.printf("  - Subnet: %s\n", networkConfig.subnet);
    Serial.printf("  - DNS: %s\n", networkConfig.dns);
  }
  
  Serial.println("==========================\n");
}

/**
 * @brief Charge la configuration réseau depuis NVS
 */
void loadNetworkConfig() {
  Preferences prefs;
  if (!prefs.begin("network", true)) {  // read-only
    Serial.println("⚠️ Erreur lecture NVS network, utilisation config par défaut");
    return;
  }
  
  String hostname = prefs.getString("hostname", DEFAULT_HOSTNAME);
  if (hostname.length() > 0 && hostname.length() < 32) {
    strncpy(networkConfig.hostname, hostname.c_str(), 31);
    networkConfig.hostname[31] = '\0';
  }
  
  networkConfig.useDHCP = prefs.getBool("useDHCP", true);
  
  String staticIP = prefs.getString("staticIP", "192.168.1.100");
  String gateway = prefs.getString("gateway", "192.168.1.1");
  String subnet = prefs.getString("subnet", "255.255.255.0");
  String dns = prefs.getString("dns", "8.8.8.8");
  
  strncpy(networkConfig.staticIP, staticIP.c_str(), 15);
  networkConfig.staticIP[15] = '\0';
  strncpy(networkConfig.gateway, gateway.c_str(), 15);
  networkConfig.gateway[15] = '\0';
  strncpy(networkConfig.subnet, subnet.c_str(), 15);
  networkConfig.subnet[15] = '\0';
  strncpy(networkConfig.dns, dns.c_str(), 15);
  networkConfig.dns[15] = '\0';
  
  prefs.end();
  
  Serial.printf("📂 Config réseau chargée: %s.local (%s)\n", 
                networkConfig.hostname, 
                networkConfig.useDHCP ? "DHCP" : networkConfig.staticIP);
}

/**
 * @brief Sauvegarde la configuration réseau dans NVS
 */
void saveNetworkConfig() {
  Preferences prefs;
  if (!prefs.begin("network", false)) {  // read-write
    Serial.println("❌ Erreur sauvegarde config réseau dans NVS");
    return;
  }
  
  prefs.putString("hostname", networkConfig.hostname);
  prefs.putBool("useDHCP", networkConfig.useDHCP);
  prefs.putString("staticIP", networkConfig.staticIP);
  prefs.putString("gateway", networkConfig.gateway);
  prefs.putString("subnet", networkConfig.subnet);
  prefs.putString("dns", networkConfig.dns);
  
  prefs.end();
  
  Serial.printf("💾 Config réseau sauvegardée: %s.local (%s)\n", 
                networkConfig.hostname,
                networkConfig.useDHCP ? "DHCP" : networkConfig.staticIP);
}

/**
 * @brief Réinitialise la configuration réseau aux valeurs par défaut
 */
void resetNetworkConfig() {
  Preferences prefs;
  if (prefs.begin("network", false)) {  // read-write
    prefs.clear();
    prefs.end();
    Serial.println("🔄 Configuration réseau effacée de NVS");
  }
  
  // Réinitialiser en mémoire
  strncpy(networkConfig.hostname, DEFAULT_HOSTNAME, 31);
  networkConfig.hostname[31] = '\0';
  networkConfig.useDHCP = true;
  strncpy(networkConfig.staticIP, "192.168.1.100", 15);
  strncpy(networkConfig.gateway, "192.168.1.1", 15);
  strncpy(networkConfig.subnet, "255.255.255.0", 15);
  strncpy(networkConfig.dns, "8.8.8.8", 15);
  
  Serial.println("🔄 Configuration réseau réinitialisée");
}
