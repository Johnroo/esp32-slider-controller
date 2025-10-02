/**
 * @file Presets.cpp
 * @brief Implémentation du module de gestion des presets et interpolation
 */

#include "Presets.h"
#include <Preferences.h>

//==================== Variables globales ====================
Preset presets[NUM_PRESETS];           // Presets actifs
int activePreset = -1;                 // Preset actuellement actif (-1 = aucun)
Bank banks[NUM_BANKS];                 // Banques de presets
uint8_t activeBank = 0;                // Banque active (0-9)
InterpPoint interpPoints[MAX_INTERP_POINTS];  // Points d'interpolation actifs
uint8_t interpCount = 2;               // Nombre de points d'interpolation (par défaut: A@0%, B@100%)
InterpAuto interpAuto = {false, 0, 0, 1};  // Mode interpolation automatique
float interp_jog_cmd = 0.0f;           // Commande de jog manuel (-1..+1)

//==================== Fonctions utilitaires ====================
static inline long clampL(long v, long vmin, long vmax){ return v < vmin ? vmin : (v > vmax ? vmax : v); }
static inline float clampF(float v, float vmin, float vmax){ return v < vmin ? vmin : (v > vmax ? vmax : v); }
static inline float lerp(float a, float b, float u){ return a + (b - a) * u; }

//==================== Fonctions d'initialisation ====================
void initPresets() {
  Serial.println("🎯 Initialisation du module Presets...");
  
  // Initialiser les presets par défaut
  for (int i = 0; i < NUM_PRESETS; i++) {
    presets[i] = {0, 0, 0, 0};
  }
  
  // Initialiser les points d'interpolation par défaut (A@0%, B@100%)
  interpPoints[0] = {0, 0.0f};  // Preset 0 à 0%
  interpPoints[1] = {1, 1.0f};  // Preset 1 à 100%
  interpCount = 2;
  
  // Initialiser les banques
  for (int b = 0; b < NUM_BANKS; b++) {
    for (int i = 0; i < NUM_PRESETS; i++) {
      banks[b].presets[i] = {0, 0, 0, 0};
    }
    banks[b].interpPoints[0] = {0, 0.0f};
    banks[b].interpPoints[1] = {1, 1.0f};
    banks[b].interpCount = 2;
  }
  
  Serial.println("✅ Module Presets initialisé");
}

//==================== Fonctions de gestion des presets ====================
void savePreset(uint8_t index) {
  if (index >= NUM_PRESETS) {
    Serial.printf("❌ Index preset invalide: %d\n", index);
    return;
  }
  
  // Sauvegarder la position actuelle des moteurs
  presets[index].p = panPos;
  presets[index].t = tiltPos;
  presets[index].z = zoomPos;
  presets[index].s = slidePos;
  
  // Mettre à jour la banque active
  banks[activeBank].presets[index] = presets[index];
  
  Serial.printf("💾 Preset %d sauvegardé: P=%ld T=%ld Z=%ld S=%ld\n", 
                index, presets[index].p, presets[index].t, presets[index].z, presets[index].s);
}

void loadPreset(uint8_t index) {
  if (index >= NUM_PRESETS) {
    Serial.printf("❌ Index preset invalide: %d\n", index);
    return;
  }
  
  const Preset& preset = presets[index];
  
  // Déplacer les moteurs vers les positions du preset
  moveMotorTo(0, preset.p);  // PAN
  moveMotorTo(1, preset.t);  // TILT
  moveMotorTo(2, preset.z);  // ZOOM
  moveMotorTo(3, preset.s);  // SLIDE
  
  activePreset = index;
  
  Serial.printf("🎯 Preset %d chargé: P=%ld T=%ld Z=%ld S=%ld\n", 
                index, preset.p, preset.t, preset.z, preset.s);
}

const Preset& getPreset(uint8_t index) {
  static Preset defaultPreset = {0, 0, 0, 0};
  
  if (index >= NUM_PRESETS) {
    return defaultPreset;
  }
  
  return presets[index];
}

void setPreset(uint8_t index, long p, long t, long z, long s) {
  if (index >= NUM_PRESETS) {
    Serial.printf("❌ Index preset invalide: %d\n", index);
    return;
  }
  
  presets[index].p = p;
  presets[index].t = t;
  presets[index].z = z;
  presets[index].s = s;
  
  // Mettre à jour la banque active
  banks[activeBank].presets[index] = presets[index];
  
  Serial.printf("📝 Preset %d défini: P=%ld T=%ld Z=%ld S=%ld\n", 
                index, p, t, z, s);
}

int getActivePreset() {
  return activePreset;
}

void setActivePreset(int index) {
  if (index >= -1 && index < NUM_PRESETS) {
    activePreset = index;
    Serial.printf("🎯 Preset actif: %d\n", index);
  }
}

//==================== Fonctions de gestion des banques ====================
void saveBank(uint8_t bankIndex) {
  if (bankIndex >= NUM_BANKS) {
    Serial.printf("❌ Index banque invalide: %d\n", bankIndex);
    return;
  }
  
  // Copier les presets et points d'interpolation actuels dans la banque
  for (int i = 0; i < NUM_PRESETS; i++) {
    banks[bankIndex].presets[i] = presets[i];
  }
  
  for (int i = 0; i < interpCount; i++) {
    banks[bankIndex].interpPoints[i] = interpPoints[i];
  }
  banks[bankIndex].interpCount = interpCount;
  
  Serial.printf("💾 Banque %d sauvegardée (%d presets, %d points interp)\n", 
                bankIndex, NUM_PRESETS, banks[bankIndex].interpCount);
}

void loadBank(uint8_t bankIndex) {
  if (bankIndex >= NUM_BANKS) {
    Serial.printf("❌ Index banque invalide: %d\n", bankIndex);
    return;
  }
  
  // Charger les presets depuis la banque
  for (int i = 0; i < NUM_PRESETS; i++) {
    presets[i] = banks[bankIndex].presets[i];
  }
  
  // Charger les points d'interpolation depuis la banque
  interpCount = banks[bankIndex].interpCount;
  for (int i = 0; i < interpCount; i++) {
    interpPoints[i] = banks[bankIndex].interpPoints[i];
  }
  
  activeBank = bankIndex;
  
  Serial.printf("📂 Banque %d chargée (%d presets, %d points interp) - Axe d'interpolation activé\n", 
                bankIndex, NUM_PRESETS, interpCount);
}

uint8_t getActiveBank() {
  return activeBank;
}

void setActiveBank(uint8_t bankIndex) {
  if (bankIndex < NUM_BANKS) {
    activeBank = bankIndex;
    Serial.printf("📁 Banque active: %d\n", bankIndex);
  }
}

void saveActiveBank() {
  saveBank(activeBank);
}

void loadActiveBank() {
  loadBank(activeBank);
}

//==================== Fonctions d'interpolation ====================
void computeInterpolatedPosition(float fraction, long &P, long &T, long &Z, long &S) {
  // Clamp la fraction
  fraction = clampF(fraction, 0.0f, 1.0f);
  
  // Cas limites: avant le premier point ou après le dernier
  if (fraction <= interpPoints[0].fraction) {
    uint8_t idx = interpPoints[0].presetIndex;
    P = presets[idx].p;  T = presets[idx].t;
    Z = presets[idx].z;  S = presets[idx].s;
    return;
  }
  
  if (fraction >= interpPoints[interpCount-1].fraction) {
    uint8_t idx = interpPoints[interpCount-1].presetIndex;
    P = presets[idx].p;  T = presets[idx].t;
    Z = presets[idx].z;  S = presets[idx].s;
    return;
  }
  
  // Trouver le segment d'interpolation
  int j = 0;
  while (j < interpCount-1 && interpPoints[j+1].fraction < fraction) { j++; }
  
  // Interpolation linéaire entre point j et j+1
  float u0 = interpPoints[j].fraction;
  float u1 = interpPoints[j+1].fraction;
  float alpha = (fraction - u0) / (u1 - u0);
  
  uint8_t presetA = interpPoints[j].presetIndex;
  uint8_t presetB = interpPoints[j+1].presetIndex;
  
  // Interpolation linéaire des axes
  P = lround( lerp((float)presets[presetA].p, (float)presets[presetB].p, alpha) );
  T = lround( lerp((float)presets[presetA].t, (float)presets[presetB].t, alpha) );
  Z = lround( lerp((float)presets[presetA].z, (float)presets[presetB].z, alpha) );
  S = lround( lerp((float)presets[presetA].s, (float)presets[presetB].s, alpha) );
}

void setInterpolationPoints(const InterpPoint* points, uint8_t count) {
  if (count > MAX_INTERP_POINTS) {
    count = MAX_INTERP_POINTS;
  }
  
  for (int i = 0; i < count; i++) {
    interpPoints[i] = points[i];
  }
  interpCount = count;
  
  Serial.printf("🎯 %d points d'interpolation définis\n", count);
  for (int i = 0; i < count; i++) {
    Serial.printf("   Point %d: Preset %d @ %.1f%%\n", 
                  i, interpPoints[i].presetIndex, interpPoints[i].fraction * 100.0f);
  }
}

uint8_t getInterpolationPoints(InterpPoint* points) {
  for (int i = 0; i < interpCount; i++) {
    points[i] = interpPoints[i];
  }
  return interpCount;
}

uint8_t getInterpolationCount() {
  return interpCount;
}

//==================== Fonctions de contrôle d'interpolation ====================
void startAutoInterpolation(uint32_t duration_ms) {
  interpAuto.active = true;
  interpAuto.t0_ms = millis();
  interpAuto.T_ms = duration_ms;
  interpAuto.dir = 1;
  
  Serial.printf("▶️ Interpolation auto ON (T=%u ms, %.1fs)\n", duration_ms, duration_ms / 1000.0f);
}

void stopAutoInterpolation() {
  interpAuto.active = false;
  Serial.println("⏹️ Interpolation auto OFF");
}

bool isAutoInterpolationActive() {
  return interpAuto.active;
}

void gotoInterpolationPosition(float fraction) {
  fraction = clampF(fraction, 0.0f, 1.0f);
  
  // Arrêter l'interpolation automatique
  interpAuto.active = false;
  
  // Calculer position interpolée
  long P, T, Z, S;
  computeInterpolatedPosition(fraction, P, T, Z, S);
  
  // Envoyer aux moteurs
  moveMotorTo(0, P);  // PAN
  moveMotorTo(1, T);  // TILT
  moveMotorTo(2, Z);  // ZOOM
  moveMotorTo(3, S);  // SLIDE
  
  Serial.printf("🎛️ Manual interp goto %.1f%% -> P=%ld T=%ld Z=%ld S=%ld\n", 
                fraction * 100.0f, P, T, Z, S);
}

void setInterpolationJogSpeed(float speed) {
  interp_jog_cmd = clampF(speed, -1.0f, 1.0f);
  Serial.printf("🎛️ Interp jog speed = %.2f\n", interp_jog_cmd);
}

void updateInterpolation() {
  uint32_t now = millis();
  
  // Mode interpolation automatique multi-presets (prioritaire)
  if (interpAuto.active) {
    float tau = (float)(now - interpAuto.t0_ms) / (float)interpAuto.T_ms;
    
    if (tau >= 1.0f) {
      // Fin du cycle, redémarrer
      interpAuto.t0_ms = now;
      interpAuto.dir = (interpAuto.dir > 0 ? -1 : +1);
      tau = 0.0f;
    }
    
    // Courbe s(tau) = tau^2 * (3 - 2*tau) (smoothstep)
    float s = tau * tau * (3.0f - 2.0f * tau);
    float u = (interpAuto.dir > 0 ? s : (1.0f - s));
    
    // Calculer la position interpolée pour tous les axes
    long P, T, Z, S;
    computeInterpolatedPosition(u, P, T, Z, S);
    
    // Envoyer aux moteurs
    moveMotorTo(0, P);  // PAN
    moveMotorTo(1, T);  // TILT
    moveMotorTo(2, Z);  // ZOOM
    moveMotorTo(3, S);  // SLIDE
    
    return;  // ignore les autres modes pendant l'interpolation auto
  }
  
  // Jog interpolation manuel (prioritaire)
  static float interp_fraction = 0.0f;  // fraction courante sur l'axe d'interpolation
  if (fabs(interp_jog_cmd) > 0.001f) {
    // Déterminer le segment [j, j+1] de la courbe correspondant à interp_fraction
    int j = 0;
    while (j < interpCount - 1 && interpPoints[j+1].fraction < interp_fraction) {
      j++;
    }
    
    float u0 = interpPoints[j].fraction;
    float u1 = interpPoints[j+1].fraction;
    if (u1 < u0) u1 = u0;  // sécurité (au cas où, mais interpPoints est trié)
    
    // Vitesse de déplacement sur l'axe d'interpolation (fraction/s)
    const float maxFracSpeed = 0.5f;  // 50% de l'axe par seconde max
    const float dt = 0.01f;  // 10ms (100Hz)
    
    uint8_t presetA = interpPoints[j].presetIndex;
    uint8_t presetB = interpPoints[j+1].presetIndex;
    long dP = presets[presetB].p - presets[presetA].p;
    long dT = presets[presetB].t - presets[presetA].t;
    long dZ = presets[presetB].z - presets[presetA].z;
    long dS = presets[presetB].s - presets[presetA].s;
    
    // Calculer la nouvelle fraction sur l'axe d'interpolation
    float du = interp_jog_cmd * maxFracSpeed * dt;
    interp_fraction = clampF(interp_fraction + du, 0.0f, 1.0f);
    
    // Calculer la nouvelle position interpolée et l'envoyer aux moteurs
    long P, T, Z, S;
    computeInterpolatedPosition(interp_fraction, P, T, Z, S);
    
    moveMotorTo(0, P);  // PAN
    moveMotorTo(1, T);  // TILT
    moveMotorTo(2, Z);  // ZOOM
    moveMotorTo(3, S);  // SLIDE
  }
}

//==================== Fonctions de sérialisation ====================
bool serializeBankToJSON(uint8_t bankIndex, String &jsonString) {
  if (bankIndex >= NUM_BANKS) {
    return false;
  }
  
  JsonDocument doc;
  
  // Sérialiser les presets
  JsonArray presetsArray = doc["presets"].to<JsonArray>();
  for (int i = 0; i < NUM_PRESETS; i++) {
    JsonObject preset = presetsArray.add<JsonObject>();
    preset["p"] = banks[bankIndex].presets[i].p;
    preset["t"] = banks[bankIndex].presets[i].t;
    preset["z"] = banks[bankIndex].presets[i].z;
    preset["s"] = banks[bankIndex].presets[i].s;
  }
  
  // Sérialiser l'interpolation
  JsonArray interpArray = doc["interp"].to<JsonArray>();
  for (int i = 0; i < banks[bankIndex].interpCount; i++) {
    JsonObject interp = interpArray.add<JsonObject>();
    interp["presetIndex"] = banks[bankIndex].interpPoints[i].presetIndex;
    interp["fraction"] = banks[bankIndex].interpPoints[i].fraction * 100.0f; // Convertir en pourcentage
  }
  doc["interpCount"] = banks[bankIndex].interpCount;
  
  jsonString = "";
  serializeJson(doc, jsonString);
  
  return true;
}

bool deserializeBankFromJSON(uint8_t bankIndex, const String &jsonString) {
  if (bankIndex >= NUM_BANKS) {
    return false;
  }
  
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, jsonString);
  
  if (error) {
    Serial.printf("❌ Erreur JSON: %s\n", error.c_str());
    return false;
  }
  
  // Charger les presets
  if (doc.containsKey("presets")) {
    JsonArray presetsArray = doc["presets"];
    for (int i = 0; i < NUM_PRESETS && i < presetsArray.size(); i++) {
      JsonObject preset = presetsArray[i];
      banks[bankIndex].presets[i].p = preset["p"];
      banks[bankIndex].presets[i].t = preset["t"];
      banks[bankIndex].presets[i].z = preset["z"];
      banks[bankIndex].presets[i].s = preset["s"];
    }
  }
  
  // Charger l'interpolation
  if (doc.containsKey("interp")) {
    JsonArray interpArray = doc["interp"];
    banks[bankIndex].interpCount = min((int)interpArray.size(), MAX_INTERP_POINTS);
    for (int i = 0; i < banks[bankIndex].interpCount; i++) {
      JsonObject interp = interpArray[i];
      banks[bankIndex].interpPoints[i].presetIndex = interp["presetIndex"];
      banks[bankIndex].interpPoints[i].fraction = interp["fraction"].as<float>() / 100.0f; // Convertir depuis pourcentage
    }
  }
  
  // Si c'est la banque active, mettre à jour les variables globales
  if (bankIndex == activeBank) {
    for (int i = 0; i < NUM_PRESETS; i++) {
      presets[i] = banks[bankIndex].presets[i];
    }
    
    interpCount = banks[bankIndex].interpCount;
    for (int i = 0; i < interpCount; i++) {
      interpPoints[i] = banks[bankIndex].interpPoints[i];
    }
  }
  
  return true;
}
