/**
 * @file Presets.cpp
 * @brief Implémentation du module de gestion des presets et interpolation
 * @author Laurent Eyen
 * @date 2024
 */

#include "Presets.h"
#include "MotorControl.h"
#include "Utils.h"
#include "Joystick.h"

// Déclaration externe pour joy_filt
extern JoyState joy_filt;

//==================== Variables globales ====================
Preset presets[MAX_PRESETS];
int activePreset = -1;
InterpPoint interpPoints[MAX_INTERP_POINTS];
uint8_t interpCount = 2;
Bank banks[MAX_BANKS];
uint8_t activeBank = 0;
Preferences nvs;
InterpAuto interpAuto;
float interp_jog_cmd = 0.0f;

// Offsets persistants pendant l'interpolation (auto ou jog)
long interp_offset_p = 0;
long interp_offset_t = 0;
long interp_offset_z = 0;
long interp_offset_s = 0;

//==================== Fonctions du module ====================

/**
 * @brief Initialise le système de presets
 */
void initPresets() {
  Serial.println("🎯 Initialisation du système de presets...");
  
  // Initialiser les presets avec des valeurs par défaut
  for (int i = 0; i < MAX_PRESETS; i++) {
    presets[i] = {0, 0, 0, 0};
  }
  
  // Initialiser les points d'interpolation par défaut
  interpPoints[0] = {0, 0.0f};
  interpPoints[1] = {1, 1.0f};
  interpCount = 2;
  
  // Initialiser les banques
  for (int i = 0; i < MAX_BANKS; i++) {
    for (int j = 0; j < MAX_PRESETS; j++) {
      banks[i].presets[j] = {0, 0, 0, 0};
    }
    banks[i].interpPoints[0] = {0, 0.0f};
    banks[i].interpPoints[1] = {1, 1.0f};
    banks[i].interpCount = 2;
  }
  
  Serial.println("✅ Système de presets initialisé");
}

/**
 * @brief Définit un preset avec les positions actuelles
 */
void setPreset(uint8_t index) {
  if (index >= MAX_PRESETS) return;
  
  presets[index].p = panPos;
  presets[index].t = tiltPos;
  presets[index].z = zoomPos;
  presets[index].s = slidePos;
  
  Serial.printf("💾 Preset %d défini: P=%ld, T=%ld, Z=%ld, S=%ld\n", 
                index, presets[index].p, presets[index].t, presets[index].z, presets[index].s);
}

/**
 * @brief Obtient un preset
 */
Preset& getPreset(uint8_t index) {
  static Preset empty = {0, 0, 0, 0};
  if (index >= MAX_PRESETS) return empty;
  return presets[index];
}

/**
 * @brief Sauvegarde un preset dans la banque active
 */
void savePreset(uint8_t index) {
  if (index >= MAX_PRESETS) return;
  
  banks[activeBank].presets[index] = presets[index];
  Serial.printf("💾 Preset %d sauvegardé dans la banque %d\n", index, activeBank);
}

/**
 * @brief Charge un preset depuis la banque active
 */
void loadPreset(uint8_t index) {
  if (index >= MAX_PRESETS) return;
  
  presets[index] = banks[activeBank].presets[index];
  activePreset = index;
  
  // Appliquer les positions aux moteurs
  steppers[0]->moveTo(presets[index].p);
  steppers[1]->moveTo(presets[index].t);
  steppers[2]->moveTo(presets[index].z);
  steppers[3]->moveTo(presets[index].s);
  
  Serial.printf("📂 Preset %d chargé: P=%ld, T=%ld, Z=%ld, S=%ld\n", 
                index, presets[index].p, presets[index].t, presets[index].z, presets[index].s);
}

/**
 * @brief Sauvegarde une banque complète
 */
void saveBank(uint8_t idx) {
  if (idx >= MAX_BANKS) return;
  
  // Copier les variables globales vers la structure Bank
  for (int i = 0; i < MAX_PRESETS; i++) {
    banks[idx].presets[i] = presets[i];
  }
  for (int i = 0; i < MAX_INTERP_POINTS; i++) {
    banks[idx].interpPoints[i] = interpPoints[i];
  }
  banks[idx].interpCount = interpCount;
  
  // Initialiser NVS
  if (!nvs.begin("banks")) {
    Serial.println("❌ Erreur NVS");
    return;
  }
  
  // Créer le JSON
  DynamicJsonDocument doc(4096);
  JsonArray presetsArray = doc.createNestedArray("presets");
  JsonArray interpArray = doc.createNestedArray("interp");
  
  // Sérialiser les presets
  for (int i = 0; i < MAX_PRESETS; i++) {
    JsonObject preset = presetsArray.createNestedObject();
    preset["p"] = banks[idx].presets[i].p;
    preset["t"] = banks[idx].presets[i].t;
    preset["z"] = banks[idx].presets[i].z;
    preset["s"] = banks[idx].presets[i].s;
  }
  
  // Sérialiser l'interpolation
  for (int i = 0; i < banks[idx].interpCount; i++) {
    JsonObject interp = interpArray.createNestedObject();
    interp["presetIndex"] = banks[idx].interpPoints[i].presetIndex;
    interp["fraction"] = banks[idx].interpPoints[i].fraction * 100.0f; // Convertir en pourcentage
  }
  doc["interpCount"] = banks[idx].interpCount;
  
  // Sérialiser en string
  String jsonString;
  serializeJson(doc, jsonString);
  
  // Sauvegarder dans NVS
  String key = "bank_" + String(idx);
  nvs.putString(key.c_str(), jsonString);
  nvs.end();
  
  Serial.printf("💾 Banque %d sauvegardée (%d presets, %d points interp)\n", idx, MAX_PRESETS, banks[idx].interpCount);
}

/**
 * @brief Charge une banque complète
 */
void loadBank(uint8_t idx) {
  if (idx >= MAX_BANKS) return;
  
  // Initialiser NVS
  if (!nvs.begin("banks")) {
    Serial.println("❌ Erreur NVS");
    return;
  }
  
  String key = "bank_" + String(idx);
  String jsonString = nvs.getString(key.c_str());
  nvs.end();
  
  if (jsonString.length() == 0) {
    Serial.printf("⚠️ Banque %d vide, initialisation avec valeurs par défaut\n", idx);
    
    // Initialiser avec des valeurs par défaut
    for (int i = 0; i < MAX_PRESETS; i++) {
      banks[idx].presets[i] = {0, 0, 0, 0};
      presets[i] = banks[idx].presets[i];
    }
    banks[idx].interpPoints[0] = {0, 0.0f};
    banks[idx].interpPoints[1] = {1, 1.0f};
    banks[idx].interpCount = 2;
    interpPoints[0] = banks[idx].interpPoints[0];
    interpPoints[1] = banks[idx].interpPoints[1];
    interpCount = banks[idx].interpCount;
    return;
  }
  
  // Parser le JSON
  DynamicJsonDocument doc(4096);
  DeserializationError error = deserializeJson(doc, jsonString);
  if (error) {
    Serial.printf("❌ Erreur parsing JSON banque %d: %s\n", idx, error.c_str());
    return;
  }
  
  // Charger les presets dans la structure Bank ET les variables globales
  JsonArray presetsArray = doc["presets"];
  for (int i = 0; i < MAX_PRESETS && i < presetsArray.size(); i++) {
    JsonObject preset = presetsArray[i];
    banks[idx].presets[i].p = preset["p"];
    banks[idx].presets[i].t = preset["t"];
    banks[idx].presets[i].z = preset["z"];
    banks[idx].presets[i].s = preset["s"];
    
    // Copier vers les variables globales
    presets[i] = banks[idx].presets[i];
  }
  
  // Charger l'interpolation dans la structure Bank ET les variables globales
  JsonArray interpArray = doc["interp"];
  banks[idx].interpCount = min((int)interpArray.size(), MAX_INTERP_POINTS);
  for (int i = 0; i < banks[idx].interpCount; i++) {
    JsonObject interp = interpArray[i];
    banks[idx].interpPoints[i].presetIndex = interp["presetIndex"];
    banks[idx].interpPoints[i].fraction = interp["fraction"].as<float>() / 100.0f; // Convertir depuis pourcentage
    
    // Copier vers les variables globales
    interpPoints[i] = banks[idx].interpPoints[i];
  }
  interpCount = banks[idx].interpCount;
  
  Serial.printf("📂 Banque %d chargée (%d presets, %d points interp) - Axe d'interpolation activé\n", idx, MAX_PRESETS, interpCount);
}

/**
 * @brief Met à jour l'interpolation automatique
 */
void updateInterpolation() {
  uint32_t now = millis();
  
  // Mode interpolation automatique multi-presets (prioritaire)
  if (interpAuto.active) {
    float tau = (float)(now - interpAuto.t0_ms) / (float)interpAuto.T_ms;
    if (tau >= 1.0f) {
      tau = 0.0f;
      interpAuto.t0_ms = now;
      // Inversion de direction pour boucle continue
      interpAuto.dir = (interpAuto.dir > 0 ? -1 : +1);
    }
    
    // Profil minimum-jerk pour un mouvement fluide aller/retour
    float s = s_minjerk(tau);
    float u = (interpAuto.dir > 0 ? s : (1.0f - s));
    
    // Calculer la position interpolée pour tous les axes
    long P, T, Z, S;
    computeInterpolatedPosition(u, P, T, Z, S);
    
    // Ajouter les offsets persistants + offsets joystick instantanés
    P += interp_offset_p + getEffectivePanOffset(true);
    T += interp_offset_t + getEffectiveTiltOffset(true);
    Z += interp_offset_z + getEffectiveZoomOffset(true);
    S += interp_offset_s + getEffectiveSlideOffset(true);
    
    // Clamp limites
    P = clampL(P, cfg[0].min_limit, cfg[0].max_limit);
    T = clampL(T, cfg[1].min_limit, cfg[1].max_limit);
    Z = clampL(Z, cfg[2].min_limit, cfg[2].max_limit);
    S = clampL(S, cfg[3].min_limit, cfg[3].max_limit);
    
    // Envoyer aux moteurs
    steppers[0]->moveTo(P);
    steppers[1]->moveTo(T);
    steppers[2]->moveTo(Z);
    steppers[3]->moveTo(S);
    
    return;  // ignore les autres modes pendant l'interpolation auto
  }
}

/**
 * @brief Calcule une position interpolée
 */
void computeInterpolatedPosition(float u, long &P, long &T, long &Z, long &S) {
  // Bornes : si en-dessous du premier point ou au-delà du dernier
  if (u <= interpPoints[0].fraction) {
    uint8_t idx = interpPoints[0].presetIndex;
    P = presets[idx].p;  T = presets[idx].t;
    Z = presets[idx].z;  S = presets[idx].s;
    return;
  }
  if (u >= interpPoints[interpCount-1].fraction) {
    uint8_t idx = interpPoints[interpCount-1].presetIndex;
    P = presets[idx].p;  T = presets[idx].t;
    Z = presets[idx].z;  S = presets[idx].s;
    return;
  }
  
  // Trouver le segment [j, j+1] contenant u
  uint8_t j = 0;
  while (j < interpCount-1 && interpPoints[j+1].fraction < u) { j++; }
  
  // Interpolation linéaire entre point j et j+1
  float u0 = interpPoints[j].fraction;
  float u1 = interpPoints[j+1].fraction;
  float alpha = (u - u0) / (u1 - u0);  // fraction relative entre ces deux points
  
  uint8_t presetA = interpPoints[j].presetIndex;
  uint8_t presetB = interpPoints[j+1].presetIndex;
  
  // Interpolation linéaire des axes
  P = lround( lerp((float)presets[presetA].p, (float)presets[presetB].p, alpha) );
  T = lround( lerp((float)presets[presetA].t, (float)presets[presetB].t, alpha) );
  Z = lround( lerp((float)presets[presetA].z, (float)presets[presetB].z, alpha) );
  S = lround( lerp((float)presets[presetA].s, (float)presets[presetB].s, alpha) );
}

/**
 * @brief Démarre l'interpolation automatique
 */
void startInterpolation(float duration) {
  if (duration > 0.0f) {
    // Réinitialiser les offsets persistants
    interp_offset_p = 0;
    interp_offset_t = 0;
    interp_offset_z = 0;
    interp_offset_s = 0;
    
    uint32_t T_ms = (uint32_t)lround(duration * 1000);
    interpAuto.T_ms = T_ms;
    interpAuto.t0_ms = millis();
    interpAuto.active = true;
    interpAuto.dir = +1;
    
    Serial.printf("▶️ Interpolation auto ON (T=%u ms, %.1fs)\n", T_ms, duration);
  } else {
    interpAuto.active = false;
    Serial.println("⏹️ Interpolation auto OFF");
  }
}

/**
 * @brief Arrête l'interpolation automatique
 */
void stopInterpolation() {
  interpAuto.active = false;
  Serial.println("⏹️ Interpolation auto OFF");
}

/**
 * @brief Vérifie si l'interpolation est active
 */
bool isInterpolationActive() {
  return interpAuto.active;
}

/**
 * @brief Définit la commande de jog d'interpolation
 */
void setInterpJogCommand(float cmd) {
  interp_jog_cmd = cmd;
}

/**
 * @brief Met à jour l'interpolation manuelle (jog)
 * @details Intègre interp_jog_cmd comme vitesse sur l'axe d'interpolation
 */
void updateInterpolationJog() {
    static float u = 0.0f;
    static uint32_t last = millis();
    static bool baselineSaved = false;
    uint32_t now = millis();
    float dt = (now - last) / 1000.0f;
    last = now;

    // Intégrer la vitesse jog sur [0,1]
    u += interp_jog_cmd * dt * 0.2f; // 0.2 = vitesse max en fraction/s (ajuster)
    if (u < 0) u = 0; if (u > 1) u = 1;

    long P, T, Z, S;
    computeInterpolatedPosition(u, P, T, Z, S);
    
    // Ajouter les offsets persistants + offsets joystick instantanés
    P += interp_offset_p + getEffectivePanOffset(true);
    T += interp_offset_t + getEffectiveTiltOffset(true);
    Z += interp_offset_z + getEffectiveZoomOffset(true);
    S += interp_offset_s + getEffectiveSlideOffset(true);
    
    // Clamp limites
    P = clampL(P, cfg[0].min_limit, cfg[0].max_limit);
    T = clampL(T, cfg[1].min_limit, cfg[1].max_limit);
    Z = clampL(Z, cfg[2].min_limit, cfg[2].max_limit);
    S = clampL(S, cfg[3].min_limit, cfg[3].max_limit);
    
    // Gestion du baseline
    if (!baselineSaved && fabs(interp_jog_cmd) > 0.001f) {
        saveOffsetBaseline();
        baselineSaved = true;
    }
    if (fabs(interp_jog_cmd) < 0.001f) baselineSaved = false;
    
    // Mise à jour des offsets persistants pendant que le joystick bouge
    if (fabsf(joy_filt.pan) > 0.01f || fabsf(joy_filt.tilt) > 0.01f ||
        fabsf(joy_filt.zoom) > 0.01f || fabsf(joy_filt.slide) > 0.01f) {
        interp_offset_p = pan_offset_latched;
        interp_offset_t = tilt_offset_latched;
        interp_offset_z = zoom_offset_latched;
        interp_offset_s = slide_offset_latched;
    }
    
    steppers[0]->moveTo(P);
    steppers[1]->moveTo(T);
    steppers[2]->moveTo(Z);
    steppers[3]->moveTo(S);
}

/**
 * @brief Calcule la durée optimale pour un mouvement
 */
uint32_t pickDurationMsForDeltas(long deltaP, long deltaT, long deltaZ, long deltaS) {
  // Calculer les distances en pas
  float distP = abs(deltaP);
  float distT = abs(deltaT);
  float distZ = abs(deltaZ);
  float distS = abs(deltaS);
  
  // Vitesses maximales (steps/s) - à ajuster selon la config
  float maxSpeedP = 20000.0f;
  float maxSpeedT = 20000.0f;
  float maxSpeedZ = 20000.0f;
  float maxSpeedS = 10000.0f;
  
  // Calculer les durées pour chaque axe
  float timeP = (distP > 0) ? (distP / maxSpeedP) : 0.0f;
  float timeT = (distT > 0) ? (distT / maxSpeedT) : 0.0f;
  float timeZ = (distZ > 0) ? (distZ / maxSpeedZ) : 0.0f;
  float timeS = (distS > 0) ? (distS / maxSpeedS) : 0.0f;
  
  // Prendre la durée maximale
  float maxTime = max(max(timeP, timeT), max(timeZ, timeS));
  
  // Convertir en millisecondes avec un minimum de 500ms
  uint32_t duration = max(500u, (uint32_t)lround(maxTime * 1000.0f));
  
  return duration;
}
