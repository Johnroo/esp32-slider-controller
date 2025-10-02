#include <Arduino.h>
#include <WiFi.h>
#include <FastAccelStepper.h>
#include <TMCStepper.h>
#include <WiFiManager.h>
#include <ArduinoOTA.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include <OSCBundle.h>
#include <OSCData.h>
#include <math.h>
#include <ArduinoJson.h>
#include <Preferences.h>

//==================== Configuration ====================
#define NUM_MOTORS 4

//==================== NEW: Presets, offsets, mapping ====================
enum PresetMode : uint8_t { PRESET_ABSOLUTE = 0, PRESET_FOLLOW_SLIDE = 1 };

struct Preset {
  long p, t, z, s;          // positions absolues
  long pan_anchor;          // ancre P relative au slide
  long tilt_anchor;         // ancre T relative au slide
  uint8_t mode;             // PRESET_ABSOLUTE ou PRESET_FOLLOW_SLIDE
};

Preset presets[8];         // utilitaires: /preset/set i p t z s
int activePreset = -1;

// Offsets joystick (en steps)
volatile long pan_offset_steps  = 0;
volatile long tilt_offset_steps = 0;

// Offsets joystick latched (s'accumulent, persistent)
volatile long pan_offset_latched  = 0;
volatile long tilt_offset_latched = 0;

long PAN_OFFSET_RANGE  = 800;   // max offset +/- (configurable via OSC)
long TILT_OFFSET_RANGE = 800;

// Baseline d'offset au recall (Δoffset = latched - baseline)
struct OffsetSession {
  long pan0 = 0;
  long tilt0 = 0;
} offset_session;

const long OFFSET_DEADBAND_STEPS = 2; // anti-bruit
static inline long odband(long v){ return (labs(v) <= OFFSET_DEADBAND_STEPS) ? 0 : v; }

static inline long eff_pan_offset()  { return odband(pan_offset_latched  - offset_session.pan0); }
static inline long eff_tilt_offset() { return odband(tilt_offset_latched - offset_session.tilt0); }

static inline long active_pan_offset(bool recall_phase){ 
  return recall_phase ? eff_pan_offset() : pan_offset_steps; 
}
static inline long active_tilt_offset(bool recall_phase){ 
  return recall_phase ? eff_tilt_offset() : tilt_offset_steps; 
}

// Mapping linéaire slide -> compensation pan/tilt (en steps)
long PAN_AT_SLIDE_MIN  = +800;  // ex: +800 à gauche
long PAN_AT_SLIDE_MAX  = -800;  // ex: -800 à droite
long TILT_AT_SLIDE_MIN = 0;
long TILT_AT_SLIDE_MAX = 0;

// Mouvement synchronisé en cours
struct SyncMove {
  bool  active = false;
  uint32_t t0_ms = 0;
  uint32_t T_ms  = 2000;      // durée demandée
  long start[NUM_MOTORS];
  long goal_base[NUM_MOTORS]; // cible sans offsets/couplages
} sync_move;

// Politique d'annulation des presets
struct CancelPolicy { 
  bool by_joystick = false;  // joystick n'annule pas les presets (offsets actifs)
  bool by_axis = true;      // Direct Axis annule les presets (contrôle direct)
} cancel;

// Couplage Pan/Tilt ↔ Slide pendant le jog
struct Follow {
  bool enabled = true;
  bool valid   = false;
  long pan_anchor  = 0;
  long tilt_anchor = 0;
} follow;

// Interpolation d'ancres pendant un recall autour de l'autopan
struct AnchorMorph {
  bool active = false;
  long p0 = 0, t0 = 0;
  long p1 = 0, t1 = 0;
  uint32_t t0_ms = 0, T_ms = 0;
} anchor_morph;

// Politique de recall du slide
enum class SlideRecallPolicy : uint8_t { KEEP_AB = 0, GOTO_THEN_RESUME = 1 };
struct RecallPolicy {
  SlideRecallPolicy slide = SlideRecallPolicy::KEEP_AB;
} recallPolicy;

//==================== Homing Slide (StallGuard) ====================
// Permettre un homing sans capteur sur l'axe slide via StallGuard (TMC2209)
bool    doAutoHomeSlide     = true;   // lancer automatiquement au démarrage si true
uint8_t slide_sg_threshold  = 100;    // SGTHRS par défaut (sensibilité moyenne)

// Mode AB infini pour le slide (déprécié - remplacé par interpolation multi-presets)
struct SlideAB {
  bool enabled = false;
  long A = 0, B = 0;       // steps
  uint32_t T_ms = 4000;    // durée d'un aller
  uint32_t t0_ms = 0;
  int dir = +1;            // +1: A->B, -1: B->A
} slideAB;

// Axe d'interpolation multi-presets (nouveau système)
struct InterpPoint { 
  uint8_t presetIndex; 
  float fraction; 
};

InterpPoint interpPoints[6];  // jusqu'à 6 points (A, B, C, D, E, F)
uint8_t interpCount = 2;      // par défaut 2 points (A@0%, B@100%)

//==================== Bank System ====================
struct Bank {
  Preset presets[8];
  InterpPoint interpPoints[6];
  uint8_t interpCount;
};

Bank banks[10];              // 10 banques de presets
uint8_t activeBank = 0;      // banque active (0-9)
Preferences nvs;             // pour la persistance NVS

struct InterpAuto {
  bool active = false;
  uint32_t t0_ms = 0;
  uint32_t T_ms = 5000;   // durée du parcours 0->100%
  int dir = +1;           // sens courant (+1 = 0→100%, -1 = 100%→0%)
} interpAuto;

// Jog slide
float slide_jog_cmd = 0.0f;    // -1..+1
float SLIDE_JOG_SPEED = 6000;  // steps/s @ |cmd|=1 (à ajuster)

// Jog interpolation manuel
float interp_jog_cmd = 0.0f;    // -1..+1 (vitesse normalisée)

// Jog Pan/Tilt (vitesses de jog) - dérivées de la config
float PAN_JOG_SPEED  = 3000.0f; // steps/s @ |joy|=1 (sera calculé dans setup)
float TILT_JOG_SPEED = 3000.0f; // steps/s @ |joy|=1 (sera calculé dans setup)

// Pins STEP/DIR/EN
const int STEP_PINS[NUM_MOTORS]    = {18, 21, 23, 26};
const int DIR_PINS[NUM_MOTORS]     = {19, 22, 25, 27};
const int ENABLE_PINS[NUM_MOTORS]  = {13, 14, 32, 33};

// UART TMC2209
#define UART_TX 17
#define UART_RX 16
#define ADDR_PAN   0b00
#define ADDR_TILT  0b01
#define ADDR_ZOOM  0b10
#define ADDR_SLIDE 0b11
#define R_SENSE 0.11f

// Configuration des axes
struct AxisConfig {
  long min_limit;
  long max_limit;
  int  current_ma;
  int  microsteps;
  int  max_speed;
  int  max_accel;
  int  sgt;
  bool coolstep;
  bool spreadcycle;
  bool stallguard;
};

AxisConfig cfg[NUM_MOTORS] = {
  // Pan
  {-27106, 27106, 1200, 16, 20000, 12000, 0, false, false, true},
  // Tilt  
  {-2439, 2439, 1200, 16, 20000, 12000, 0, false, false, true},
  // Zoom
  {-20000, 20000, 400, 16, 20000, 8000, 0, false, false, true},
  // Slide
  {-20000, 20000, 1800, 8, 10000, 12000, 0, false, false, true}
};

//==================== Objets moteurs ====================
FastAccelStepperEngine engine;
FastAccelStepper* steppers[NUM_MOTORS];

TMC2209Stepper driver_pan  (&Serial2, R_SENSE, ADDR_PAN);
TMC2209Stepper driver_tilt (&Serial2, R_SENSE, ADDR_TILT);
TMC2209Stepper driver_zoom (&Serial2, R_SENSE, ADDR_ZOOM);
TMC2209Stepper driver_slide(&Serial2, R_SENSE, ADDR_SLIDE);
TMC2209Stepper* drivers[NUM_MOTORS] = {&driver_pan,&driver_tilt,&driver_zoom,&driver_slide};

//==================== Setup Drivers TMC ====================
void setupDriversTMC() {
  Serial2.begin(115200, SERIAL_8N1, UART_RX, UART_TX);
  delay(50);

  for (int i=0; i<NUM_MOTORS; i++) {
    auto d = drivers[i];
    d->begin();
    d->toff(5);                                // enable driver
    d->rms_current(cfg[i].current_ma);         // courant RMS
    d->microsteps(cfg[i].microsteps);          // µsteps
    d->pwm_autoscale(true);                    // pour StealthChop
    d->en_spreadCycle(cfg[i].spreadcycle);
    d->SGTHRS(cfg[i].sgt);                     // StallGuard threshold
    // d->coolstep_en(cfg[i].coolstep);  // Pas disponible sur TMC2209
    // d->stallguard(cfg[i].stallguard);  // Pas disponible sur TMC2209
  }
}

// Homing du slide via StallGuard4 (TMC2209)
// StallGuard4 fonctionne en StealthChop (spreadCycle = false)

#define SLIDE_INDEX     3
#define HOMING_SPEED    9000     // steps/s (2000-4000 range pour SG4)
#define HOMING_ACCEL    90000    // accel élevée pour atteindre vitesse rapidement
// SG_THRESHOLD remplacé par slide_sg_threshold (variable globale)
#define SG_DETECT       100      // seuil SG_RESULT pour détecter stall (commencer à ~100)
#define HOMING_TIMEOUT  20000    // ms
#define BACKOFF_STEPS   300      // pas de recul après détection

void home_slide() {
  int i = SLIDE_INDEX;
  if (!steppers[i]) return;

  Serial.println("🏠 HOMING SLIDE START (StallGuard4)");

  // Désactiver tous les modes automatiques
  slideAB.enabled = false;
  sync_move.active = false;
  follow.enabled = false;

  // Sauvegarder la configuration actuelle
  long original_current = cfg[i].current_ma;
  long original_accel = cfg[i].max_accel;
  long original_speed = cfg[i].max_speed;

  // Augmenter temporairement le courant (+20%)
  long homing_current = original_current + (original_current * 20 / 100);
  drivers[i]->rms_current(homing_current);
  Serial.printf("⚡ Courant homing: %ld mA (+20%%)\n", homing_current);

  // Configuration pour StallGuard4 (StealthChop requis)
  drivers[i]->en_spreadCycle(false);  // StealthChop pour StallGuard4
  drivers[i]->TPWMTHRS(0xFFFFF);      // Garder StealthChop à haute vitesse
  drivers[i]->SGTHRS(slide_sg_threshold);   // Sensibilité StallGuard
  drivers[i]->TCOOLTHRS(0xFFFFF);     // Activer SG même à basse vitesse

  // Configuration vitesse/accel pour homing
  steppers[i]->setAcceleration(HOMING_ACCEL);
  steppers[i]->setSpeedInHz(HOMING_SPEED);

  Serial.printf("🎯 Config: Speed=%d, Accel=%d, SGTHRS=%d, SG_DETECT=%d\n", 
                 HOMING_SPEED, HOMING_ACCEL, slide_sg_threshold, SG_DETECT);

  // ------------------ PHASE INF (vers butée inférieure) ------------------
  Serial.println("▶️ Vers butée INF...");
  steppers[i]->runBackward();
  uint32_t t0 = millis();
  uint32_t last_sg_log = 0;
  bool stall_detected = false;
  
  while (millis() - t0 < HOMING_TIMEOUT && !stall_detected) {
    uint16_t sg = drivers[i]->SG_RESULT();
    uint32_t now = millis();
    
    // Log SG_RESULT toutes les 300ms pour debug
    if (now - last_sg_log > 300) {
      Serial.printf("SG_RESULT: %u (seuil: %u)\n", sg, SG_DETECT);
      last_sg_log = now;
    }
    
    // Détecter stall après 500ms de mouvement
    if (millis() - t0 > 500 && sg < SG_DETECT) {
      Serial.printf("💥 INF Stall détecté (SG=%u < %u)\n", sg, SG_DETECT);
      steppers[i]->forceStop();
      stall_detected = true;
    }
    delay(2);
  }
  
  if (!stall_detected) {
    Serial.println("⚠️ Timeout INF - pas de stall détecté");
  }
  
  long minPos = steppers[i]->getCurrentPosition();
  Serial.printf("📍 Position INF: %ld\n", minPos);

  // Recul pour se dégager de la butée
  Serial.printf("↩️ Recul de %d pas...\n", BACKOFF_STEPS);
  steppers[i]->move(BACKOFF_STEPS);
  while (steppers[i]->isRunning()) delay(2);

  // ------------------ PHASE SUP (vers butée supérieure) ------------------
  Serial.println("▶️ Vers butée SUP...");
  steppers[i]->runForward();
  t0 = millis();
  last_sg_log = 0;
  stall_detected = false;
  
  while (millis() - t0 < HOMING_TIMEOUT && !stall_detected) {
    uint16_t sg = drivers[i]->SG_RESULT();
    uint32_t now = millis();
    
    // Log SG_RESULT toutes les 300ms pour debug
    if (now - last_sg_log > 300) {
      Serial.printf("SG_RESULT: %u (seuil: %u)\n", sg, SG_DETECT);
      last_sg_log = now;
    }
    
    // Détecter stall après 500ms de mouvement
    if (millis() - t0 > 500 && sg < SG_DETECT) {
      Serial.printf("💥 SUP Stall détecté (SG=%u < %u)\n", sg, SG_DETECT);
      steppers[i]->forceStop();
      stall_detected = true;
    }
    delay(2);
  }
  
  if (!stall_detected) {
    Serial.println("⚠️ Timeout SUP - pas de stall détecté");
  }
  
  long maxPos = steppers[i]->getCurrentPosition();
  Serial.printf("📍 Position SUP: %ld\n", maxPos);

  // ------------------ CALCUL CENTRE ET LIMITES ------------------
  long center = (minPos + maxPos) / 2;
  long range = maxPos - minPos;
  
  Serial.printf("📏 Course détectée: %ld pas (%.1f mm)\n", range, range * 0.1); // Assumant 0.1mm/pas
  
  // Aller au centre
  Serial.printf("🎯 Aller au centre: %ld\n", center);
  steppers[i]->moveTo(center);
  while (steppers[i]->isRunning()) delay(2);
  
  // Définir position 0 au centre
  steppers[i]->setCurrentPosition(0);
  
  // Calculer limites relatives au centre
  cfg[i].min_limit = minPos - center;
  cfg[i].max_limit = maxPos - center;
  
  Serial.printf("✅ Homing terminé!\n");
  Serial.printf("   Centre: %ld (position 0)\n", center);
  Serial.printf("   Limites: [%ld, %ld]\n", cfg[i].min_limit, cfg[i].max_limit);
  Serial.printf("   Course: %ld pas\n", cfg[i].max_limit - cfg[i].min_limit);

  // ------------------ RESTAURATION CONFIG NORMALE ------------------
  Serial.println("🔄 Restauration configuration normale...");
  
  // Restaurer courant normal
  drivers[i]->rms_current(original_current);
  
  // Restaurer vitesse/accel normales
  steppers[i]->setAcceleration(original_accel);
  steppers[i]->setSpeedInHz(original_speed);
  
  // Le driver reste en StealthChop (spreadCycle = false) comme partout ailleurs
  // Pas besoin de restaurer le mode car on utilise StealthChop partout
  
  Serial.println("✅ Homing terminé avec succès!");
}


//==================== NEW: Helpers ====================
static inline long clampL(long v, long vmin, long vmax){ return v < vmin ? vmin : (v > vmax ? vmax : v); }
static inline float clampF(float v, float vmin, float vmax){ return v < vmin ? vmin : (v > vmax ? vmax : v); }
static inline float lerp(float a, float b, float u){ return a + (b - a) * u; }

// Minimum-jerk s(t) = 10τ^3 - 15τ^4 + 6τ^5 avec τ = t/T
// ds/dt max = 1.875/T ; d2s/dt2 max ≈ 5.7735/T^2
static inline float s_minjerk(float tau){
  tau = clampF(tau, 0.0f, 1.0f);
  return 10*tau*tau*tau - 15*tau*tau*tau*tau + 6*tau*tau*tau*tau*tau;
}

//==================== NEW: Joystick Pipeline ====================
struct JoyCfg { 
  float deadzone=0.06f, expo=0.35f, slew_per_s=8000.0f, filt_hz=60.0f, 
        pan_tilt_speed=1.0f, slide_speed=1.0f; 
} joy;

struct JoyState { 
  float pan=0, tilt=0, slide=0; 
};

volatile JoyState joy_raw;  // alimenté par l'OSC
static   JoyState joy_cmd, joy_filt;

static inline float apply_deadzone_expo(float x, float dz, float expo){
  x = clampF(x, -1.f, 1.f); 
  if (fabsf(x) <= dz) return 0.f;
  float s = x >= 0 ? 1.f : -1.f, u = (fabsf(x) - dz) / (1.f - dz);
  return s * ((1-expo) * u + expo * u * u * u);
}

static inline float iir_1pole(float y, float x, float f, float dt){ 
  if(f <= 0) return x; 
  float a = 1.f - expf(-2.f * 3.1415926f * f * dt); 
  return y + a * (x - y); 
}

static inline float slew_limit(float y, float x, float slew, float dt){
  if (slew <= 0) return x; 
  float d = x - y, m = slew * dt;
  if (d > m) d = m; 
  if (d < -m) d = -m; 
  return y + d;
}

void joystick_tick(){
  static uint32_t t0 = millis(); 
  uint32_t now = millis(); 
  float dt = (now - t0) * 0.001f; 
  if(dt <= 0) return; 
  t0 = now;
  
  joy_cmd.pan   = apply_deadzone_expo(joy_raw.pan,  joy.deadzone, joy.expo);
  joy_cmd.tilt  = apply_deadzone_expo(joy_raw.tilt, joy.deadzone, joy.expo);
  joy_cmd.slide = apply_deadzone_expo(joy_raw.slide, joy.deadzone, joy.expo);

  joy_filt.pan   = slew_limit(joy_filt.pan,   iir_1pole(joy_filt.pan,   joy_cmd.pan,   joy.filt_hz, dt), joy.slew_per_s/(float)PAN_OFFSET_RANGE, dt);
  joy_filt.tilt  = slew_limit(joy_filt.tilt,  iir_1pole(joy_filt.tilt,  joy_cmd.tilt,  joy.filt_hz, dt), joy.slew_per_s/(float)TILT_OFFSET_RANGE, dt);
  joy_filt.slide = slew_limit(joy_filt.slide, iir_1pole(joy_filt.slide, joy_cmd.slide, joy.filt_hz, dt), joy.slew_per_s / SLIDE_JOG_SPEED, dt);

  // Intégration des offsets joystick (comportement "latched")
  if (sync_move.active || slideAB.enabled || fabsf(slide_jog_cmd) > 0.001f) {
    // Vitesse d'empilement en steps/s à |joy|=1 (30% de la Vmax de l'axe)
    const float PAN_OFFSET_RATE  = cfg[0].max_speed * 0.3f;  // steps/s
    const float TILT_OFFSET_RATE = cfg[1].max_speed * 0.3f;  // steps/s
    
    long d_pan  = lroundf(joy_filt.pan  * PAN_OFFSET_RATE  * dt);
    long d_tilt = lroundf(joy_filt.tilt * TILT_OFFSET_RATE * dt);
    
    pan_offset_latched  = clampL(pan_offset_latched  + d_pan,  -PAN_OFFSET_RANGE,  +PAN_OFFSET_RANGE);
    tilt_offset_latched = clampL(tilt_offset_latched + d_tilt, -TILT_OFFSET_RANGE, +TILT_OFFSET_RANGE);
  }
  
  // Publie les offsets utilisés par le planificateur
  pan_offset_steps  = pan_offset_latched;
  tilt_offset_steps = tilt_offset_latched;
  slide_jog_cmd     = clampF(joy_filt.slide * joy.slide_speed, -1.f, +1.f);
}

//==================== NEW: Planificateur "temps commun" ====================
uint32_t pick_duration_ms_for_deltas(const long start[NUM_MOTORS], const long goal[NUM_MOTORS], uint32_t T_req_ms){
  double T = T_req_ms / 1000.0;
  for(;;){
    bool ok = true;
    for(int i=0;i<NUM_MOTORS;i++){
      double d = fabs((double)goal[i] - (double)start[i]);
      double v_need = d * 1.875 / T;          // steps/s
      double a_need = d * 5.7735 / (T*T);     // steps/s^2
      if (v_need > cfg[i].max_speed*0.90 || a_need > cfg[i].max_accel*0.90){
        // augmente T de 10%
        T *= 1.10;
        ok = false;
        break;
      }
    }
    if (ok) break;
  }
  return (uint32_t)lround(T*1000.0);
}

//==================== Bank Management Functions ====================
void saveBank(uint8_t idx) {
  if (idx >= 10) return;
  
  // Copier les variables globales vers la structure Bank
  for (int i = 0; i < 8; i++) {
    banks[idx].presets[i] = presets[i];
  }
  for (int i = 0; i < 6; i++) {
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
  for (int i = 0; i < 8; i++) {
    JsonObject preset = presetsArray.createNestedObject();
    preset["p"] = banks[idx].presets[i].p;
    preset["t"] = banks[idx].presets[i].t;
    preset["z"] = banks[idx].presets[i].z;
    preset["s"] = banks[idx].presets[i].s;
    preset["mode"] = banks[idx].presets[i].mode;
    preset["pan_anchor"] = banks[idx].presets[i].pan_anchor;
    preset["tilt_anchor"] = banks[idx].presets[i].tilt_anchor;
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
  
  // Sauvegarder en NVS
  String key = "bank" + String(idx);
  nvs.putString(key.c_str(), jsonString);
  nvs.end();
  
  Serial.printf("💾 Banque %d sauvegardée (%d presets, %d points interp)\n", idx, 8, banks[idx].interpCount);
}

void loadBank(uint8_t idx) {
  if (idx >= 10) return;
  
  // Initialiser NVS
  if (!nvs.begin("banks")) {
    Serial.println("❌ Erreur NVS");
    return;
  }
  
  String key = "bank" + String(idx);
  String jsonString = nvs.getString(key.c_str());
  nvs.end();
  
  if (jsonString.length() == 0) {
    Serial.printf("⚠️ Banque %d vide, initialisation par défaut\n", idx);
    // Initialiser avec des valeurs par défaut
    for (int i = 0; i < 8; i++) {
      banks[idx].presets[i] = {0, 0, 0, 0, 0, 0, 0};
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
    Serial.printf("❌ Erreur JSON banque %d: %s\n", idx, error.c_str());
    return;
  }
  
  // Charger les presets dans la structure Bank ET les variables globales
  JsonArray presetsArray = doc["presets"];
  for (int i = 0; i < 8 && i < presetsArray.size(); i++) {
    JsonObject preset = presetsArray[i];
    banks[idx].presets[i].p = preset["p"];
    banks[idx].presets[i].t = preset["t"];
    banks[idx].presets[i].z = preset["z"];
    banks[idx].presets[i].s = preset["s"];
    banks[idx].presets[i].mode = preset["mode"];
    banks[idx].presets[i].pan_anchor = preset["pan_anchor"];
    banks[idx].presets[i].tilt_anchor = preset["tilt_anchor"];
    
    // Copier vers les variables globales
    presets[i] = banks[idx].presets[i];
  }
  
  // Charger l'interpolation dans la structure Bank ET les variables globales
  JsonArray interpArray = doc["interp"];
  banks[idx].interpCount = min((int)interpArray.size(), 6);
  for (int i = 0; i < banks[idx].interpCount; i++) {
    JsonObject interp = interpArray[i];
    banks[idx].interpPoints[i].presetIndex = interp["presetIndex"];
    banks[idx].interpPoints[i].fraction = interp["fraction"].as<float>() / 100.0f; // Convertir depuis pourcentage
    
    // Copier vers les variables globales
    interpPoints[i] = banks[idx].interpPoints[i];
  }
  interpCount = banks[idx].interpCount;
  
  // Appliquer automatiquement les points d'interpolation (comme si l'utilisateur avait cliqué "Apply")
  // Les points sont déjà chargés dans interpPoints[] et interpCount, l'axe d'interpolation est actif
  
  Serial.printf("📂 Banque %d chargée (%d presets, %d points interp) - Axe d'interpolation activé\n", idx, 8, interpCount);
}

void saveActiveBank() {
  saveBank(activeBank);
}

void loadActiveBank() {
  loadBank(activeBank);
}

//==================== NEW: Mapping slide->pan/tilt ====================
long pan_comp_from_slide(long slide){
  float u = (float)(slide - cfg[3].min_limit) / (float)(cfg[3].max_limit - cfg[3].min_limit);
  return (long) lround(lerp(PAN_AT_SLIDE_MIN, PAN_AT_SLIDE_MAX, clampF(u,0,1)));
}
long tilt_comp_from_slide(long slide){
  float u = (float)(slide - cfg[3].min_limit) / (float)(cfg[3].max_limit - cfg[3].min_limit);
  return (long) lround(lerp(TILT_AT_SLIDE_MIN, TILT_AT_SLIDE_MAX, clampF(u,0,1)));
}

// Helper pour rafraîchir l'ancre de suivi
static inline void follow_refresh_anchor() {
  long s = steppers[3]->targetPos();
  follow.pan_anchor  = steppers[0]->targetPos() - pan_comp_from_slide(s);
  follow.tilt_anchor = steppers[1]->targetPos() - tilt_comp_from_slide(s);
  follow.valid = true;
}

//==================== Interpolation multi-presets ====================
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

//==================== NEW: Tick de coordination ====================
void coordinator_tick(){
  static uint32_t last_ms = millis();
  uint32_t now = millis();
  uint32_t dt_ms = now - last_ms;
  if (dt_ms == 0) return;
  last_ms = now;

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
    
    // Envoyer aux moteurs
    steppers[0]->moveTo(P);
    steppers[1]->moveTo(T);
    steppers[2]->moveTo(Z);
    steppers[3]->moveTo(S);
    
    return;  // ignore les autres modes pendant l'interpolation auto
  }

  // Interpolation d'ancre min-jerk pour recall autour de l'autopan
  if (anchor_morph.active) {
    float tau = (float)(now - anchor_morph.t0_ms) / (float)anchor_morph.T_ms;
    if (tau >= 1.0f) { tau = 1.0f; anchor_morph.active = false; }
    float s = s_minjerk(tau);
    follow.pan_anchor  = lround( lerp((float)anchor_morph.p0, (float)anchor_morph.p1, s) );
    follow.tilt_anchor = lround( lerp((float)anchor_morph.t0, (float)anchor_morph.t1, s) );
    follow.valid = true; // ancres imposées
  }

  // Interpolation d’ancre min-jerk pour recall autour de l’autopan
  
  if (slideAB.enabled && !sync_move.active){
    float tau = (float)(now - slideAB.t0_ms) / (float)slideAB.T_ms;
    if (tau >= 1.0f){ slideAB.dir = -slideAB.dir; slideAB.t0_ms = now; tau = 0.0f; }
    float s = s_minjerk(tau);
    long Sgoal = (slideAB.dir > 0)
                   ? (long)lround(lerp(slideAB.A, slideAB.B, s))
                   : (long)lround(lerp(slideAB.B, slideAB.A, s));
    Sgoal = clampL(Sgoal, cfg[3].min_limit, cfg[3].max_limit);
    steppers[3]->moveTo(Sgoal);

    // Faire suivre Pan/Tilt via la map + offsets joystick
    if (follow.enabled){
      if (!follow.valid) follow_refresh_anchor();
      long pComp = pan_comp_from_slide(Sgoal);
      long tComp = tilt_comp_from_slide(Sgoal);
      bool in_recall = sync_move.active || anchor_morph.active;
      long Pgoal = clampL(follow.pan_anchor  + pComp + active_pan_offset(in_recall),  cfg[0].min_limit, cfg[0].max_limit);
      long Tgoal = clampL(follow.tilt_anchor + tComp + active_tilt_offset(in_recall), cfg[1].min_limit, cfg[1].max_limit);
      steppers[0]->moveTo(Pgoal);
      steppers[1]->moveTo(Tgoal);
    }

    // Ne plus retourner ici — on laisse la suite traiter le joystick P/T
  }

  // 2) Jog direct Pan/Tilt/Slide (vitesse) quand pas de mouvement sync
  if (!sync_move.active){
    float dt = dt_ms / 1000.0f;
    
    // Jog interpolation manuel (prioritaire)
    static float interp_fraction = 0.0f;  // fraction courante sur l'axe d'interpolation
    if (fabs(interp_jog_cmd) > 0.001f) {
      // Déterminer le segment [j, j+1] de la courbe correspondant à interp_fraction
      uint8_t j = 0;
      while (j < interpCount - 1 && interpPoints[j+1].fraction < interp_fraction) {
        j++;
      }
      float u0 = interpPoints[j].fraction;
      float u1 = interpPoints[j+1].fraction;
      if (u1 < u0) u1 = u0;  // sécurité (au cas où, mais interpPoints est trié)
      
      // Calculer les écarts en pas sur ce segment pour chaque axe
      uint8_t presetA = interpPoints[j].presetIndex;
      uint8_t presetB = interpPoints[j+1].presetIndex;
      long dP = presets[presetB].p - presets[presetA].p;
      long dT = presets[presetB].t - presets[presetA].t;
      long dZ = presets[presetB].z - presets[presetA].z;
      long dS = presets[presetB].s - presets[presetA].s;
      float frac_len = (u1 - u0 > 0.0f ? u1 - u0 : 1.0f);
      
      // Calcul de la vitesse fractionnelle maximale autorisée (steps/s limitant)
      float maxFracSpeed = INFINITY;
      if (dP != 0) maxFracSpeed = fmin(maxFracSpeed, cfg[0].max_speed * frac_len / fabs(dP));
      if (dT != 0) maxFracSpeed = fmin(maxFracSpeed, cfg[1].max_speed * frac_len / fabs(dT));
      if (dZ != 0) maxFracSpeed = fmin(maxFracSpeed, cfg[2].max_speed * frac_len / fabs(dZ));
      if (dS != 0) maxFracSpeed = fmin(maxFracSpeed, cfg[3].max_speed * frac_len / fabs(dS));
      if (maxFracSpeed == INFINITY) {
        maxFracSpeed = 0.0f; // aucun mouvement requis si tous décalages nuls
      }
      
      // Intégration de la position fractionnelle en fonction de la vitesse demandée
      float du = interp_jog_cmd * maxFracSpeed * dt;
      interp_fraction = clampF(interp_fraction + du, 0.0f, 1.0f);
      
      // Calculer la nouvelle position interpolée et l'envoyer aux moteurs
      long P, T, Z, S;
      computeInterpolatedPosition(interp_fraction, P, T, Z, S);
      steppers[0]->moveTo(P);
      steppers[1]->moveTo(T);
      steppers[2]->moveTo(Z);
      steppers[3]->moveTo(S);
      
      // On quitte pour ne pas interférer avec les autres jogs
      return;
    }
    
    // Jog Pan
    if (fabs(joy_filt.pan) > 0.001f) {
      long p = steppers[0]->targetPos();
      p = clampL(p + (long)lround(joy_filt.pan * PAN_JOG_SPEED * joy.pan_tilt_speed * dt), cfg[0].min_limit, cfg[0].max_limit);
      steppers[0]->moveTo(p);
    }
    
    // Jog Tilt
    if (fabs(joy_filt.tilt) > 0.001f) {
      long t = steppers[1]->targetPos();
      t = clampL(t + (long)lround(joy_filt.tilt * TILT_JOG_SPEED * joy.pan_tilt_speed * dt), cfg[1].min_limit, cfg[1].max_limit);
      steppers[1]->moveTo(t);
    }
    
    // Jog Slide (+ follow map for Pan/Tilt) - bloqué pendant AB
    if (!slideAB.enabled && fabs(slide_jog_cmd) > 0.001f){
      long s = steppers[3]->targetPos();
      long Sgoal = clampL(s + (long)lround(slide_jog_cmd * SLIDE_JOG_SPEED * dt),
                          cfg[3].min_limit, cfg[3].max_limit);
      steppers[3]->moveTo(Sgoal);

      // Faire suivre Pan/Tilt via la map slide->pan/tilt
      if (follow.enabled) {
        if (!follow.valid) follow_refresh_anchor();
        long pComp = pan_comp_from_slide(Sgoal);
        long tComp = tilt_comp_from_slide(Sgoal);
        bool in_recall = sync_move.active || anchor_morph.active;
        long Pgoal = clampL(follow.pan_anchor  + pComp + active_pan_offset(in_recall),  cfg[0].min_limit, cfg[0].max_limit);
        long Tgoal = clampL(follow.tilt_anchor + tComp + active_tilt_offset(in_recall), cfg[1].min_limit, cfg[1].max_limit);
        
        // Ne PAS écraser un axe si le joystick le pilote déjà
        bool joyP = fabsf(joy_filt.pan)  > 0.001f;
        bool joyT = fabsf(joy_filt.tilt) > 0.001f;
        
        if (!joyP) steppers[0]->moveTo(Pgoal);
        if (!joyT) steppers[1]->moveTo(Tgoal);
      }
    }
  }

  // 2) Mouvement synchronisé
  if (sync_move.active){
    float tau = (float)(now - sync_move.t0_ms) / (float)sync_move.T_ms;
    if (tau >= 1.0f){
      // Fin de mouvement
      sync_move.active = false;
      follow.valid = false;   // re-anchor next time
      
      // Rephase AB après GOTO_THEN_RESUME pour éviter un saut
      if (recallPolicy.slide == SlideRecallPolicy::GOTO_THEN_RESUME && slideAB.enabled) {
        slideAB.t0_ms = millis();
        long current_slide = steppers[3]->getCurrentPosition();
        slideAB.dir = (abs(current_slide - slideAB.A) < abs(current_slide - slideAB.B)) ? +1 : -1;
        Serial.println("🔄 AB rephased after GOTO_THEN_RESUME");
      }
      
      tau = 1.0f;
    }
    float s = s_minjerk(tau);

    // Slide de référence (pour couplage)
    long slide_ref = (long)lround( sync_move.start[3] + (sync_move.goal_base[3] - sync_move.start[3]) * s );
    slide_ref = clampL(slide_ref, cfg[3].min_limit, cfg[3].max_limit);

    // Compensations en fonction du slide + offsets joystick (toujours actifs)
    long pan_comp  = pan_comp_from_slide(slide_ref);
    long tilt_comp = tilt_comp_from_slide(slide_ref);

    long pan_goal  = sync_move.goal_base[0] + pan_comp + eff_pan_offset();
    long tilt_goal = sync_move.goal_base[1] + tilt_comp + eff_tilt_offset();
    long zoom_goal = sync_move.goal_base[2];
    long slide_goal= sync_move.goal_base[3];

    // Cibles "à l'instant" suivant s(t)
    long P = (long)lround( sync_move.start[0] + (pan_goal  - sync_move.start[0]) * s );
    long T = (long)lround( sync_move.start[1] + (tilt_goal - sync_move.start[1]) * s );
    long Z = (long)lround( sync_move.start[2] + (zoom_goal - sync_move.start[2]) * s );
    long S = (long)lround( sync_move.start[3] + (slide_goal- sync_move.start[3]) * s );

    // Clip limites
    P = clampL(P, cfg[0].min_limit, cfg[0].max_limit);
    T = clampL(T, cfg[1].min_limit, cfg[1].max_limit);
    Z = clampL(Z, cfg[2].min_limit, cfg[2].max_limit);
    S = clampL(S, cfg[3].min_limit, cfg[3].max_limit);

    // On pousse les cibles. FastAccelStepper replanifie en douceur.
    steppers[0]->moveTo(P);
    steppers[1]->moveTo(T);
    steppers[2]->moveTo(Z);
    steppers[3]->moveTo(S);
  }
}

//==================== Variables globales ====================
long panPos = 0;
long tiltPos = 0;
long zoomPos = 0;
long slidePos = 0;

//==================== OSC ====================
WiFiUDP udp;
OSCErrorCode error;
const int OSC_PORT = 8000;

//==================== Web Server ====================
AsyncWebServer webServer(80);

//==================== Setup OSC ====================
void setupOSC() {
  if (udp.begin(OSC_PORT)) {
    Serial.println("✅ OSC Server started on port " + String(OSC_PORT));
    Serial.println("📡 Waiting for OSC messages...");
  } else {
    Serial.println("❌ Failed to start OSC server on port " + String(OSC_PORT));
  }
}

//==================== Process OSC ====================
void processOSC() {
  OSCMessage msg;
  int size = udp.parsePacket();
  
  if (size > 0) {
    Serial.println("🔔 OSC packet received, size: " + String(size));
    
    while (size--) {
      msg.fill(udp.read());
    }
    
    Serial.println("🔍 OSC message address: " + String(msg.getAddress()));
    Serial.println("🔍 OSC message size: " + String(msg.size()));
    
    if (!msg.hasError()) {
      // Traitement des messages OSC
      // Joystick en OSC (-1..+1)
      msg.dispatch("/pan", [](OSCMessage &m){ 
        // Annuler preset selon la politique
        if (cancel.by_joystick && sync_move.active) {
          sync_move.active = false;
          Serial.println("⏹️ Cancel by joystick");
        }
        
        joy_raw.pan = clampF(m.getFloat(0), -1.f, +1.f); 
      });
      msg.dispatch("/tilt", [](OSCMessage &m){ 
        // Annuler preset selon la politique
        if (cancel.by_joystick && sync_move.active) {
          sync_move.active = false;
          Serial.println("⏹️ Cancel by joystick");
        }
        
        joy_raw.tilt = clampF(m.getFloat(0), -1.f, +1.f); 
      });
      msg.dispatch("/joy/pt", [](OSCMessage &m){ 
        // Annuler preset selon la politique
        if (cancel.by_joystick && sync_move.active) {
          sync_move.active = false;
          Serial.println("⏹️ Cancel by joystick");
        }
        
        joy_raw.pan = clampF(m.getFloat(0), -1.f, +1.f);
        joy_raw.tilt = clampF(m.getFloat(1), -1.f, +1.f); 
      });
      msg.dispatch("/slide/jog", [](OSCMessage &m){ 
        // Annuler preset selon la politique
        if (cancel.by_joystick && sync_move.active) {
          sync_move.active = false;
          Serial.println("⏹️ Cancel by joystick");
        }
        
        joy_raw.slide = clampF(m.getFloat(0), -1.f, +1.f); 
      });
      
      // Optionnel: réglages runtime
      msg.dispatch("/joy/config", [](OSCMessage &m){
        if (m.size() > 0) joy.deadzone = clampF(m.getFloat(0), 0.f, 0.5f);
        if (m.size() > 1) joy.expo = clampF(m.getFloat(1), 0.f, 0.95f);
        if (m.size() > 2) joy.slew_per_s = fabsf(m.getFloat(2));
        if (m.size() > 3) joy.filt_hz = fabsf(m.getFloat(3));
        if (m.size() > 4) joy.pan_tilt_speed = clampF(m.getFloat(4), 0.1f, 3.0f);
        if (m.size() > 5) joy.slide_speed = clampF(m.getFloat(5), 0.1f, 3.0f);
        
        Serial.printf("🎛 Joy cfg: dz=%.2f expo=%.2f slew=%.0f filt=%.1f PT=%.2fx S=%.2fx\n",
                      joy.deadzone, joy.expo, joy.slew_per_s, joy.filt_hz,
                      joy.pan_tilt_speed, joy.slide_speed);
      });
      
      // Configuration de la politique d'annulation
      msg.dispatch("/preset/cancel_policy", [](OSCMessage &m){
        if (m.size() > 0) cancel.by_joystick = m.getInt(0) != 0;
        if (m.size() > 1) cancel.by_axis = m.getInt(1) != 0;
        Serial.printf("⚙️ Cancel policy: joystick=%d axis=%d\n", cancel.by_joystick, cancel.by_axis);
      });
      
      // Configuration du suivi Pan/Tilt ↔ Slide
      msg.dispatch("/follow/en", [](OSCMessage &m){
        follow.enabled = m.getInt(0) != 0;
        follow.valid = false;
        Serial.printf("🎯 Follow mapping on jog: %s\n", follow.enabled ? "ON" : "OFF");
      });
      
      // Mode AB infini pour le slide (déprécié)
      msg.dispatch("/slide/ab", [](OSCMessage &m){
        slideAB.enabled = m.getInt(0) != 0;
        slideAB.t0_ms = millis();
        follow.valid = false; // re-anchor au démarrage
        Serial.printf("♾️ Slide AB %s\n", slideAB.enabled ? "ON" : "OFF");
      });
      
      msg.dispatch("/slide/ab/set", [](OSCMessage &m){
        float uA = clampF(m.getFloat(0), 0.f, 1.f);
        float uB = clampF(m.getFloat(1), 0.f, 1.f);
        float T  = fabsf(m.getFloat(2)); if (T <= 0) T = 2.f;
        slideAB.A    = (long)lround(lerp(cfg[3].min_limit, cfg[3].max_limit, uA));
        slideAB.B    = (long)lround(lerp(cfg[3].min_limit, cfg[3].max_limit, uB));
        slideAB.T_ms = (uint32_t)lround(T * 1000.f);
        slideAB.t0_ms= millis();
        slideAB.dir  = +1;
        Serial.printf("AB set A=%ld B=%ld T=%u ms\n", slideAB.A, slideAB.B, slideAB.T_ms);
      });
      
      //==================== Routes OSC pour interpolation multi-presets ====================
      msg.dispatch("/interp/setpoints", [](OSCMessage &m){
        if (m.size() < 1) return;
        uint8_t N = m.getInt(0);
        if (N > 6) N = 6;  // max 6 points
        if (N < 2) N = 2;  // min 2 points
        
        interpCount = N;
        for (uint8_t j = 0; j < N; ++j) {
          if ((1 + 2*j + 1) < m.size()) {
            interpPoints[j].presetIndex = m.getInt(1 + 2*j);
            interpPoints[j].fraction = m.getFloat(1 + 2*j + 1);
          }
        }
        Serial.printf("🎯 Interp points set: N=%d\n", N);
        for (uint8_t j = 0; j < N; ++j) {
          Serial.printf("   Point %d: Preset %d @ %.1f%%\n", j, interpPoints[j].presetIndex, interpPoints[j].fraction * 100.0f);
        }
      });
      
      msg.dispatch("/interp/auto", [](OSCMessage &m){
        bool enable = m.getInt(0) != 0;
        float duration = (m.size() > 1 ? m.getFloat(1) : 5.0f);
        
        if (enable) {
          uint32_t T_ms = (duration <= 0 ? 5000 : (uint32_t)lround(duration * 1000));
          interpAuto.T_ms = T_ms;
          interpAuto.t0_ms = millis();
          interpAuto.active = true;
          
          // Désactiver modes concurrents
          slideAB.enabled = false;
          sync_move.active = false;
          follow.valid = false;
          
          Serial.printf("▶️ Interpolation auto ON (T=%u ms, %.1fs)\n", T_ms, duration);
        } else {
          interpAuto.active = false;
          Serial.println("⏹️ Interpolation auto OFF");
        }
      });
      
      msg.dispatch("/interp/goto", [](OSCMessage &m){
        float fraction = clampF(m.getFloat(0), 0.0f, 1.0f);
        
        // Annuler modes auto
        interpAuto.active = false;
        sync_move.active = false;
        slideAB.enabled = false;
        
        // Calculer position interpolée
        long P, T, Z, S;
        computeInterpolatedPosition(fraction, P, T, Z, S);
        
        // Commander les moteurs
        steppers[0]->moveTo(P);
        steppers[1]->moveTo(T);
        steppers[2]->moveTo(Z);
        steppers[3]->moveTo(S);
        
        Serial.printf("🎛️ Manual interp goto %.1f%% -> P=%ld T=%ld Z=%ld S=%ld\n", 
                      fraction * 100.0f, P, T, Z, S);
      });

      msg.dispatch("/interp/jog", [](OSCMessage &m){
        float value = clampF(m.getFloat(0), -1.0f, 1.0f);
        
        // Annuler les mouvements automatiques ou presets en cours
        interpAuto.active = false;
        sync_move.active = false;
        slideAB.enabled = false;
        follow.valid = false;
        
        // Appliquer la nouvelle consigne de vitesse de l'axe d'interpolation
        interp_jog_cmd = value;
        Serial.printf("🎛️ Interp jog speed = %.2f\n", value);
      });
      
      msg.dispatch("/axis_pan", [](OSCMessage &msg) {
        // Annuler preset selon la politique
        if (cancel.by_axis && sync_move.active) {
          sync_move.active = false;
          Serial.println("⏹️ Manual override: cancel preset");
        }
        
        float value = clampF(msg.getFloat(0), 0.0f, 1.0f);
        long pos_val = (long)(value * (cfg[0].max_limit - cfg[0].min_limit) + cfg[0].min_limit);
        Serial.println("🔧 Moving Pan to: " + String(pos_val));
        steppers[0]->moveTo(pos_val);
        Serial.println("Axis Pan: " + String(value) + " -> " + String(pos_val));
        Serial.println("🔧 Pan stepper running: " + String(steppers[0]->isRunning()));
      });
      
      msg.dispatch("/axis_tilt", [](OSCMessage &msg) {
        // Annuler preset selon la politique
        if (cancel.by_axis && sync_move.active) {
          sync_move.active = false;
          Serial.println("⏹️ Manual override: cancel preset");
        }
        
        float value = clampF(msg.getFloat(0), 0.0f, 1.0f);
        long pos_val = (long)(value * (cfg[1].max_limit - cfg[1].min_limit) + cfg[1].min_limit);
        Serial.println("🔧 Moving Tilt to: " + String(pos_val));
        steppers[1]->moveTo(pos_val);
        Serial.println("Axis Tilt: " + String(value) + " -> " + String(pos_val));
        Serial.println("🔧 Tilt stepper running: " + String(steppers[1]->isRunning()));
      });
      
      msg.dispatch("/axis_zoom", [](OSCMessage &msg) {
        // Annuler preset selon la politique
        if (cancel.by_axis && sync_move.active) {
          sync_move.active = false;
          Serial.println("⏹️ Manual override: cancel preset");
        }
        
        float value = clampF(msg.getFloat(0), 0.0f, 1.0f);
        long pos_val = (long)(value * (cfg[2].max_limit - cfg[2].min_limit) + cfg[2].min_limit);
        Serial.println("🔧 Moving Zoom to: " + String(pos_val));
        steppers[2]->moveTo(pos_val);
        Serial.println("Axis Zoom: " + String(value) + " -> " + String(pos_val));
        Serial.println("🔧 Zoom stepper running: " + String(steppers[2]->isRunning()));
      });
      
      msg.dispatch("/axis_slide", [](OSCMessage &msg) {
        // Annuler preset selon la politique
        if (cancel.by_axis && sync_move.active) {
          sync_move.active = false;
          Serial.println("⏹️ Manual override: cancel preset");
        }
        
        float value = clampF(msg.getFloat(0), 0.0f, 1.0f);
        long pos_val = (long)(value * (cfg[3].max_limit - cfg[3].min_limit) + cfg[3].min_limit);
        Serial.println("🔧 Moving Slide to: " + String(pos_val));
        steppers[3]->moveTo(pos_val);
        Serial.println("Axis Slide: " + String(value) + " -> " + String(pos_val));
        Serial.println("🔧 Slide stepper running: " + String(steppers[3]->isRunning()));
      });
      
      //==================== NEW: Routes OSC avancées ====================
      // Homing slide & StallGuard threshold
      msg.dispatch("/slide/home", [](OSCMessage &m){
        Serial.println("\xF0\x9F\x8F\xA0 Commande OSC: homing du slide");
        home_slide();
      });
      msg.dispatch("/slide/sgthrs", [](OSCMessage &m){
        if (m.size() > 0) {
          int thr = m.getInt(0);
          if (thr < 0) thr = 0; if (thr > 255) thr = 255;
          slide_sg_threshold = (uint8_t)thr;
          drivers[3]->SGTHRS(slide_sg_threshold);
          cfg[3].sgt = slide_sg_threshold;
          Serial.printf("\xE2\x9A\x99\xEF\xB8\x8F Nouvelle SGTHRS (slide) = %d\n", slide_sg_threshold);
        }
      });

      //==================== Motor Configuration Routes ====================
      msg.dispatch("/motor/pan/max_speed", [](OSCMessage &m){
        if (m.size() > 0) {
          int speed = m.getInt(0);
          if (speed >= 2000 && speed <= 20000){
            cfg[0].max_speed = speed;
            steppers[0]->setSpeedInHz(speed);
            Serial.printf("🔧 Pan max_speed = %d steps/s\n", speed);
          }
        }
      });

      msg.dispatch("/motor/pan/max_accel", [](OSCMessage &m){
        if (m.size() > 0) {
          int accel = m.getInt(0);
          if (accel >= 1000 && accel <= 999999){
            cfg[0].max_accel = accel;
            steppers[0]->setAcceleration(accel);
            Serial.printf("🔧 Pan max_accel = %d steps/s²\n", accel);
          }
        }
      });

      msg.dispatch("/motor/tilt/max_speed", [](OSCMessage &m){
        if (m.size() > 0) {
          int speed = m.getInt(0);
          if (speed >= 2000 && speed <= 20000){
            cfg[1].max_speed = speed;
            steppers[1]->setSpeedInHz(speed);
            Serial.printf("🔧 Tilt max_speed = %d steps/s\n", speed);
          }
        }
      });

      msg.dispatch("/motor/tilt/max_accel", [](OSCMessage &m){
        if (m.size() > 0) {
          int accel = m.getInt(0);
          if (accel >= 1000 && accel <= 999999){
            cfg[1].max_accel = accel;
            steppers[1]->setAcceleration(accel);
            Serial.printf("🔧 Tilt max_accel = %d steps/s²\n", accel);
          }
        }
      });

      msg.dispatch("/motor/zoom/max_speed", [](OSCMessage &m){
        if (m.size() > 0) {
          int speed = m.getInt(0);
          if (speed >= 2000 && speed <= 20000){
            cfg[2].max_speed = speed;
            steppers[2]->setSpeedInHz(speed);
            Serial.printf("🔧 Zoom max_speed = %d steps/s\n", speed);
          }
        }
      });

      msg.dispatch("/motor/zoom/max_accel", [](OSCMessage &m){
        if (m.size() > 0) {
          int accel = m.getInt(0);
          if (accel >= 1000 && accel <= 999999){
            cfg[2].max_accel = accel;
            steppers[2]->setAcceleration(accel);
            Serial.printf("🔧 Zoom max_accel = %d steps/s²\n", accel);
          }
        }
      });

      msg.dispatch("/motor/slide/max_speed", [](OSCMessage &m){
        if (m.size() > 0) {
          int speed = m.getInt(0);
          if (speed >= 2000 && speed <= 20000){
            cfg[3].max_speed = speed;
            steppers[3]->setSpeedInHz(speed);
            Serial.printf("🔧 Slide max_speed = %d steps/s\n", speed);
          }
        }
      });

      msg.dispatch("/motor/slide/max_accel", [](OSCMessage &m){
        if (m.size() > 0) {
          int accel = m.getInt(0);
          if (accel >= 1000 && accel <= 999999){
            cfg[3].max_accel = accel;
            steppers[3]->setAcceleration(accel);
            Serial.printf("🔧 Slide max_accel = %d steps/s²\n", accel);
          }
        }
      });

      //==================== Bank Management Routes ====================
      msg.dispatch("/bank/set", [](OSCMessage &m){
        if (m.size() > 0) {
          uint8_t idx = m.getInt(0);
          if (idx < 10) {
            activeBank = idx;
            loadBank(idx);
            Serial.printf("🏦 Banque active changée vers %d\n", idx);
            
            // Renvoyer les points d'interpolation après le chargement
            DynamicJsonDocument doc(1024);
            doc["interpCount"] = interpCount;
            
            JsonArray interpArray = doc.createNestedArray("interp");
            for (int i = 0; i < interpCount; i++) {
              JsonObject interp = interpArray.createNestedObject();
              interp["presetIndex"] = interpPoints[i].presetIndex;
              interp["fraction"] = interpPoints[i].fraction * 100.0f; // Convertir en pourcentage
            }
            
            // Sérialiser et envoyer
            String jsonString;
            serializeJson(doc, jsonString);
            Serial.printf("📤 Bank %d interp points: %s\n", idx, jsonString.c_str());
          }
        }
      });

      msg.dispatch("/bank/save", [](OSCMessage &m){
        saveActiveBank();
        Serial.printf("💾 Banque %d sauvegardée\n", activeBank);
      });

      msg.dispatch("/bank/get_interp", [](OSCMessage &m){
        // Construire le JSON avec les points d'interpolation actuels
        DynamicJsonDocument doc(1024);
        doc["interpCount"] = interpCount;
        
        JsonArray interpArray = doc.createNestedArray("interp");
        for (int i = 0; i < interpCount; i++) {
          JsonObject interp = interpArray.createNestedObject();
          interp["presetIndex"] = interpPoints[i].presetIndex;
          interp["fraction"] = interpPoints[i].fraction * 100.0f; // Convertir en pourcentage
        }
        
        // Sérialiser en string
        String jsonString;
        serializeJson(doc, jsonString);
        
        // Envoyer via OSC (on va utiliser une route spéciale pour le retour)
        // Pour l'instant, on log le JSON
        Serial.printf("📤 Interp points JSON: %s\n", jsonString.c_str());
      });

      msg.dispatch("/preset/set", [](OSCMessage &m){
        int i = m.getInt(0);
        presets[i].p = m.getInt(1);
        presets[i].t = m.getInt(2);
        presets[i].z = m.getInt(3);
        presets[i].s = m.getInt(4);
        Serial.printf("Preset %d saved\n", i);
      });

      // Store réel: capture positions courantes et calcule les ancres
      msg.dispatch("/preset/store", [](OSCMessage &m){
        int i = m.getInt(0);
        long S = steppers[3]->getCurrentPosition();
        long P = steppers[0]->getCurrentPosition();
        long T = steppers[1]->getCurrentPosition();
        long Z = steppers[2]->getCurrentPosition();

        long pComp = pan_comp_from_slide(S);
        long tComp = tilt_comp_from_slide(S);

        presets[i].p = P; presets[i].t = T; presets[i].z = Z; presets[i].s = S;
        presets[i].pan_anchor  = P - pComp;
        presets[i].tilt_anchor = T - tComp;
        presets[i].mode = follow.enabled ? PRESET_FOLLOW_SLIDE : PRESET_ABSOLUTE;

        Serial.printf("\xF0\x9F\x92\xBE Store preset %d | ABS P:%ld T:%ld Z:%ld S:%ld | ANCHOR P:%ld T:%ld | mode:%d\n",
                      i, P,T,Z,S, presets[i].pan_anchor, presets[i].tilt_anchor, presets[i].mode);
      });

      // Forcer le mode d'un preset
      msg.dispatch("/preset/mode", [](OSCMessage &m){
        int i = m.getInt(0);
        int md = m.getInt(1);
        presets[i].mode = (md==1) ? PRESET_FOLLOW_SLIDE : PRESET_ABSOLUTE;
        Serial.printf("Preset %d mode=%d\n", i, presets[i].mode);
      });

      // Policy de recall slide
      msg.dispatch("/preset/recall_policy", [](OSCMessage &m){
        if (m.size() > 0) {
          int sp = m.getInt(0);
          recallPolicy.slide = (sp == 1) ? SlideRecallPolicy::GOTO_THEN_RESUME : SlideRecallPolicy::KEEP_AB;
          Serial.printf("Recall policy slide=%d\n", (int)recallPolicy.slide);
        }
      });

      msg.dispatch("/preset/recall", [](OSCMessage &m){
        int i = m.getInt(0);
        float Tsec = m.getFloat(1); if (Tsec <= 0) Tsec = 2.0f;
        activePreset = i;

        // Baseline des offsets au début du recall
        offset_session.pan0  = pan_offset_latched;
        offset_session.tilt0 = tilt_offset_latched;

        uint32_t Tms_req = (uint32_t)lround(Tsec*1000.0);

        bool can_follow_recall = follow.enabled && slideAB.enabled && (presets[i].mode==PRESET_FOLLOW_SLIDE);
        if (can_follow_recall && recallPolicy.slide == SlideRecallPolicy::KEEP_AB) {
          // Morph des ancres autour de l'autopan AB : n'arme pas sync_move
          anchor_morph.active = true;
          anchor_morph.p0 = follow.pan_anchor;
          anchor_morph.t0 = follow.tilt_anchor;
          anchor_morph.p1 = presets[i].pan_anchor;
          anchor_morph.t1 = presets[i].tilt_anchor;
          anchor_morph.t0_ms = millis();
          anchor_morph.T_ms  = Tms_req;
          follow.valid = true;
          Serial.printf("\xE2\x96\xBA Recall(FOLLOW+AB): morph anchors to P:%ld T:%ld in %u ms\n",
                        anchor_morph.p1, anchor_morph.t1, anchor_morph.T_ms);
        } else {
          // Mouvement synchronisé normal (ABSOLUTE ou policy GOTO_THEN_RESUME)
          long base_goal[NUM_MOTORS] = { presets[i].p, presets[i].t, presets[i].z, presets[i].s };

          for(int ax=0; ax<NUM_MOTORS; ++ax){
            sync_move.start[ax]     = steppers[ax]->getCurrentPosition();
            sync_move.goal_base[ax] = clampL(base_goal[ax], cfg[ax].min_limit, cfg[ax].max_limit);
          }

          sync_move.T_ms = pick_duration_ms_for_deltas(sync_move.start, sync_move.goal_base, Tms_req);
          sync_move.t0_ms = millis();
          sync_move.active = true;
          Serial.printf("\xE2\x96\xBA Recall(ABS): P:%ld T:%ld Z:%ld S:%ld in %u ms\n",
                        sync_move.goal_base[0], sync_move.goal_base[1],
                        sync_move.goal_base[2], sync_move.goal_base[3], sync_move.T_ms);
        }
      });

      // Note: /pan, /tilt, /slide/jog sont déjà gérés plus haut dans le pipeline joystick

      // Slide: goto [0..1] en T sec (déplacement temps imposé)
      msg.dispatch("/slide/goto", [](OSCMessage &m){
        float u = clampF(m.getFloat(0), 0.0f, 1.0f);
        float Tsec = m.getFloat(1); if (Tsec <= 0) Tsec = 2.0f;

        long s_goal = (long)lround(lerp(cfg[3].min_limit, cfg[3].max_limit, u));
        // Construire un "preset" ad-hoc qui ne bouge que le slide
        for(int ax=0; ax<NUM_MOTORS; ++ax){
          sync_move.start[ax]     = steppers[ax]->getCurrentPosition();
          sync_move.goal_base[ax] = (ax==3) ? s_goal : sync_move.start[ax];
        }
        uint32_t Tms_req = (uint32_t)lround(Tsec*1000.0);
        sync_move.T_ms = pick_duration_ms_for_deltas(sync_move.start, sync_move.goal_base, Tms_req);
        sync_move.t0_ms = millis();
        sync_move.active = true;
      });

      // Config: ranges offsets et mapping slide->pan/tilt
      msg.dispatch("/config/offset_range", [](OSCMessage &m){
        PAN_OFFSET_RANGE  = m.getInt(0);
        TILT_OFFSET_RANGE = m.getInt(1);
      });
      msg.dispatch("/config/pan_map", [](OSCMessage &m){
        PAN_AT_SLIDE_MIN = m.getInt(0);
        PAN_AT_SLIDE_MAX = m.getInt(1);
      });
      msg.dispatch("/config/tilt_map", [](OSCMessage &m){
        TILT_AT_SLIDE_MIN = m.getInt(0);
        TILT_AT_SLIDE_MAX = m.getInt(1);
      });
      
      //==================== NEW: Routes OSC pour offsets latched ====================
      msg.dispatch("/offset/zero", [](OSCMessage &m){
        int do_pan  = (m.size()>0) ? m.getInt(0) : 1;
        int do_tilt = (m.size()>1) ? m.getInt(1) : 1;
        if (do_pan)  pan_offset_latched  = 0;
        if (do_tilt) tilt_offset_latched = 0;
        Serial.println("🔄 Reset offsets: pan=" + String(pan_offset_latched) + ", tilt=" + String(tilt_offset_latched));
      });
      
      msg.dispatch("/offset/add", [](OSCMessage &m){
        if (m.size()>0)
          pan_offset_latched  = clampL(pan_offset_latched  + m.getInt(0), -PAN_OFFSET_RANGE,  PAN_OFFSET_RANGE);
        if (m.size()>1)
          tilt_offset_latched = clampL(tilt_offset_latched + m.getInt(1), -TILT_OFFSET_RANGE, TILT_OFFSET_RANGE);
        Serial.println("➕ Add offsets: pan=" + String(pan_offset_latched) + ", tilt=" + String(tilt_offset_latched));
      });
      
      msg.dispatch("/offset/set", [](OSCMessage &m){
        if (m.size()>0) pan_offset_latched  = clampL(m.getInt(0), -PAN_OFFSET_RANGE,  PAN_OFFSET_RANGE);
        if (m.size()>1) tilt_offset_latched = clampL(m.getInt(1), -TILT_OFFSET_RANGE, TILT_OFFSET_RANGE);
        Serial.println("🎯 Set offsets: pan=" + String(pan_offset_latched) + ", tilt=" + String(tilt_offset_latched));
      });
      
      msg.dispatch("/offset/bake", [](OSCMessage &m){
        if (sync_move.active){
          // Intègre l'offset actuel dans la goal_base du preset
          sync_move.goal_base[0] = clampL(sync_move.goal_base[0] + pan_offset_latched,  cfg[0].min_limit, cfg[0].max_limit);
          sync_move.goal_base[1] = clampL(sync_move.goal_base[1] + tilt_offset_latched, cfg[1].min_limit, cfg[1].max_limit);
          Serial.println("🍞 Bake offsets into preset: pan=" + String(sync_move.goal_base[0]) + ", tilt=" + String(sync_move.goal_base[1]));
        }
        pan_offset_latched  = 0;
        tilt_offset_latched = 0;
        Serial.println("🔄 Reset offsets after bake");
      });
    } else {
      Serial.println("❌ OSC Error: " + String(msg.getError()));
    }
  }
}

//==================== Web Handlers ====================
void setupWebServer() {
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
    doc["modes"]["syncMove"] = sync_move.active;
    doc["modes"]["slideAB"] = slideAB.enabled;
    doc["modes"]["follow"] = follow.valid;
    
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
  webServer.on("/api/bank/{id}", HTTP_GET, [](AsyncWebServerRequest *req){
    int bankId = req->pathArg(0).toInt();
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
      preset["pan_anchor"] = p.pan_anchor;
      preset["tilt_anchor"] = p.tilt_anchor;
      preset["mode"] = p.mode;
    }
    
    // Récupérer les points d'interpolation de la banque
    doc["interpCount"] = banks[bankId].interpCount;
    JsonArray interpArray = doc.createNestedArray("interpPoints");
    for (int i = 0; i < banks[bankId].interpCount; i++) {
      JsonObject point = interpArray.createNestedObject();
      point["presetIndex"] = banks[bankId].interpPoints[i].presetIndex;
      point["fraction"] = banks[bankId].interpPoints[i].fraction;
    }
    
    String jsonString;
    serializeJson(doc, jsonString);
    req->send(200, "application/json", jsonString);
  });
  
  webServer.begin();
  Serial.println("🌐 Web server started");
}

//==================== Setup ====================
void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("🚀 ESP32 Slider Controller Starting...");
  
  // Calculer les vitesses de jog basées sur la config
  PAN_JOG_SPEED = cfg[0].max_speed * 0.8f;   // 80% de la vitesse max
  TILT_JOG_SPEED = cfg[1].max_speed * 0.8f;  // 80% de la vitesse max
  SLIDE_JOG_SPEED = cfg[3].max_speed * 0.8f;  // 80% de la vitesse max
  
  Serial.printf("🎯 Jog speeds: Pan=%.0f Tilt=%.0f Slide=%.0f steps/s\n", 
                PAN_JOG_SPEED, TILT_JOG_SPEED, SLIDE_JOG_SPEED);
  
  // Charger la banque 0 au démarrage
  loadBank(0);
  Serial.println("📂 Banque 0 chargée au démarrage");
  
  // Désactiver les anciens modes par défaut
  follow.enabled = false;
  slideAB.enabled = false;
  
  // Initialiser l'engine
  engine.init();
  
  // Configurer les drivers TMC
  setupDriversTMC();

  // Attacher les steppers
  for (int i=0; i<NUM_MOTORS; i++) {
    steppers[i] = engine.stepperConnectToPin(STEP_PINS[i]);
    if (steppers[i]) {
      Serial.println("✅ Stepper " + String(i) + " connected to pin " + String(STEP_PINS[i]));
      steppers[i]->setDirectionPin(DIR_PINS[i]);
      steppers[i]->setEnablePin(ENABLE_PINS[i], true);   // true = active LOW pour TMC2209
      steppers[i]->setAutoEnable(false);                 // Garde les moteurs alimentés
      steppers[i]->setSpeedInHz(cfg[i].max_speed);
      steppers[i]->setAcceleration(cfg[i].max_accel);
      steppers[i]->enableOutputs();                      // Force l'activation maintenant
      Serial.println("✅ Stepper " + String(i) + " configured");
    } else {
      Serial.println("❌ Failed to connect stepper " + String(i));
    }
  }
  
  // WiFi Manager
  WiFiManager wm;
  wm.autoConnect("ESP32-Slider");
  
  Serial.println("📡 WiFi connected: " + WiFi.localIP().toString());
  
  // OTA
  ArduinoOTA.begin();
  
  // Web Server
  setupWebServer();
  
  // OSC
  setupOSC();

  Serial.println("🎯 System ready!");

  // Homing automatique optionnel
  if (doAutoHomeSlide) {
    Serial.println("\xF0\x9F\x8F\xA0 Homing automatique du slide au d\xC3\xA9marrage...");
    home_slide();
  }
}

//==================== Loop ====================
void loop() {
  ArduinoOTA.handle();
  processOSC();
  joystick_tick();     // NEW: Pipeline joystick avec lissage
  coordinator_tick();  // NEW: Orchestrateur de mouvements synchronisés
  
  // FastAccelStepper n'a pas besoin de engine.run()
  // Les moteurs se déplacent automatiquement
  
  // Mettre à jour les positions
  panPos = steppers[0]->getCurrentPosition();
  tiltPos = steppers[1]->getCurrentPosition();
  zoomPos = steppers[2]->getCurrentPosition();
  slidePos = steppers[3]->getCurrentPosition();

  // Log périodique (console web + série)
  static unsigned long tlog = 0;
  static unsigned long osc_log = 0;
  if (millis() - tlog > 500) {
    tlog = millis();
    String s = "t=" + String(millis()/1000.0, 2) + " jog=" + String(slide_jog_cmd, 2) +
               " | P:" + String(panPos) + " T:" + String(tiltPos) +
               " Z:" + String(zoomPos) + " S:" + String(slidePos);
    Serial.println(s);
  }
  
  // Log OSC status toutes les 5 secondes
  if (millis() - osc_log > 5000) {
    osc_log = millis();
    Serial.println("🔍 OSC listening on port " + String(OSC_PORT));
    
    // Vérifier l'état des drivers TMC
    for (int i=0; i<NUM_MOTORS; i++) {
      Serial.println("🔧 Driver " + String(i) + " toff: " + String(drivers[i]->toff()) + 
                     " tstep: " + String(drivers[i]->TSTEP()));
    }
  }
}