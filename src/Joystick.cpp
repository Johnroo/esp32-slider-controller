/**
 * @file Joystick.cpp
 * @brief Implémentation du module de gestion du joystick
 * @author Laurent Eyen
 * @date 2024
 */

#include "Joystick.h"
#include "MotorControl.h"
#include "MotionPlanner.h"
#include "Presets.h"

//==================== Variables globales ====================
JoyCfg joy;
JoyState joy_raw;      // alimenté par l'OSC
JoyState joy_cmd;       // après deadzone/expo
JoyState joy_filt;      // après filtrage
OffsetSession offset_session;

// Offsets joystick (en steps)
volatile long pan_offset_steps = 0;
volatile long tilt_offset_steps = 0;
volatile long zoom_offset_steps = 0;
volatile long slide_offset_steps = 0;

// Offsets joystick latched (s'accumulent, persistent)
volatile long pan_offset_latched = 0;
volatile long tilt_offset_latched = 0;
volatile long zoom_offset_latched = 0;
volatile long slide_offset_latched = 0;

// Les variables globales sont maintenant dans le module Config

// Jog slide
float slide_jog_cmd = 0.0f;    // -1..+1

//==================== Fonctions du module ====================

/**
 * @brief Initialise le module joystick
 */
void initJoystick() {
  Serial.println("🎮 Initialisation du module joystick...");
  
  // Initialiser les structures
  joy_raw.pan = 0;
  joy_raw.tilt = 0;
  joy_raw.slide = 0;
  
  joy_cmd.pan = 0;
  joy_cmd.tilt = 0;
  joy_cmd.slide = 0;
  
  joy_filt.pan = 0;
  joy_filt.tilt = 0;
  joy_filt.slide = 0;
  
  // Initialiser les offsets
  pan_offset_steps = 0;
  tilt_offset_steps = 0;
  zoom_offset_steps = 0;
  slide_offset_steps = 0;
  pan_offset_latched = 0;
  tilt_offset_latched = 0;
  zoom_offset_latched = 0;
  slide_offset_latched = 0;
  
  // Initialiser la session d'offsets
  offset_session.pan0 = 0;
  offset_session.tilt0 = 0;
  offset_session.zoom0 = 0;
  offset_session.slide0 = 0;
  
  
  Serial.println("✅ Module joystick initialisé");
}

/**
 * @brief Met à jour le traitement du joystick
 */
void updateJoystick() {
  static uint32_t t0 = millis(); 
  uint32_t now = millis(); 
  float dt = (now - t0) * 0.001f; 
  if (dt <= 0) return; 
  t0 = now;
  
  // Appliquer deadzone et exposition
  joy_cmd.pan   = applyDeadzoneExpo(joy_raw.pan,  joy.deadzone, joy.expo);
  joy_cmd.tilt  = applyDeadzoneExpo(joy_raw.tilt, joy.deadzone, joy.expo);
  joy_cmd.slide = applyDeadzoneExpo(joy_raw.slide, joy.deadzone, joy.expo);

  // Appliquer filtrage et slew limiting
  joy_filt.pan   = slewLimit(joy_filt.pan,   iir1Pole(joy_filt.pan,   joy_cmd.pan,   joy.filt_hz, dt), joy.slew_per_s/(float)PAN_OFFSET_RANGE, dt);
  joy_filt.tilt  = slewLimit(joy_filt.tilt,  iir1Pole(joy_filt.tilt,  joy_cmd.tilt,  joy.filt_hz, dt), joy.slew_per_s/(float)TILT_OFFSET_RANGE, dt);
  joy_filt.slide = slewLimit(joy_filt.slide, iir1Pole(joy_filt.slide, joy_cmd.slide, joy.filt_hz, dt), joy.slew_per_s / SLIDE_JOG_SPEED, dt);

  // Intégration des offsets joystick (comportement "latched")
  if (isActive() || isSlideActive()) {
    // Vitesse d'empilement en steps/s à |joy|=1 (30% de la Vmax de l'axe)
    const float PAN_OFFSET_RATE  = cfg[0].max_speed * 1.0f;  // steps/s
    const float TILT_OFFSET_RATE = cfg[1].max_speed * 0.7f;  // steps/s
    const float ZOOM_OFFSET_RATE  = cfg[2].max_speed * 0.7f;  // steps/s
    const float SLIDE_OFFSET_RATE = cfg[3].max_speed * 1.0f;  // steps/s
    
    long d_pan  = lroundf(joy_filt.pan  * PAN_OFFSET_RATE  * dt);
    long d_tilt = lroundf(joy_filt.tilt * TILT_OFFSET_RATE * dt);
    long d_zoom  = lroundf(joy_filt.slide * ZOOM_OFFSET_RATE  * dt);
    long d_slide = lroundf(joy_filt.slide * SLIDE_OFFSET_RATE * dt);
    
    pan_offset_latched  = clampL(pan_offset_latched  + d_pan,  -PAN_OFFSET_RANGE,  +PAN_OFFSET_RANGE);
    tilt_offset_latched = clampL(tilt_offset_latched + d_tilt, -TILT_OFFSET_RANGE, +TILT_OFFSET_RANGE);
    zoom_offset_latched  = clampL(zoom_offset_latched  + d_zoom,  -ZOOM_OFFSET_RANGE,  +ZOOM_OFFSET_RANGE);
    slide_offset_latched = clampL(slide_offset_latched + d_slide, -SLIDE_OFFSET_RANGE, +SLIDE_OFFSET_RANGE);
  }
  
  // Publie les offsets utilisés par le planificateur
  pan_offset_steps  = pan_offset_latched;
  tilt_offset_steps = tilt_offset_latched;
  zoom_offset_steps  = zoom_offset_latched;
  slide_offset_steps = slide_offset_latched;
  slide_jog_cmd     = clampF(joy_filt.slide * joy.slide_speed, -1.f, +1.f);

  // --- Gestion du relâchement joystick : bake non destructif ---
  static bool wasMoving = false;
  bool isMoving = (fabsf(joy_filt.pan) > 0.01f ||
                   fabsf(joy_filt.tilt) > 0.01f ||
                   fabsf(joy_filt.zoom) > 0.01f ||
                   fabsf(joy_filt.slide) > 0.01f);

  if (wasMoving && !isMoving) {
      Serial.println("🍞 Bake non destructif des offsets dans la base du mouvement");
      bakeOffsetsIntoCurrentMove(
          pan_offset_latched, 
          tilt_offset_latched, 
          zoom_offset_latched, 
          slide_offset_latched
      );
      saveOffsetBaseline(); // garde cette nouvelle base comme référence
  }
  wasMoving = isMoving;
}

/**
 * @brief Applique deadzone et exposition à une valeur
 */
float applyDeadzoneExpo(float x, float dz, float expo) {
  x = clampF(x, -1.f, 1.f); 
  if (fabsf(x) < dz) return 0.f;
  
  float s = (x > 0) ? 1.f : -1.f;
  float y = (fabsf(x) - dz) / (1.f - dz);
  y = s * powf(fabsf(y), 1.f + expo);
  return clampF(y, -1.f, 1.f);
}

/**
 * @brief Applique un filtre IIR 1-pole
 */
float iir1Pole(float y, float x, float fc, float dt) {
  float alpha = 1.f - expf(-2.f * M_PI * fc * dt);
  return y + alpha * (x - y);
}

/**
 * @brief Applique une limitation de slew rate
 */
float slewLimit(float y, float x, float rate, float dt) {
  float d = x - y;
  float d_max = rate * dt;
  if (fabsf(d) <= d_max) return x;
  return y + (d > 0 ? d_max : -d_max);
}

/**
 * @brief Obtient les valeurs brutes du joystick
 */
void getRawJoystickValues(float &pan, float &tilt, float &slide) {
  pan = joy_raw.pan;
  tilt = joy_raw.tilt;
  slide = joy_raw.slide;
}

/**
 * @brief Obtient les valeurs filtrées du joystick
 */
void getFilteredJoystickValues(float &pan, float &tilt, float &slide) {
  pan = joy_filt.pan;
  tilt = joy_filt.tilt;
  slide = joy_filt.slide;
}

/**
 * @brief Définit les valeurs brutes du joystick (via OSC)
 */
void setRawJoystickValues(float pan, float tilt, float slide) {
  joy_raw.pan = clampF(pan, -1.f, +1.f);
  joy_raw.tilt = clampF(tilt, -1.f, +1.f);
  joy_raw.slide = clampF(slide, -1.f, +1.f);
}

/**
 * @brief Obtient les offsets actuels
 */
void getCurrentOffsets(long &pan, long &tilt) {
  pan = pan_offset_steps;
  tilt = tilt_offset_steps;
}

/**
 * @brief Obtient les offsets latched
 */
void getLatchedOffsets(long &pan, long &tilt, long &zoom, long &slide) {
  pan = pan_offset_latched;
  tilt = tilt_offset_latched;
  zoom = zoom_offset_latched;
  slide = slide_offset_latched;
}

/**
 * @brief Définit les offsets latched
 */
void setLatchedOffsets(long pan, long tilt, long zoom, long slide) {
  pan_offset_latched = clampL(pan, -PAN_OFFSET_RANGE, +PAN_OFFSET_RANGE);
  tilt_offset_latched = clampL(tilt, -TILT_OFFSET_RANGE, +TILT_OFFSET_RANGE);
  zoom_offset_latched = clampL(zoom, -ZOOM_OFFSET_RANGE, +ZOOM_OFFSET_RANGE);
  slide_offset_latched = clampL(slide, -SLIDE_OFFSET_RANGE, +SLIDE_OFFSET_RANGE);
}

/**
 * @brief Ajoute des offsets aux offsets latched
 */
void addLatchedOffsets(long pan, long tilt, long zoom, long slide) {
  pan_offset_latched = clampL(pan_offset_latched + pan, -PAN_OFFSET_RANGE, +PAN_OFFSET_RANGE);
  tilt_offset_latched = clampL(tilt_offset_latched + tilt, -TILT_OFFSET_RANGE, +TILT_OFFSET_RANGE);
  zoom_offset_latched = clampL(zoom_offset_latched + zoom, -ZOOM_OFFSET_RANGE, +ZOOM_OFFSET_RANGE);
  slide_offset_latched = clampL(slide_offset_latched + slide, -SLIDE_OFFSET_RANGE, +SLIDE_OFFSET_RANGE);
}

/**
 * @brief Remet à zéro les offsets latched
 */
void resetLatchedOffsets() {
  pan_offset_latched = 0;
  tilt_offset_latched = 0;
  zoom_offset_latched = 0;
  slide_offset_latched = 0;
}

/**
 * @brief Obtient la commande de jog slide
 */
float getSlideJogCommand() {
  return slide_jog_cmd;
}

/**
 * @brief Définit la commande de jog slide
 */
void setSlideJogCommand(float cmd) {
  slide_jog_cmd = clampF(cmd, -1.f, +1.f);
}

/**
 * @brief Obtient la configuration du joystick
 */
JoyCfg& getJoystickConfig() {
  return joy;
}

/**
 * @brief Définit la configuration du joystick
 */
void setJoystickConfig(const JoyCfg& config) {
  joy = config;
}

/**
 * @brief Obtient la politique d'annulation
 */

// Les fonctions getOffsetRanges, setOffsetRanges, getJogSpeeds, setJogSpeeds 
// sont maintenant dans le module Config

/**
 * @brief Vérifie si le joystick pan est actif
 */
bool isPanActive() {
  return fabsf(joy_filt.pan) > 0.001f;
}

/**
 * @brief Vérifie si le joystick tilt est actif
 */
bool isTiltActive() {
  return fabsf(joy_filt.tilt) > 0.001f;
}

/**
 * @brief Vérifie si le joystick slide est actif
 */
bool isSlideActive() {
  return fabsf(slide_jog_cmd) > 0.001f;
}

/**
 * @brief Obtient l'offset pan effectif selon le contexte
 */
long getEffectivePanOffset(bool recallPhase) {
  return recallPhase ? (pan_offset_latched - offset_session.pan0) : pan_offset_steps;
}

/**
 * @brief Obtient l'offset tilt effectif selon le contexte
 */
long getEffectiveTiltOffset(bool recallPhase) {
  return recallPhase ? (tilt_offset_latched - offset_session.tilt0) : tilt_offset_steps;
}

/**
 * @brief Obtient l'offset zoom effectif selon le contexte
 */
long getEffectiveZoomOffset(bool recallPhase) {
  return recallPhase ? (zoom_offset_latched - offset_session.zoom0) : zoom_offset_steps;
}

/**
 * @brief Obtient l'offset slide effectif selon le contexte
 */
long getEffectiveSlideOffset(bool recallPhase) {
  return recallPhase ? (slide_offset_latched - offset_session.slide0) : slide_offset_steps;
}

/**
 * @brief Sauvegarde la baseline d'offsets pour une session
 */
void saveOffsetBaseline() {
  offset_session.pan0 = pan_offset_latched;
  offset_session.tilt0 = tilt_offset_latched;
  offset_session.zoom0 = zoom_offset_latched;
  offset_session.slide0 = slide_offset_latched;
}

/**
 * @brief Restaure la baseline d'offsets pour une session
 */
void restoreOffsetBaseline() {
  pan_offset_latched = offset_session.pan0;
  tilt_offset_latched = offset_session.tilt0;
  zoom_offset_latched = offset_session.zoom0;
  slide_offset_latched = offset_session.slide0;
}
