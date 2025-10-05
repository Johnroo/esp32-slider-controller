/**
 * @file MotorControl.cpp
 * @brief Implémentation du module de contrôle des moteurs
 */

#include "MotorControl.h"

// Les pins sont maintenant définies dans le module Config

//==================== Configuration des moteurs ====================
MotorConfig cfg[NUM_MOTORS] = {
  // PAN
  {DEFAULT_PAN_MAX_SPEED, DEFAULT_PAN_ACCEL, DEFAULT_PAN_MIN_LIMIT, DEFAULT_PAN_MAX_LIMIT, DEFAULT_PAN_CURRENT, DEFAULT_PAN_MICROSTEPS, false, 100},
  // TILT
  {DEFAULT_TILT_MAX_SPEED, DEFAULT_TILT_ACCEL, DEFAULT_TILT_MIN_LIMIT, DEFAULT_TILT_MAX_LIMIT, DEFAULT_TILT_CURRENT, DEFAULT_TILT_MICROSTEPS, false, 100},
  // ZOOM
  {DEFAULT_ZOOM_MAX_SPEED, DEFAULT_ZOOM_ACCEL, DEFAULT_ZOOM_MIN_LIMIT, DEFAULT_ZOOM_MAX_LIMIT, DEFAULT_ZOOM_CURRENT, DEFAULT_ZOOM_MICROSTEPS, false, 100},
  // SLIDE
  {DEFAULT_SLIDE_MAX_SPEED, DEFAULT_SLIDE_ACCEL, DEFAULT_SLIDE_MIN_LIMIT, DEFAULT_SLIDE_MAX_LIMIT, DEFAULT_SLIDE_CURRENT, DEFAULT_SLIDE_MICROSTEPS, false, 100}
};

//==================== Objets moteurs ====================
FastAccelStepperEngine engine;
FastAccelStepper* steppers[NUM_MOTORS];

TMC2209Stepper driver_pan  (&Serial2, R_SENSE, ADDR_PAN);
TMC2209Stepper driver_tilt (&Serial2, R_SENSE, ADDR_TILT);
TMC2209Stepper driver_zoom (&Serial2, R_SENSE, ADDR_ZOOM);
TMC2209Stepper driver_slide(&Serial2, R_SENSE, ADDR_SLIDE);
TMC2209Stepper* drivers[NUM_MOTORS] = {&driver_pan, &driver_tilt, &driver_zoom, &driver_slide};

//==================== Variables de position ====================
volatile long panPos = 0;
volatile long tiltPos = 0;
volatile long zoomPos = 0;
volatile long slidePos = 0;

//==================== Fonctions d'initialisation ====================
void initMotors() {
  Serial.println("🔧 Initialisation des moteurs...");
  
  // Initialiser l'engine FastAccelStepper
  engine.init();
  
  // Configurer les drivers TMC
  setupDriversTMC();
  
  // Attacher les steppers
  for (int i = 0; i < NUM_MOTORS; i++) {
    steppers[i] = engine.stepperConnectToPin(STEP_PINS[i]);
    if (steppers[i]) {
      Serial.println("✅ Stepper " + String(i) + " connected to pin " + String(STEP_PINS[i]));
      steppers[i]->setDirectionPin(DIR_PINS[i]);
      steppers[i]->setEnablePin(ENABLE_PINS[i], true);   // true = active LOW pour TMC2209
      steppers[i]->setAutoEnable(false);                 // Garde les moteurs alimentés
      steppers[i]->setSpeedInHz(cfg[i].max_speed);
      steppers[i]->setAcceleration(cfg[i].max_accel);
      steppers[i]->enableOutputs();                      // Force l'activation maintenant
      
      // Activer les moteurs
      digitalWrite(ENABLE_PINS[i], LOW);
    } else {
      Serial.println("❌ Erreur connexion stepper " + String(i));
    }
  }
  
  Serial.println("✅ Initialisation moteurs terminée");
}

void setupDriversTMC() {
  Serial.println("🔧 Configuration des drivers TMC2209 (StealthChop optimisé)...");

  Serial2.begin(115200, SERIAL_8N1, UART_RX, UART_TX);
  delay(50);

  for (int i = 0; i < NUM_MOTORS; i++) {
    auto d = drivers[i];

    // --- Initialisation de base ---
    d->begin();
    d->toff(5);                        // Temps off minimal pour la commutation
    d->blank_time(24);                 // Temps de blank standard
    d->rms_current(cfg[i].current_ma); // Courant RMS selon config
    d->microsteps(cfg[i].microsteps);  // Microstepping selon config

    // --- Configuration StealthChop ---
    d->pwm_autoscale(true);            // Calibration automatique PWM
    d->en_spreadCycle(false);          // StealthChop activé (mode silencieux)
    d->intpol(true);                   // Interpolation interne à 256 microsteps

    // --- StallGuard désactivé par défaut ---
    d->SGTHRS(0);
    cfg[i].sgt = 0;

    // --- Log complet ---
    Serial.printf("✅ TMC2209 moteur %d: %s | µsteps=%d | I=%.1fmA | StealthChop ON | StallGuard OFF\n",
                  i,
                  (i == 0 ? "PAN" :
                   i == 1 ? "TILT" :
                   i == 2 ? "ZOOM" : "SLIDE"),
                  cfg[i].microsteps,
                  (float)cfg[i].current_ma);
  }

  Serial.println("🔇 Tous les moteurs configurés en StealthChop silencieux (pwm_autoscale, intpol activés)");
}

//==================== Fonctions de contrôle ====================
void moveMotorTo(int motor_id, long position) {
  if (motor_id >= 0 && motor_id < NUM_MOTORS && steppers[motor_id]) {
    // Appliquer les limites
    position = constrain(position, cfg[motor_id].min_limit, cfg[motor_id].max_limit);
    
    // Déplacer le moteur
    steppers[motor_id]->moveTo(position);
    
    Serial.printf("🎯 Moteur %d -> position %ld\n", motor_id, position);
  }
}

void setMotorSpeed(int motor_id, int speed) {
  if (motor_id >= 0 && motor_id < NUM_MOTORS && steppers[motor_id]) {
    steppers[motor_id]->setSpeedInHz(speed);
    Serial.printf("⚡ Moteur %d vitesse %d Hz\n", motor_id, speed);
  }
}

void updateMotorPositions() {
  // Mettre à jour les positions
  panPos = steppers[0]->getCurrentPosition();
  tiltPos = steppers[1]->getCurrentPosition();
  zoomPos = steppers[2]->getCurrentPosition();
  slidePos = steppers[3]->getCurrentPosition();
}

//==================== Fonctions utilitaires ====================
long getMotorPosition(int motor_id) {
  if (motor_id >= 0 && motor_id < NUM_MOTORS && steppers[motor_id]) {
    return steppers[motor_id]->getCurrentPosition();
  }
  return 0;
}

bool isMotorMoving(int motor_id) {
  if (motor_id >= 0 && motor_id < NUM_MOTORS && steppers[motor_id]) {
    return steppers[motor_id]->isRunning();
  }
  return false;
}

void emergencyStop() {
  Serial.println("🚨 ARRÊT D'URGENCE - Arrêt de tous les moteurs");
  
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (steppers[i]) {
      steppers[i]->forceStop();
      steppers[i]->setCurrentPosition(0);  // Reset position
    }
  }
  
  // Reset des positions
  panPos = tiltPos = zoomPos = slidePos = 0;
}