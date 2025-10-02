/**
 * @file MotorControl.cpp
 * @brief Implémentation du module de contrôle des moteurs
 */

#include "MotorControl.h"

//==================== Variables globales moteurs ====================
FastAccelStepperEngine engine;
FastAccelStepper* steppers[NUM_MOTORS];

// Drivers TMC2209
TMC2209Stepper driver_pan  (&Serial2, R_SENSE, ADDR_PAN);
TMC2209Stepper driver_tilt (&Serial2, R_SENSE, ADDR_TILT);
TMC2209Stepper driver_zoom (&Serial2, R_SENSE, ADDR_ZOOM);
TMC2209Stepper driver_slide(&Serial2, R_SENSE, ADDR_SLIDE);
TMC2209Stepper* drivers[NUM_MOTORS] = {&driver_pan,&driver_tilt,&driver_zoom,&driver_slide};

// Variables globales de position (pour compatibilité avec le code existant)
volatile long panPos = 0;
volatile long tiltPos = 0;
volatile long zoomPos = 0;
volatile long slidePos = 0;

// Configuration des moteurs
MotorConfig cfg[NUM_MOTORS] = {
  // PAN
  {
    .current_ma = 1200,
    .microsteps = 16,
    .spreadcycle = false,
    .sgt = 100,
    .max_speed = 8000,
    .max_accel = 10000,
    .min_limit = -20000,
    .max_limit = 20000
  },
  // TILT
  {
    .current_ma = 1200,
    .microsteps = 16,
    .spreadcycle = false,
    .sgt = 100,
    .max_speed = 8000,
    .max_accel = 10000,
    .min_limit = -20000,
    .max_limit = 20000
  },
  // ZOOM
  {
    .current_ma = 800,
    .microsteps = 16,
    .spreadcycle = false,
    .sgt = 100,
    .max_speed = 6000,
    .max_accel = 8000,
    .min_limit = -15000,
    .max_limit = 15000
  },
  // SLIDE
  {
    .current_ma = 1600,
    .microsteps = 16,
    .spreadcycle = false,
    .sgt = 120,
    .max_speed = 10000,
    .max_accel = 12000,
    .min_limit = -50000,
    .max_limit = 50000
  }
};

// Pins moteurs
const int STEP_PINS[NUM_MOTORS] = {14, 27, 26, 25};
const int DIR_PINS[NUM_MOTORS] = {12, 13, 32, 33};
const int ENABLE_PINS[NUM_MOTORS] = {15, 4, 2, 18};

//==================== Fonctions d'initialisation ====================
void initMotors() {
  Serial.println("🚀 Initialisation des moteurs...");
  
  // Initialiser l'engine
  engine.init();
  Serial.println("✅ FastAccelStepper engine initialisé");
  
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
  
  Serial.println("🎯 Moteurs initialisés avec succès!");
}

void setupDriversTMC() {
  Serial.println("🔧 Configuration des drivers TMC2209...");
  
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
    
    Serial.println("✅ Driver " + String(i) + " configuré (courant: " + String(cfg[i].current_ma) + "mA)");
  }
  
  Serial.println("✅ Drivers TMC2209 configurés");
}

//==================== Fonctions utilitaires moteurs ====================
bool isMotorRunning(int motorIndex) {
  if (motorIndex < 0 || motorIndex >= NUM_MOTORS || !steppers[motorIndex]) {
    return false;
  }
  return steppers[motorIndex]->isRunning();
}

void stopMotor(int motorIndex) {
  if (motorIndex < 0 || motorIndex >= NUM_MOTORS || !steppers[motorIndex]) {
    return;
  }
  steppers[motorIndex]->forceStop();
}

void stopAllMotors() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    stopMotor(i);
  }
}

long getMotorPosition(int motorIndex) {
  if (motorIndex < 0 || motorIndex >= NUM_MOTORS || !steppers[motorIndex]) {
    return 0;
  }
  return steppers[motorIndex]->getCurrentPosition();
}

long getMotorTarget(int motorIndex) {
  if (motorIndex < 0 || motorIndex >= NUM_MOTORS || !steppers[motorIndex]) {
    return 0;
  }
  return steppers[motorIndex]->targetPos();
}

void setMotorSpeed(int motorIndex, int speed) {
  if (motorIndex < 0 || motorIndex >= NUM_MOTORS || !steppers[motorIndex]) {
    return;
  }
  steppers[motorIndex]->setSpeedInHz(speed);
}

void setMotorAcceleration(int motorIndex, int acceleration) {
  if (motorIndex < 0 || motorIndex >= NUM_MOTORS || !steppers[motorIndex]) {
    return;
  }
  steppers[motorIndex]->setAcceleration(acceleration);
}

void moveMotorTo(int motorIndex, long target) {
  if (motorIndex < 0 || motorIndex >= NUM_MOTORS || !steppers[motorIndex]) {
    return;
  }
  steppers[motorIndex]->moveTo(target);
}

void moveMotor(int motorIndex, long steps) {
  if (motorIndex < 0 || motorIndex >= NUM_MOTORS || !steppers[motorIndex]) {
    return;
  }
  steppers[motorIndex]->move(steps);
}

//==================== Fonctions de position ====================
void updateMotorPositions() {
  panPos = steppers[0]->getCurrentPosition();
  tiltPos = steppers[1]->getCurrentPosition();
  zoomPos = steppers[2]->getCurrentPosition();
  slidePos = steppers[3]->getCurrentPosition();
}
