/**
 * @file MotorControl.cpp
 * @brief Implémentation du module de contrôle des moteurs
 */

#include "MotorControl.h"

//==================== Pins Hardware ====================
// Pins STEP/DIR/EN
const int STEP_PINS[NUM_MOTORS]    = {18, 21, 23, 26};
const int DIR_PINS[NUM_MOTORS]     = {19, 22, 25, 27};
const int ENABLE_PINS[NUM_MOTORS]  = {13, 14, 32, 33};

//==================== Configuration des moteurs ====================
MotorConfig cfg[NUM_MOTORS] = {
  // PAN: {-27106, 27106, 1200, 16, 20000, 12000, 0, false, false, true}
  {20000, 12000, -27106, 27106, 1200, 16, true, 100},
  // TILT: {-2439, 2439, 1200, 16, 20000, 12000, 0, false, false, true}
  {20000, 12000, -2439, 2439, 1200, 16, true, 100},
  // ZOOM: {-20000, 20000, 400, 16, 20000, 8000, 0, false, false, true}
  {20000, 8000, -20000, 20000, 400, 16, true, 100},
  // SLIDE: {-20000, 20000, 1800, 8, 10000, 12000, 0, false, false, true}
  {10000, 12000, -20000, 20000, 1800, 8, true, 100}
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
  Serial.println("🔧 Configuration des drivers TMC2209...");
  
  Serial2.begin(115200, SERIAL_8N1, UART_RX, UART_TX);
  delay(50);
  
  for (int i = 0; i < NUM_MOTORS; i++) {
    auto d = drivers[i];
    
    // Configuration de base
    d->begin();
    d->toff(5);                        // Time off
    d->blank_time(24);                 // Blank time
    d->rms_current(cfg[i].current_ma); // Courant RMS original
    d->microsteps(cfg[i].microsteps);  // Microsteps original
    d->pwm_autoscale(true);            // Pour StealthChop (crucial pour le courant RMS)
    d->en_spreadCycle(cfg[i].spreadcycle);
    d->SGTHRS(cfg[i].sgt);             // StallGuard threshold
    // d->coolstep_en(cfg[i].coolstep);  // Pas disponible sur TMC2209
    // d->stallguard(cfg[i].stallguard);  // Pas disponible sur TMC2209
  }
  
  Serial.println("✅ Configuration TMC2209 terminée");
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