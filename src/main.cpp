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

//==================== Configuration ====================
#define NUM_MOTORS 4

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
  {-10000, 10000, 800, 16, 20000, 8000, 0, false, true, false},
  // Tilt  
  {-10000, 10000, 800, 16, 20000, 8000, 0, false, true, false},
  // Zoom
  {-20000, 20000, 800, 16, 20000, 8000, 0, false, true, false},
  // Slide
  {-20000, 20000, 800, 16, 20000, 8000, 0, false, true, false}
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

//==================== Variables globales ====================
float jogCmd = 0.0f;
float panOffsetCmd = 0.0f;
float tiltOffsetCmd = 0.0f;

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
      msg.dispatch("/jog", [](OSCMessage &msg) {
        float value = msg.getFloat(0);
        jogCmd = constrain(value, -1.0f, 1.0f);
        Serial.println("Jog: " + String(jogCmd));
      });
      
      msg.dispatch("/pan", [](OSCMessage &msg) {
        float value = msg.getFloat(0);
        panOffsetCmd = constrain(value, -1.0f, 1.0f);
        Serial.println("Pan: " + String(panOffsetCmd));
      });
      
      msg.dispatch("/tilt", [](OSCMessage &msg) {
        float value = msg.getFloat(0);
        tiltOffsetCmd = constrain(value, -1.0f, 1.0f);
        Serial.println("Tilt: " + String(tiltOffsetCmd));
      });
      
      msg.dispatch("/axis_pan", [](OSCMessage &msg) {
        float value = msg.getFloat(0);
        long pos_val = (long)(value * (cfg[0].max_limit - cfg[0].min_limit) + cfg[0].min_limit);
        Serial.println("🔧 Moving Pan to: " + String(pos_val));
        steppers[0]->moveTo(pos_val);
        Serial.println("Axis Pan: " + String(value) + " -> " + String(pos_val));
        Serial.println("🔧 Pan stepper running: " + String(steppers[0]->isRunning()));
      });
      
      msg.dispatch("/axis_tilt", [](OSCMessage &msg) {
        float value = msg.getFloat(0);
        long pos_val = (long)(value * (cfg[1].max_limit - cfg[1].min_limit) + cfg[1].min_limit);
        Serial.println("🔧 Moving Tilt to: " + String(pos_val));
        steppers[1]->moveTo(pos_val);
        Serial.println("Axis Tilt: " + String(value) + " -> " + String(pos_val));
        Serial.println("🔧 Tilt stepper running: " + String(steppers[1]->isRunning()));
      });
      
      msg.dispatch("/axis_zoom", [](OSCMessage &msg) {
        float value = msg.getFloat(0);
        long pos_val = (long)(value * (cfg[2].max_limit - cfg[2].min_limit) + cfg[2].min_limit);
        Serial.println("🔧 Moving Zoom to: " + String(pos_val));
        steppers[2]->moveTo(pos_val);
        Serial.println("Axis Zoom: " + String(value) + " -> " + String(pos_val));
        Serial.println("🔧 Zoom stepper running: " + String(steppers[2]->isRunning()));
      });
      
      msg.dispatch("/axis_slide", [](OSCMessage &msg) {
        float value = msg.getFloat(0);
        long pos_val = (long)(value * (cfg[3].max_limit - cfg[3].min_limit) + cfg[3].min_limit);
        Serial.println("🔧 Moving Slide to: " + String(pos_val));
        steppers[3]->moveTo(pos_val);
        Serial.println("Axis Slide: " + String(value) + " -> " + String(pos_val));
        Serial.println("🔧 Slide stepper running: " + String(steppers[3]->isRunning()));
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
  
  webServer.on("/status", HTTP_GET, [](AsyncWebServerRequest *request){
    String status = "Pan: " + String(panPos) + " Tilt: " + String(tiltPos) + 
                   " Zoom: " + String(zoomPos) + " Slide: " + String(slidePos);
    request->send(200, "text/plain", status);
  });
  
  webServer.begin();
  Serial.println("🌐 Web server started");
}

//==================== Setup ====================
void setup() {
  Serial.begin(115200);
  delay(200);
  
  Serial.println("🚀 ESP32 Slider Controller Starting...");
  
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
}

//==================== Loop ====================
void loop() {
  ArduinoOTA.handle();
  processOSC();
  
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
    String s = "t=" + String(millis()/1000.0, 2) + " jog=" + String(jogCmd, 2) +
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