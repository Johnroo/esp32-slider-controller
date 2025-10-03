#include "Diagnostics.h"
#include "Config.h"
#include "MotorControl.h"
#include "Joystick.h"
#include "OSCManager.h"

void Diagnostics::initDiagnostics() {
    // Pas d'initialisation nécessaire pour le moment
}

void Diagnostics::updateDiagnostics() {
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