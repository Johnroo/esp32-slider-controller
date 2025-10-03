#ifndef OSCMANAGER_H
#define OSCMANAGER_H

#include <WiFiUdp.h>
#include <OSCMessage.h>
#include <OSCBundle.h>
#include <OSCData.h>

// Déclarations externes pour les variables et fonctions définies dans main.cpp
enum class SlideRecallPolicy : uint8_t { KEEP_AB = 0, GOTO_THEN_RESUME = 1 };
struct RecallPolicy {
  SlideRecallPolicy slide = SlideRecallPolicy::KEEP_AB;
};
extern struct RecallPolicy recallPolicy;
extern void saveActiveBank();
extern void home_slide();

/**
 * OSCManager - Gestionnaire des messages OSC
 * 
 * Ce module encapsule toute la logique OSC du contrôleur ESP32.
 * Il gère la réception et le traitement de tous les messages OSC
 * pour le contrôle des moteurs, presets, interpolation, etc.
 */
class OSCManager {
public:
    /**
     * Initialise le serveur OSC sur le port configuré
     * @return true si l'initialisation a réussi
     */
    static bool initOSC();
    
    /**
     * Traite les messages OSC entrants
     * À appeler dans la boucle principale
     */
    static void processOSC();

private:
    static WiFiUDP udp;
    static OSCErrorCode error;
    
    // Fonctions de traitement des routes OSC
    static void handleJoystickRoutes(OSCMessage &msg);
    static void handleAxisRoutes(OSCMessage &msg);
    static void handlePresetRoutes(OSCMessage &msg);
    static void handleInterpolationRoutes(OSCMessage &msg);
    static void handleBankRoutes(OSCMessage &msg);
    static void handleMotorConfigRoutes(OSCMessage &msg);
    static void handleHomingRoutes(OSCMessage &msg);
    static void handleOffsetRoutes(OSCMessage &msg);
    static void handleConfigRoutes(OSCMessage &msg);
};

#endif // OSCMANAGER_H
