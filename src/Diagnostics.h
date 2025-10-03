#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H

/**
 * Diagnostics - Module de diagnostic et logging périodique
 * 
 * Ce module gère les logs périodiques du système :
 * - Affichage temps + jog + positions moteurs (500ms)
 * - Vérification OSC listening + état des drivers TMC (5s)
 */
class Diagnostics {
public:
    /**
     * Initialise le module de diagnostics
     */
    static void initDiagnostics();
    
    /**
     * Met à jour les diagnostics et logs périodiques
     * À appeler dans la boucle principale
     */
    static void updateDiagnostics();
};

#endif // DIAGNOSTICS_H