#ifndef COORDINATOR_H
#define COORDINATOR_H

/**
 * Coordinator - Orchestrateur des mouvements synchronisés
 * 
 * Ce module gère la coordination des différents types de mouvements :
 * - Interpolation automatique multi-presets
 * - Suivi coordonné slide-pan/tilt
 * - Jog direct des axes
 * - Mouvements synchronisés
 */
class Coordinator {
public:
    /**
     * Fonction principale de coordination des mouvements
     * À appeler dans la boucle principale
     */
    static void coordinatorTick();
};

#endif // COORDINATOR_H
