/*
 * KinematicFilter.hpp
 *
 *  Created on: Apr 5, 2012
 *      Author: Romain Moulin
 */

#ifndef KINEMATICFILTER_HPP_
#define KINEMATICFILTER_HPP_

#include <math/core>
#include <models/core>

namespace arp_ods
{

class KinematicFilter
{
    public:
        KinematicFilter();
        virtual ~KinematicFilter();

        /**
         * Filtre un twist pour obtenir un twist acceptable.\n
         * Convertit un Twist desire en un twist qui respecte les contraintes physiques.\n
         * tous les twists sont des "Twist du repère de référence du chassis par rapport au sol projeté et réduit dans le repère de référence du chassis"
         * Les twist doivent être similaire (même mobile/même base projeté réduit dans le même repère)
         * Le cas du twist à norme de translation nul est mis à part car les angles tourelles sont indéfinis.
         * Seul le cas départ de Twist(0,0,X) vres Twist(X,X,X) pose problème. Vu qu'il correspond à une reconfiguration
         * de tourelle avant de partir on laissera à la couche supérieur le soin de couper les tractions et le twist ne sera pas filtré
         * @param desiredTwist :    [in] le twist desire
         * @param currentTwist :    [in] le twist actuel du robot
         * @param currentMS :       [in] l'état courant des moteurs (en provenance de HML)
         * @param params :          [in] paramètres géométriques du robot
         * @param dt :              [in] delay between now and last computation.
         * @param minStartSpeed:    [in] minimum de vitesse mesuré. Si on mesure moins, on considère qu'on roule à cette vitesse
         * @param acceptableTwist : [out]le resultat. Un twist qui essaie de se rapprocher du twist desire tout en respectant les contraintes physiques.
         * @param quality :         [out] give us the closeness to desiredTwist of acceptableTwist in [0,1]. 0 mean we stay on last Twists
         * @return : true if computation succeed, false otherwise (param inconsistent for instance)
         */
        static bool filterTwist(const arp_math::Twist2D & desiredTwist,
                arp_math::Twist2D currentTwist,
                arp_model::MotorState currentMS,
                const arp_model::UbiquityParams & params,
                const double & dt,
                const double & minStartSpeed,
                arp_math::Twist2D & acceptableTwist,
                double & quality);

        /**
         * Check if the current motor state command is reachable
         * @param desiredState : the stat we try to reach already containing the solution for angle.
         * @param measuredState : the current motor state.
         * @param params : geometrical parameters required to use UbiquityKinematics models
         * @return a percentage between 0 and 1 which give a rates to apply on the desiredTwist commanded
         */
        static bool isMotorStateReachable(const arp_model::MotorState & desiredMS,
                                        const arp_model::MotorState & measuredMS,
                                        const arp_model::UbiquityParams & params,
                                        const double & dt);
};

}
#endif /* KINEMATICFILTER_HPP_ */
