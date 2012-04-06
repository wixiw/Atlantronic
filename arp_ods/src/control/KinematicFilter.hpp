/*
 * KinematicFilter.hpp
 *
 *  Created on: Apr 5, 2012
 *      Author: Romain Moulin
 */

#ifndef KINEMATICFILTER_HPP_
#define KINEMATICFILTER_HPP_

#include <math/core>
#include <models/UbiquityParams.hpp>
#include <models/UbiquityKinematics.hpp>


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
         * @param desTwist : le twist desire
         * @param currentTwist : le twist actuel du robot
         * @param turretCurrentState : le twist actuel du robot
         * @param acceptableTwist : le resultat. Un twist qui essaie de se rapprocher du twist desire tout en respectant les contraintes physiques.
         * @param params : paramètres géométriques du robot
         * @return : true if computation succeed, false otherwise (param inconsistent for instance)
         */
        static bool filterTwist(const arp_math::Twist2D & desTwist, const arp_math::Twist2D & currentTwist, const arp_model::TurretState & turretCurrentState,  arp_math::Twist2D& acceptableTwist, const arp_model::UbiquityParams & params);

    protected:
        /*
         * transport of the twist to what is nice for the filter: twist at center of gravity
         */
        static void transportToCog(const arp_math::Twist2D & refTwist, arp_math::Twist2D& cogTwist, const arp_model::UbiquityParams & params);
        /*
         * get back the twist from centre of gravity to the usual referential
         */
        static void transportToRef(const arp_math::Twist2D & cogTwist, arp_math::Twist2D& refTwist, const arp_model::UbiquityParams & params);
        /**
         * Non Holonomy handling: if the robot is asked a twist with a CIR that it will not reached, then it's no use to let him go fast
         */
        static void filterForNonholonomy(const arp_math::Twist2D & inputTwist, const arp_math::Twist2D & currentTwist, const arp_model::TurretState & turretCurrentState, arp_math::Twist2D& outputTwist, const arp_model::UbiquityParams & params);

        /**
         * Handling of the hardware contraints:
         * we known that the current twist would be an acceptable solution.
         * so we choose a Twist that is something between the desired twist and the current twist
         */
        static void filterForConstraints(const arp_math::Twist2D & inputTwist, const arp_math::Twist2D & currentTwist, arp_math::Twist2D& outputTwist, const arp_model::UbiquityParams & params);
};

}
#endif /* KINEMATICFILTER_HPP_ */
