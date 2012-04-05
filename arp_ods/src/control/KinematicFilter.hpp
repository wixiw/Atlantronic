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

using namespace arp_core;
using namespace arp_math;

namespace arp_ods
{

class KinematicFilter
{
    public:
        KinematicFilter();
        virtual ~KinematicFilter();

        /**
         * Filtre un twist pour obtenir un twist acceptable
         * Convertit un Twist desire en un twist qui respecte les contraintes physique
         * tous les twists sont " base roulante par rapport au sol exprimé dans le repère XXX, réduit au centre du chassis "
         * @param desTwist : le twist desire
         * @param currentTwist : le twist actuel du robot
         * @param acceptableTwist : le resultat. Un twist qui essaie de se rapprocher du twist desire tout en respectant les contraintes physiques.
         * @param params : paramètres géométriques du robot
         * @return : true if computation succeed, false otherwise (param inconsistent for instance)
         */
        static bool filterTwist(Twist2D const  desTwist, Twist2D const  currentTwist, MotorCommands const motorsCurrentState,  Twist2D& acceptableTwist, UbiquityParams const params);

    protected:
        /*
         * transport of the twist to what is nice for the filter: twist at center of gravity
         */
        static void transportToCog(Twist2D const  refTwist, Twist2D& cogTwist, UbiquityParams const params);
        /*
         * get back the twist from centre of gravity to the usual referential
         */
        static void transportToRef(Twist2D const  cogTwist, Twist2D& refTwist, UbiquityParams const params);
        /**
         * Non Holonomy handling: if the robot is asked a twist with a CIR that it will not reached, then it's no use to let him go fast
         */
        static void filterForNonholonomy(Twist2D const  inputTwist, Twist2D const  currentTwist, MotorCommands const motorsCurrentState, Twist2D& outputTwist, UbiquityParams const params);

        /**
         * Handling of the hardware contraints:
         * we known that the current twist would be an acceptable solution.
         * so we choose a Twist that is something between the desired twist and the current twist
         */
        static void filterForConstraints(Twist2D const  inputTwist, Twist2D const  currentTwist, Twist2D& outputTwist, UbiquityParams const params);
};

}
#endif /* KINEMATICFILTER_HPP_ */
