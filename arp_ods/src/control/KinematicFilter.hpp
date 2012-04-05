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

class KinematicFilter
{
    public:
        KinematicFilter();
        virtual ~KinematicFilter();

        /**
         * Filtre d'un twist pour obtenir un twist acceptable
         * Convertit un Twist desire en un twist qui respecte les contraintes physique
         * tous les twists sont " base roulante par rapport au sol exprimé dans le repère XXX, réduit au centre du chassis "
         * @param desTwist : le twist desire
         * @param currentTwist : le twist actuel du robot
         * @param acceptableTwist : le resultat. Un twist qui essaie de se rapprocher du twist desire tout en respectant les contraintes physiques.
         * @param params : paramètres géométriques du robot
         * @return : true if computation succeed, false otherwise (param inconsistent for instance)
         */
        static bool filterTwist(Twist2D const  desTwist, Twist2D const  currentTwist, Twist2D& acceptableTwist, UbiquityParams const params);
    protected:
        static void refTwist2cogTwist(Twist2D const  refTwist, Twist2D& cogTwist, UbiquityParams const params);
        static void cogTwist2refTwist(Twist2D const  cogTwist, Twist2D& refTwist, UbiquityParams const params);
        static void filterForNonholonomy(Twist2D const  inputTwist, Twist2D const  currentTwist, Twist2D& outputTwist, UbiquityParams const params);
        static void filterForConstraints(Twist2D const  inputTwist, Twist2D const  currentTwist, Twist2D& outputTwist, UbiquityParams const params);
};

#endif /* KINEMATICFILTER_HPP_ */
