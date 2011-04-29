/*
 * Logitech3DTeleop.hpp
 *
 *  Created on: 2 nov. 2010
 *      Author: ard
 */

#ifndef Logitech3DTeleop_HPP
#define Logitech3DTeleop_HPP

//include orocos
#include <taskcontexts/ARDTaskContext.hpp>

using namespace arp_core;


namespace arp_master
{

    class Logitech3DTeleop : public ARDTaskContext
    {
    public:
    	Logitech3DTeleop(const std::string& name);
        ~Logitech3DTeleop();

        void updateHook();

    protected:
        /** Axe X du manche : dirigé de haut en bas sur la manette entre [-1;1] */
        InputPort<double> inY;
        /** Axe z du manche : rotation horaire entre [-1;1] */
        InputPort<double> inZ;
        InputPort<bool> inDeadMan;
        OutputPort<int> outLeftSpeed;
        OutputPort<int> outRightSpeed;

        double propLongGain;
        double propRotGain;
    };

}

#endif /* PCM3362_HPP_ */
