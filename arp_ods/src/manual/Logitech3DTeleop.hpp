/*
 * Logitech3DTeleop.hpp
 *
 *  Created on: 01 may 2011
 *      Author: wla
 */

#ifndef Logitech3DTeleop_HPP
#define Logitech3DTeleop_HPP

//include orocos
#include <taskcontexts/ARDTaskContext.hpp>
#include <arp_core/Velocity.h>
#include <std_msgs/Bool.h>

using namespace arp_core;
using namespace std_msgs;

namespace arp_ods
{

    class Logitech3DTeleop: public ARDTaskContext
    {
    public:
        Logitech3DTeleop(const std::string& name);
        ~Logitech3DTeleop();

        bool configureHook();
        void updateHook();

    protected:
        Velocity attrVelocityCommand;

        /** Axe X du manche : dirigé de haut en bas sur la manette entre [-1;1] */
        InputPort<double> inY;
        /** Axe z du manche : rotation horaire entre [-1;1] */
        InputPort<double> inZ;
        InputPort<bool> inDeadMan;
        InputPort<Bool> inPower;
        OutputPort<Velocity> outVelocityCmd;

        OperationCaller<bool(bool, double)> m_ooSetPower;

        double propLongGain;
        double propRotGain;
    };

}

#endif /* PCM3362_HPP_ */
