/*
 * Tor.hpp
 *
 *  Created on: 20 April 2014
 *      Author: wla
 */

#ifndef TOR_HPP_
#define TOR_HPP_

#include "components/taskcontexts/Stm32TaskContext.hpp"
#include "linux/tools/robot_interface.h"
#include "ros/ros.h"

namespace arp_stm32
{

class Tor: public Stm32TaskContext
{
    public:
        Tor(const std::string& name);

/****************************************************************
 * Interface Orocos
 ****************************************************************/

        bool configureHook();
        void updateHook();

        RTT::OutputPort<bool> outObjectPresent;

/****************************************************************
 * Interface ROS
 ****************************************************************/

    protected:
        void createOrocosInterface();

        /** Read concretely to robot_interface. the mutex has to be previously taken. Returns the value */
        bool getGpioValue();

        RobotInterface& m_robotItf;

        bool attrSignal;

        bool propInvertSignal;
        int propTorId;
};

} /* namespace arp_stm32 */
#endif /* TOR_HPP_ */
