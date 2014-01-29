/*
 * Discovery.hpp
 *
 *  Created on: 26 Jan 2014
 *      Author: wla
 */

#ifndef DISCOVERY_HPP_
#define DISCOVERY_HPP_

#include "Stm32TaskContext.hpp"
#include "linux/tools/robot_interface.h"

namespace arp_stm32
{

class Discovery: public Stm32TaskContext
{
    public:
        Discovery(const std::string& name);
        bool configureHook();
        void updateHook();

    protected:
        static void robotItfCallbackWrapper(void* arg);
        void robotItfUpdated();
        RobotInterface m_robotItf;
        void createOrocosInterface();


};

} /* namespace arp_rlu */
#endif /* DISCOVERY_HPP_ */
