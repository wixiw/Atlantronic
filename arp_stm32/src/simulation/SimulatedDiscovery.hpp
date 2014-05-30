/*
 * SimulatedDiscovery.hpp
 *
 *  Created on: 22 April 2014
 *      Author: wla
 */

#ifndef SIMULATEDDISCOVERY_HPP_
#define SIMULATEDDISCOVERY_HPP_

#include "components/discovery/Discovery.hpp"
#include "linux/tools/qemu.h"
#include <std_msgs/Bool.h>

namespace arp_stm32
{

class SimulatedDiscovery: public arp_stm32::Discovery
{
    public:
        SimulatedDiscovery(const std::string& name);
        ~SimulatedDiscovery();

        bool configureHook();
        void updateHook();
        bool breakUpdateHook();

        static Qemu m_qemu;
        virtual bool ooReset();

    protected:
        void createOrocosInterface();

        std::string propStm32ExecutableName;

        //Ports used to drive qemu IOs
        RTT::InputPort<std_msgs::Bool> inFakeUserButton1;
        RTT::InputPort<std_msgs::Bool> inFakeUserButton2;
        RTT::InputPort<std_msgs::Bool> inFakeStart;

        bool attrFakeStart;
};

} /* namespace arp_stm32 */
#endif /* SIMULATEDDISCOVERY_HPP_ */
