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

namespace arp_stm32
{

class SimulatedDiscovery: public Discovery
{
    public:
        SimulatedDiscovery(const std::string& name);

        bool configureHook();

    protected:
        void createOrocosInterface();

        Qemu m_qemu;
        std::string propStm32ExecutableName;
};

} /* namespace arp_stm32 */
#endif /* SIMULATEDDISCOVERY_HPP_ */
