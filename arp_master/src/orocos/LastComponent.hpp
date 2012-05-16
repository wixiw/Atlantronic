/*
 * LastComponent.hpp
 *
 *  Created on: Apr 6, 2012
 *      Author: ard
 */

#ifndef LASTCOMPONENT_HPP_
#define LASTCOMPONENT_HPP_

#include "orocos/MasterTaskContext.hpp"
#include <std_msgs/Bool.h>

namespace arp_master
{

class LastComponent: public MasterTaskContext
{
    public:
        LastComponent(const std::string name);
        ~LastComponent();
        bool startHook();
        void updateHook();

    protected:
        RTT::OutputPort<std_msgs::Bool> outDeployed;
};

} /* namespace arp_core */
#endif /* LASTCOMPONENT_HPP_ */
