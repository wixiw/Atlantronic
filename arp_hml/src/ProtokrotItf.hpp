/*
 * ProtokrotItf.hpp
 *
 *  Created on: 10 april 2011
 *      Author: wla
 */

#ifndef PROTOKROTITF_HPP_
#define PROTOKROTITF_HPP_

#include <taskcontexts/ARDTaskContext.hpp>
#include <arp_core/DifferentialCommand.h>
#include <arp_core/Odo.h>
#include <arp_core/Start.h>
#include <arp_core/StartColor.h>

using namespace arp_core;

namespace arp_hml
{

    class ProtokrotItf: public ARDTaskContext
    {
    public:
    	ProtokrotItf(const std::string& name);
        void updateHook();

        DifferentialCommand attrCurrentCmd;
        Odo attrOdometers;


        InputPort<DifferentialCommand> inDifferentialCmd;
        OutputPort<Odo> outOdometryMeasures;
        OutputPort<Start> outIoStart;
        OutputPort<StartColor> outIoColorSwitch;
        OutputPort<bool> outEmergencyStop;
    };

}

#endif
