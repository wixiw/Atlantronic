/*
 * HmlItf.hpp
 *
 *  Created on: 10 april 2011
 *      Author: wla
 */

#ifndef HMLITF_HPP_
#define HMLITF_HPP_

#include <taskcontexts/ARDTaskContext.hpp>
#include <arp_master/DifferentialCommand.h>
#include <arp_master/Odo.h>

using namespace arp_core;
using namespace arp_master;

namespace arp_hml
{

    class HmlItf: public ARDTaskContext
    {
    public:
        HmlItf(const std::string& name);
        void updateHook();

        DifferentialCommand attrCurrentCmd;
        Odo attrOdometers;


        InputPort<DifferentialCommand> inDifferentialCmd;
        OutputPort<Odo> outOdometryMeasures;
    };

}

#endif
