/*
 * WoodheadSimul.hpp
 *
 *  Created on: 09 May 2012
 *      Author: wla
 */


#ifndef WOODHEAD_SIMUL_HPP_
#define WOODHEAD_SIMUL_HPP_

//include orocos
#include "orocos/taskcontexts/HmlTaskContext.hpp"

namespace arp_hml
{

    class WoodheadSimul : public HmlTaskContext
    {
    public:
        WoodheadSimul(const std::string& name);
    	void updateHook();

    protected:
    	OutputPort<bool> outBit1;
    	OutputPort<bool> outBit2;
    	OutputPort<bool> outBit3;
    	OutputPort<bool> outBit4;
    	OutputPort<bool> outBit5;
    	OutputPort<bool> outBit6;
    	OutputPort<bool> outBit7;
    	OutputPort<bool> outBit8;

    	OutputPort<bool> outConnected;

        InputPort<bool> inBit1;
        InputPort<bool> inBit2;
        InputPort<bool> inBit3;
        InputPort<bool> inBit4;
        InputPort<bool> inBit5;
        InputPort<bool> inBit6;
        InputPort<bool> inBit7;
        InputPort<bool> inBit8;
    };

}

#endif
