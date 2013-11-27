/*
 * WoodheadIn.hpp
 *
 *  Created on: 29 Apr. 2011
 *      Author: wla
 */


#ifndef WOODHEAD_IN_HPP_
#define WOODHEAD_IN_HPP_

//include orocos
#include "orocos/can/CanOpenNode.hpp"

using namespace arp_core;


namespace arp_hml
{

    class WoodheadIn : public CanOpenNode
    {
    public:
    	WoodheadIn(const std::string& name);
    protected:
    	OutputPort<bool> outBit1;
    	OutputPort<bool> outBit2;
    	OutputPort<bool> outBit3;
    	OutputPort<bool> outBit4;
    	OutputPort<bool> outBit5;
    	OutputPort<bool> outBit6;
    	OutputPort<bool> outBit7;
    	OutputPort<bool> outBit8;

    	UNS8* m_inputs;

        bool configureHook();
        void updateHook();
    	virtual bool checkInputsPorts();

    };

}

#endif
