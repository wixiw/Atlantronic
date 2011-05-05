/*
 * Woodhead.hpp
 *
 *  Created on: 29 Apr. 2011
 *      Author: wla
 */


#ifndef WOODHEAD_HPP_
#define WOODHEAD_HPP_

//include orocos
#include "hml/can/CanOpenNode.hpp"

using namespace arp_core;


namespace arp_hml
{

    class Woodhead : public CanOpenNode
    {
    public:
    	Woodhead(const std::string& name);
    	bool configureHook();
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

    	UNS8* m_outputs;

    };

}

#endif
