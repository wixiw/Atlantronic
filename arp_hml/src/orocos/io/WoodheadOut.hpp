/*
 * WoodheadOutOut.hpp
 *
 *  Created on: 29 Apr. 2011
 *      Author: wla
 */


#ifndef WOODHEAD_OUT_HPP_
#define WOODHEAD_OUT_HPP_

//include orocos
#include "orocos/can/CanOpenNode.hpp"

using namespace arp_core;


namespace arp_hml
{

    class WoodheadOut : public CanOpenNode
    {
    public:
    	WoodheadOut(const std::string& name);

    	/**
    	 * Get CanFestival pointer
    	 */
    	virtual bool configureHook();

    	/**
    	 * Replace the normal updateHook as this device is a writter to be executed at the end of the loop
    	 */
    	virtual void updateLate();

    protected:
    	InputPort<bool> inBit1;
    	InputPort<bool> inBit2;
    	InputPort<bool> inBit3;
    	InputPort<bool> inBit4;
    	InputPort<bool> inBit5;
    	InputPort<bool> inBit6;
    	InputPort<bool> inBit7;
    	InputPort<bool> inBit8;

    	UNS8* m_outputs;

    	virtual bool checkInputsPorts();

    };

}

#endif
