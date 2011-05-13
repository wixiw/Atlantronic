/*
 * TestJoystick.hpp
 *
 *  Created on: 13 May 2011
 *      Author: wla
 */


#ifndef TestJoystick_HPP_
#define TestJoystick_HPP_

#include "orocos/taskcontexts/HmlTaskContext.hpp"

using namespace arp_core;


namespace arp_hml
{

    class TestJoystick : public HmlTaskContext
    {
    public:
    	TestJoystick(const std::string& name);
    	bool configureHook();
    	bool startHook();
    	void updateHook();

    	InputPort<bool> inButton;
    	InputPort<double> inAxe;

    protected:


    };

}

#endif
