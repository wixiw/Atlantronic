/*
 * DefaultJoystick.hpp
 *
 *  Created on: 29/09/10
 *      Author: wla
 */

#ifndef DEFAULT_JOYSTICK_HPP_
#define DEFAULT_JOYSTICK_HPP_

#include "joystick/Joystick.hpp"

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/extras/Properties.hpp>
#include <rtt/OperationCaller.hpp>

namespace arp_hml
{
    using namespace RTT;

    class DefaultJoystick: public Joystick
    {
    public:
        DefaultJoystick(const std::string& name);
        ~DefaultJoystick();

    protected:
        virtual bool checkIdentity();
        virtual void buttonEvent( struct js_event js );
        virtual void axisEvent( struct js_event js );
        virtual void initEvent( struct js_event js );
    };
}

#endif /* DEFAULT_JOYSTICK_HPP_ */
