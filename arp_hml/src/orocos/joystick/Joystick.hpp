/*
 * Joystick.hpp
 *
 *  Created on: 29/09/10
 *      Author: wla
 */

#ifndef JOYSTICK_HPP_
#define JOYSTICK_HPP_

//include orocos
#include "orocos/taskcontexts/HmlTaskContext.hpp"

//includes pour le joystick
#include <linux/input.h>
#include <linux/joystick.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

using namespace RTT;
using namespace arp_core;

namespace arp_hml
{
    class Joystick: public HmlTaskContext
    {
    public:
        Joystick(const std::string& name);
        ~Joystick();
        bool configureHook();
        bool startHook();
        void updateHook();
        void errorHook();
        void stopHook();

        string propDevName;
        int propMinimalDriverVersion;

        string getJoystickName();

        bool attrIsConnected;
        bool attrIsIdentityOk;

    protected:
        virtual bool checkIdentity() = 0;
        virtual void buttonEvent( struct js_event js ) = 0;
        virtual void axisEvent( struct js_event js ) = 0;
        virtual void initEvent( struct js_event js );


    private:
        int m_fd;

        bool releaseJoystick();
        bool takeJoystick();
        bool isJoystickConnected();
        bool checkDriverVersion();



    };
}

#endif /* JOYSTICK_HPP_ */
