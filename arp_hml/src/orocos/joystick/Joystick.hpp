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

#include "time/ArdTime.hpp"

namespace arp_hml
{
	/**
	 * \ingroup arp_hml
	 *
	 * \class Joystick
	 *
	 * \brief Generic USB Joystick hardware interface
	 *
	 *  This is a generic class to handle an usb joystick in Orocos.
	 *  You should not use this class directly but inheritate it and redefine
	 *  the checkIdentity, buttonEvent, axisEvent, initEvent method, so that
	 *  they fits to your hardware.
	 *
	 *  You can see orocos/joystick/DefaultJoystick.cpp which is a simple use case.
	 *  The first time you use your joystick you should deploy a DefaultJoystick
	 *  component a play with all your buttons and axis in order to read in the
	 *  task browser the id to use in your custom buttonEvent, axisEvent methods.
	 */
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

        /**
         * Is true when event have been received from the driver for more than propMaxNoEventDelay
         * It prevents from lost of communication at full speed when joystick is suddenly deconnected.
         */
        bool isJoystickAlive();

        /** Name of the joystick drive file usually /dev/input/js0 */
        std::string propDevName;
        /** Minimal driver version on which this code is working */
        int propMinimalDriverVersion;
        /** Maximal delay when no event is received to put the component in safe mode in the isJoystickAlive method */
        arp_time::ArdTimeDelta propMaxNoEventDelay;

        /** Returns the joystick name as reading from USB manufacturer information */
        std::string getJoystickName();

        /** Is true when the component thinks the joystick is connected. When true, this information is not always accurate
         * (that's why there is an isJoystickAlive method), but you can be sure that if it is false, the joystick is badly connected.
         */
        bool attrIsConnected;

        /** Is true when the getJoystickName returns an awaited name in the checkIdentity method */
        bool attrIsIdentityOk;

    protected:
        /**
         * Compares getJoystickName with an hardcoded awaited name
         */
        virtual bool checkIdentity() = 0;

        /**
         * When you make a new Joystick, you surely want to override this fonction. It is automatically
         * called by Joystick::updateHook() when the driver has received a button event
         */
        virtual void buttonEvent( struct js_event js ) = 0;
        /**
         * When you make a new Joystick, you surely want to override this fonction. It is automatically
         * called by Joystick::updateHook() when the driver has received a joystick event
         */
        virtual void axisEvent( struct js_event js ) = 0;
        /**
         * When you make a new Joystick, you surely want to override this fonction. It is automatically
         * called by Joystick::updateHook() when the driver has received an init event
         * (I have never seen this event but anyway, it safer to put in there default values of button and axis)
         */
        virtual void initEvent( struct js_event js );


    private:
        /** Pointer on joystick's driver file propDevName*/
        int m_fd;

        /** Release the m_fd file descriptor */
        bool releaseJoystick();
        /** Take the m_fd file descriptor */
        bool takeJoystick();
        /** Is true when the propDevName file is present in the filesystem */
        bool isJoystickConnected();
        /** check if the linux driver version is more than propMinimalDriverVersion */
        bool checkDriverVersion();
        /** Records the last event date for isJoystickAlive computation */
        arp_time::ArdAbsoluteTime lastEventTime;


    };
}

#endif /* JOYSTICK_HPP_ */
