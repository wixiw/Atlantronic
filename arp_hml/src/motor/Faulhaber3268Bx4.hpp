/*
 * Faulhaber3268Bx4.hpp
 *
 *  Created on: 10 mars 2011
 *      Author: ard
 */

#ifndef FAULHABER3268BX4_HPP_
#define FAULHABER3268BX4_HPP_

#include "can/CanOpenNode.hpp"
#include "motor/ArdMotorItf.hpp"
#include "can/ard_can_types.hpp"

using namespace arp_core;

namespace arp_hml
{

    class Faulhaber3268Bx4 : public CanOpenNode, public ArdMotorItf
    {
    public:
        Faulhaber3268Bx4(const std::string& name);
        void updateHook();
        bool configureHook();
        void stopHook();

        bool setOperationMode(int operationMode);
        bool init();
        bool enableDrive();
        bool disableDrive();
        bool reset();
        bool getLimitSwitchStatus();
        bool startWatchdog();
        bool stopWatchdog();
        bool isInError();
        unsigned int getError();

        static const int F_CMD_DI = 0x08;
        static const int F_CMD_EN = 0x0F;
        static const int F_CMD_V = 0x93;

    protected:
        ArdDs402::enum_DS402_state attrState;

        InputPort<double> inSpeedCmd;

        OutputPort<double> outCommandedSpeed;
        OutputPort<double> outMeasuredPosition;
        OutputPort<double> outMeasuredCurrent;

        OutputPort<int> outLastSentCommand;
        OutputPort<double> outLastSentCommandParam;
        OutputPort<int> outLastSentCommandReturn;

        INTEGER32* m_measuredPosition;
        INTEGER16* m_measuredCurrent;
        UNS8* m_faulhaberCommand;
        UNS32* m_faulhaberCommandParameter;
        UNS8* m_faulhaberCommandReturn;
        UNS8* m_faulhaberCommandReturnCode;
        UNS32* m_faulhaberCommandReturnParameter;
        UNS16* m_ds402State;

        void ooSendSpeed( int speed);
        void ooReadSpeed();
        void ooEnableDrive(bool enable);

    };

}

#endif /* FAULHABER3268BX4_HPP_ */
