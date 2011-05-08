/*
 * Faulhaber3268Bx4.hpp
 *
 *  Created on: 10 mars 2011
 *      Author: ard
 */

#ifndef FAULHABER3268BX4_HPP_
#define FAULHABER3268BX4_HPP_

#include "hml/can/CanOpenNode.hpp"
#include "hml/motor/ArdMotorItf.hpp"
#include "hml/can/ard_can_types.hpp"

using namespace arp_core;
//TODO WLA mettre la doc
namespace arp_hml
{

    class Faulhaber3268Bx4 : public CanOpenNode, public ArdMotorItf
    {
    public:
        Faulhaber3268Bx4(const std::string& name);
        void updateHook();
        bool configureHook();
        void stopHook();

        bool init();
        void enableDrive();
        void disableDrive();
        bool reset();
        bool getLimitSwitchStatus();
        bool startWatchdog();
        bool stopWatchdog();
        bool isInError();
        unsigned int getError();
        void runSpeed();
        void runTorque();
        void runPosition();
        void runHoming();
        void runOther();

        static const int F_CMD_DI = 0x08;
        static const int F_CMD_EN = 0x0F;
        static const int F_CMD_V = 0x93;

        static const int F_RET_OK = 1;
        static const int F_RET_EEPROM_WRITTEN = -2;
        static const int F_RET_OVER_TEMP = -4;
        static const int F_RET_INVALID_PARAM = -5;
        static const int F_RET_UNKNOWN_CMD = -7;
        static const int F_RET_CMD_UNAVAILABLE = -8;
        static const int F_RET_FLASH_DEFECT = -13;

    protected:
        ArdDs402::enum_DS402_state attrState;
        int attrCommandedSpeed;

        bool propInvertDriveDirection;
        /** Reductor's value **/
        double propReductorValue;
        /** Encoder resolution in point by rev**/
        int propEncoderResolution;

        InputPort<double> inSpeedCmd;
        InputPort<double> inPositionCmd;
        InputPort<double> inTorqueCmd;

        OutputPort<double> outMeasuredPosition;
        OutputPort<double> outMeasuredTorque;
        OutputPort<double> outComputedSpeed;

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

        /** last command line faulhaber request */
        UNS8 m_faulhaberScriptCommand;
        UNS32 m_faulhaberScriptCommandParam;

        bool m_faulhaberCommandTodo;


        void getInputs();
        void setOutputs();
        void readCaptors();

/*************************************************************************/
        /* INTERFACE OROCOS */
/*************************************************************************/

        /**
         * Enable motors to move
         */
        void ooEnableDrive();

        /**
         * Enable motors to move
         */
        void ooDisableDrive();

        /**
         * Allows to send faulhaber commands in command line
         * param cmd : the faulhaber code of the command
         * param param : the faulhaber command's parameters
         */
        void ooFaulhaberCmd(int cmd, int param);

        /**
         * Choose the operation mode of the motor.
         * param mode : "speed","position","torque","homing","faulhaber"
         */
        void ooSetOperationMode(string mode);

    };

}

#endif /* FAULHABER3268BX4_HPP_ */
