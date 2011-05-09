/*
 * Faulhaber3268Bx4.hpp
 *
 *  Created on: 10 mars 2011
 *      Author: wla
 *
 *  This class allows to drive a Faulhaber 3268 Bx4 Can controlledd motor.
 *  It currently only have 2 modes of operation : SPEED_CONTROL and OTHER (=faulhaber command)
 *  Don't forget to enable the drive after boot to use it with the ooEnableDrive() and then
 *  switch back to spped mode with ooSetOperationMode("speed")
 */

#ifndef FAULHABER3268BX4_HPP_
#define FAULHABER3268BX4_HPP_

#include "hml/can/CanOpenNode.hpp"
#include "hml/motor/ArdMotorItf.hpp"
#include "hml/can/ard_can_types.hpp"
#include <sys/time.h>

using namespace arp_core;
//TODO WLA mettre la doc
namespace arp_hml
{

    class Faulhaber3268Bx4 : public CanOpenNode, public ArdMotorItf
    {
    public:
    	/**
    	 * Construct the Orocos interface
    	 * Get pointers to CanOpen dictonnary variables
    	 */
        Faulhaber3268Bx4(const std::string& name);

/****************************************************************************
 * 			OROCOS INTERFACE
 ****************************************************************************/

    public:

        /**
         * Read the motor commands and send on CAN bus.
         * Read the motor measures and publish them to Orocos
         */
        void updateHook();

        /**
         * Calls the ArdMotorItf::init() function
         */
        bool configureHook();

        /**
         * Disable drive
         */
        void stopHook();

<<<<<<< .working
=======

    protected:

        ArdDs402::enum_DS402_state attrState;
        double attrCommandedSpeed;
        double attrPeriod;

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
        OutputPort<bool> outDriveEnable;
        OutputPort<string> outCurrentOperationMode;

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
         * param mode : "speed","position","torque","homing","other"<=>faulhaber command
         */
        bool ooSetOperationMode(std::string mode);

        /**
         * Blocks on this until the drive is enabled.
         * param timeout : maximal blocking time in s.
         * return true is the drive is enable, false if the tiemout has expired
         */
        bool coWaitEnable(double timeout);

/****************************************************************************
 * 			ARDITFMOTOT INTERFACE
 ****************************************************************************/

    public:

        /**
         * Select a new Operation Mode for the motor
         */
        bool setOperationMode(ArdMotorItf::operationMode_t operationMode);

        /**
         * check if all pointers to CanOpen Dictionnary are non null
         */
>>>>>>> .merge-right.r435
        bool init();
<<<<<<< .working
        void enableDrive();
        void disableDrive();
=======

        /**
         * Switch the motor to OTHER mode of operation, and ask a faulhaber EN command to be send
         * Don't forget to switch the motor back to its normal operation mode
         */
        void enableDrive();

        /**
         * Switch the motor to OTHER mode of operation, and ask a faulhaber DI command to be send
         * Don't forget to switch the motor back to its normal operation mode
         */
        void disableDrive();

>>>>>>> .merge-right.r435
        bool reset();
        bool getLimitSwitchStatus();
        bool startWatchdog();
        bool stopWatchdog();
        bool isInError();
        unsigned int getError();
<<<<<<< .working
        void runSpeed();
        void runTorque();
        void runPosition();
        void runHoming();
        void runOther();
=======
        void runSpeed();
        void runTorque();
        void runPosition();
        void runHoming();
        void runOther();

/****************************************************************************
 * 			Faulhaber specific
 ****************************************************************************/

    public:
>>>>>>> .merge-right.r435

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

<<<<<<< .working
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
=======
>>>>>>> .merge-right.r435

<<<<<<< .working
        OutputPort<double> outMeasuredPosition;
        OutputPort<double> outMeasuredTorque;
        OutputPort<double> outComputedSpeed;

        OutputPort<int> outLastSentCommand;
        OutputPort<double> outLastSentCommandParam;
        OutputPort<int> outLastSentCommandReturn;
=======
    protected:
>>>>>>> .merge-right.r435

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

<<<<<<< .working
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
=======
        bool m_faulhaberCommandTodo;
>>>>>>> .merge-right.r435

        //memoire de la derniere valeur de position pour un calcul de vitesse
        double m_oldPositionMeasure;
        struct timespec m_oldPositionMeasureTime;

        void getInputs();
        void setOutputs();
        void readCaptors();

<<<<<<< .working
        /**
         * Choose the operation mode of the motor.
         * param mode : "speed","position","torque","homing","faulhaber"
         */
        void ooSetOperationMode(string mode);
=======
        virtual bool checkInputsPorts();

        /**
         * Elapsed time between begin and now, using data type timespec.
         * Return values simply to indicate return point
         */
        void delta_t(struct timespec *interval, struct timespec *begin, struct timespec *now);
>>>>>>> .merge-right.r435

    };

}

#endif /* FAULHABER3268BX4_HPP_ */
