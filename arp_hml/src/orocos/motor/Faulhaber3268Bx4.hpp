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

#include "orocos/can/CanOpenNode.hpp"
#include "orocos/motor/ArdMotorItf.hpp"
#include "orocos/motor/FaulhaberStates.h"
#include "orocos/can/ard_can_types.hpp"
#include <sys/time.h>

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

    protected:

        /**
         * DS402 state of the motor. Whatever Faulhaber command ask, this state is always up to
         * date. It provides accurate information on what's happening inside. Moreover, this state could
         * provide automation feedback like "target reached".
         */
        ArdDs402::enum_DS402_state attrState;

        OutputPort<int> outStateBrut;

        /** time for which the propMaximalTorque has been reached */
        double attrBlockingDelay;
        /** brut value of the odometer */
        int attrIncrementalOdometer;
        /** index of the Faulhaber Command PDO */
        int attrFaulhaberCommandPdoIndex;

        /** Only used in running homing */
        HomingState attrHomingState;
        /** is true when homing is finished */
        bool attrHardNotify;

        /** Is true when you when to invert the speed command and feedback of the motor softly **/
        bool propInvertDriveDirection;
        /** Reductor's value from the motor's axe to the reductor's output's axe. So it should be greather than 1**/
        double propReductorValue;
        /** Encoder resolution in point by rev **/
        int propEncoderResolution;
        /** Maximal Torque allowed in Amps*/
        double propMaximalTorque;
        /** Maximal delay beetween 2 commands to consider someone is still giving coherent orders*/
        double propInputsTimeout;
        /** Homing speed in rad/s on the reductor output */
        double propHomingSpeed;
        /** Timeout before thinking a wheel is blocked */
        double propBlockingTorqueTimeout;

        /** Command to be used in position mode. It must be provided in rad on the reductor's output.
         * It is not available yet. */
        InputPort<double> inPositionCmd;
        /** Command to be used in speed mode. It must be provided in rad/s on the reductor's output **/
        InputPort<double> inSpeedCmd;
        /** Command to be used in torque mode. This mode is not available yes **/
        InputPort<double> inTorqueCmd;

        /** Provides the measured position of the encoder from CAN. It is converted in rad on the reductor's output's axe. **/
        OutputPort<double> outPosition;
        /** Sync time of the position mesure*/
        OutputPort<timespec> outClock;
        /** Provides the torque measured from CAN. In Amps**/
        OutputPort<double> outTorque;
        /** Provides a computed speed from the encoder position. In rad/s on the reductor's output's axe. */
        OutputPort<double> outVelocity;
        /** Prints the last Faulhaber command sent on CAN in OTHER mode of operation **/
        OutputPort<int> outLastSentCommand;
        /** Prints the last Faulhaber params sent on CAN in OTHER mode of operation **/
        OutputPort<double> outLastSentCommandParam;
        /** Prints the last Faulhaber command return received from CAN in OTHER mode of operation **/
        OutputPort<int> outLastSentCommandReturn;
        /** Is true when the drive is ready to be operated (axe blocked). If it is false, the axe is free of any mouvement **/
        OutputPort<bool> outDriveEnable;
        /** Provides the current mode of operation of the motor (speed,position,torque,homing,other=faulhaber) **/
        OutputPort<std::string> outCurrentOperationMode;
        /** Is true when the propMaximalTorque has been reached*/
        OutputPort<bool> outMaxTorqueTimeout;
        /** Is true when the Homing sequence is finished. It has no sense when the motor is not in HOMING mode */
        OutputPort<bool> outHomingDone;

        /**
          * Read the motor measures from CAN and publish them to Orocos
          * should be called early in the cycle
          */
         void updateHook();

         /**
          * Read the motor commands and send on CAN bus.
          * Should be called late in the cycle
          */
         void updateLateHook();

         /**
          * Calls the ArdMotorItf::init() function
          */
         bool configureHook();

         /**
          * Disable drive
          */
         void stopHook();

        /**
         * Limits the torque on the motor via a current limitation.
         * @param ampValue : the maximal current that the motor can require in A. Should be in ]0.2;10[
         * @return true when the change will be done, false if something prevents the change (like if the motor is not disabled or component not running)
         */
        bool ooLimitCurrent(double ampValue);

        /**
         * Allows to send faulhaber commands in command line
         * param cmd : the faulhaber code of the command
         * param param : the faulhaber command's parameters
         */
        void ooFaulhaberCmd(int cmd, int param);

        /**
         * TODO WLA : post coupe virer
         * Permet d'attendre pour bloquer le script de déploiement
         * @param dt temps en s à dormir
         */
        void ooSleep(int dt);

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

        /**
         * Utility function to hide the long orocos interface declaration outside of the constructor
         */
        void createOrocosInterface();

/****************************************************************************
 * 			ARDITFMOTOR INTERFACE
 ****************************************************************************/

    public:

        /**
         * Select a new Operation Mode for the motor
         */
        bool setOperationMode(ArdMotorItf::operationMode_t operationMode);

        /**
         * check if all pointers to CanOpen Dictionnary are non null
         */
        bool init();

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

        /** See ArdMotorItf for details **/
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

/****************************************************************************
 * 			Faulhaber specific
 ****************************************************************************/

    public:

        /**
         * Faulhaber command codes
         */
        ///disable drive
        static const int F_CMD_DI = 0x08;
        ///enable drive
        static const int F_CMD_EN = 0x0F;
        ///send a speed command
        static const int F_CMD_V = 0x93;

        ///limit the maximal current
        static const int F_CMD_LPC = 0x81;
        ///limit the continous current
        static const int F_CMD_LCC = 0x80;

        ///launch positioning
        static const int F_CMD_M = 0x3C;
        ///load an absolute position target
        static const int F_CMD_LA = 0xB4;

        ///configure homing : edge for switches
        static const int F_CMD_HP = 0x79;
        static const int F_CMD_HP_FAILING_EDGE = 0;
        static const int F_CMD_HP_RISING_EDGE = 1;

        //Switches masks
        static const UNS32 F_ANALOG_SWICTH_MASK = 0b1;
        static const UNS32 F_FAULT_SWICTH_MASK = 0b10;
        static const UNS32 F_3RD_SWICTH_MASK = 0b100;

        ///configure homing : reset on switch
        static const int F_CMD_SHA = 0x8A;
        ///configure homing : stop on switch
        static const int F_CMD_SHL = 0x90;
        ///configure homing : notify on switch
        static const int F_CMD_SHN = 0x9A;
        ///configure homing : grouped config
        static const int F_CMD_HOC = 0x5B;

        ///configure homing : speed
        static const int F_CMD_HOSP = 0x78 ;

        ///configure homing : speed
        static const int F_CMD_GOHOSEQ = 0x2F ;

        static const int F_CMD_HN = 0x76;

        /*
         * Faulhaber return codes should be clear
         */
        static const int F_RET_OK = 1;
        static const int F_RET_EEPROM_WRITTEN = -2;
        static const int F_RET_OVER_TEMP = -4;
        static const int F_RET_INVALID_PARAM = -5;
        static const int F_RET_UNKNOWN_CMD = -7;
        static const int F_RET_CMD_UNAVAILABLE = -8;
        static const int F_RET_FLASH_DEFECT = -13;



        /** Use this to convert Nm to A*/
        static const double NM_TO_A = 1;

    protected:

        /*
         * Pointers to CanFestival Dictionnary values. they are the direct values read from CAN
         */
        INTEGER32* m_measuredPosition;
        INTEGER16* m_measuredCurrent;
        UNS8* m_measuredPeriod;
        UNS8* m_faulhaberCommand;
        UNS32* m_faulhaberCommandParameter;
        UNS8* m_faulhaberCommandReturn;
        UNS8* m_faulhaberCommandReturnCode;
        UNS32* m_faulhaberCommandReturnParameter;
        UNS16* m_ds402State;

        /** Last command line faulhaber request  */
        UNS8 m_faulhaberScriptCommand;
        UNS32 m_faulhaberScriptCommandParam;

        /** Is set to true when a new command has arrived and has to be executed */
        bool m_faulhaberCommandTodo;

        /** Last used position for speed computation */
        double m_oldPositionMeasure;
        /** Time of last speed computation */
        timespec m_oldPositionMeasureTime;
        /** Last speed command received */
        timespec m_oldSpeedCommandTime;
        /** Last torque command received */
        timespec m_oldTorqueCommandTime;
        /** is motor blocked */
        bool m_isMotorBlocked;

        /** Read the input ports and prepare internal variables. It allows to do a snapshot of inputs to work with coherent datas */
        void getInputs();
        /** Write tghe output ports when all internal datas are computed. It allows to provide coherent datas to the outside */
        void setOutputs();
        /** Read informations from the motor captors (position and current) */
        void readCaptors();

        /** Derivated to check personnal properties */
        virtual bool checkProperties();

    };

}

#endif /* FAULHABER3268BX4_HPP_ */
