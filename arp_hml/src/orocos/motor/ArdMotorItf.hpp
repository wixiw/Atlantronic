/*
 * ArdMotor.hpp
 *
 *  Created on: 21 mars 2011
 *      Author: wla
 */

#ifndef ARDMOTORITF_HPP_
#define ARDMOTORITF_HPP_

#include <string>

namespace arp_hml
{
    class ArdMotorItf
    {
    public:
        /**
        * Motion type of the controller.
        */
        enum operationMode_t
        {
            SPEED_CONTROL,
            TORQUE_CONTROL,
            POSITION_CONTROL,
            HOMING,
            OTHER
        };

        /**
        * Sets the motion type drive.
        */
        virtual bool setOperationMode(operationMode_t operationMode);

        /**
        * Gets the motion type drive.
        * returns an
        */
        virtual operationMode_t getOperationMode();

        /**
        * Initializes the drive.
        */
        virtual bool init() = 0;

        /**
        * Resets the drive.
        * The drive changes into the state after initializaton.
        */
        virtual bool reset() = 0;

        /**
         * Run the drive. Call this every cycle
         * You should not derivate this normally.
         */
        virtual void run();

        /**
         * This function is called by the run function when the motor is in the SPEED_CONTROL mode
         */
        virtual void runSpeed() = 0;

        /**
         * This function is called by the run function when the motor is in the TORQUE_CONTROL mode
         */
        virtual void runTorque() = 0;

        /**
         * This function is called by the run function when the motor is in the POSITION_CONTROL mode
         */
        virtual void runPosition() = 0;

        /**
         * This function is called by the run function when the motor is in the HOMING mode
         */
        virtual void runHoming() = 0;

        /**
         * This function is called by the run function when the motor is in the OTHER mode
         */
        virtual void runOther() = 0;

        /**
        * Enables the motor.
        * After calling the drive accepts velocity and position commands.
        */
        virtual void enableDrive() = 0;

        /**
        * Disables the motor.
        * After calling the drive won't accepts velocity and position commands.
        */
        virtual void disableDrive() = 0;

        /**
        * Returns the status of the limit switch needed for homing.
        * true = limit switch is reached; false = not reached
        */
        virtual bool getLimitSwitchStatus() = 0;

        /**
        * Starts the watchdog.
        * The update is is done each time a command is given to the motor
        */
        virtual bool startWatchdog() = 0;

        /**
        * Stops the watchdog.
        * The update is is done each time a command is given to the motor
        */
        virtual bool stopWatchdog() = 0;

        /**
        * Returns true if an error has been detected.
        */
        virtual bool isInError() = 0;

        /**
        * Return a bitfield containing information about the pending errors.
        */
        virtual unsigned int getError() = 0;

        /**
        * Returns the torque value in Nm
        */
        virtual double getTorqueMeasure();

        /**
        * Sends command for motor Torque (in Nm)
        */
        virtual void setTorqueCmd(double torqueCmdNm );

        /**
        * Returns the speed value in rad/s
        */
        virtual double getSpeedMeasure();

        /**
        * Sends speed command in rad/s
        */
        virtual void setSpeedCmd(double speedCmdRadS );

        /**
        * Returns the position of the motor in rad/s
        */
        virtual double getPositionMeasure();

        /**
        * Sends position command in rad/s
        */
        virtual void setPositionCmd( double positionCmdRadS );

        /**
         * Convert an operationMode_t variable into a string
         * SPEED_CONTROL="speed"
         * TORQUE_CONTROL="torque"
         * POSITION_CONTROL="position"
         * HOMING="homing"
         * OTHER="other"
         * returns "unknown" if switch case error
         */
        std::string getStringFromMode( operationMode_t mode );

        /**
         * Convert a string variable into a operationMode_t
         * SPEED_CONTROL="speed"
         * TORQUE_CONTROL="torque"
         * POSITION_CONTROL="position"
         * HOMING="homing"
         * OTHER="other"
         * returns OTHER if the string doesn't match anything
         */
        operationMode_t getModeFromString( std::string mode );

    protected:
        operationMode_t m_operationMode;

        /** Contains the last torque measure in Nm */
        double m_torqueMeasure;
        /** Contains the last torque command in Nm */
        double m_torqueCommand;
        /** Contains the last speed measure in rad/s */
        double m_speedMeasure;
        /** Contains the last speed command in rad/s */
        double m_speedCommand;
        /** Contains the last position measure in rad */
        double m_positionMeasure;
        /** Contains the last position command in rad */
        double m_positionCommand;
    };
};
#endif /* ARDMOTORITF_HPP_ */
