/*
 * ArdMotor.hpp
 *
 *  Created on: 21 mars 2011
 *      Author: ard
 */

#ifndef ARDMOTORITF_HPP_
#define ARDMOTORITF_HPP_

namespace arp_hml
{
    class ArdMotorItf
    {
    public:
        /**
        * Motion type of the controller.
        */
        enum operationMode
        {
            SPEED_CONTROL,
            TORQUE_CONTROL,
            POSITION_CONTROL,
            HOMING,
            OTHER
        };

        /**
        * Sets the motion type drive.
        * The function is not implemented for Harmonica.
        * The harmonica drive is configured just for velocity mode.
        */
        virtual bool setOperationMode(int operationMode) = 0;

        /**
        * Initializes the drive.
        */
        virtual bool init() = 0;

        /**
        * Enables the motor.
        * After calling the drive accepts velocity and position commands.
        */
        virtual bool enableDrive() = 0;

        /**
        * Disables the motor.
        * After calling the drive won't accepts velocity and position commands.
        */
        virtual bool disableDrive() = 0;

        /**
        * Resets the drive.
        * The drive changes into the state after initializaton.
        */
        virtual bool reset() = 0;

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
        virtual double getTorque(){ return m_torqueMeasure;};

        /**
        * Sends command for motor Torque (in Nm)
        */
        virtual void setTorque(double torqueCmdNm ){ m_torqueCommand=torqueCmdNm;};

        /**
        * Returns the speed value in rad/s
        */
        virtual double getSpeed(){ return m_speedMeasure;};

        /**
        * Sends speed command in rad/s
        */
        virtual void setSpeed(double speedCmdRadS ){ m_speedCommand=speedCmdRadS;};

        /**
        * Returns the position of the motor in rad/s
        */
        virtual double getPosition(){ return m_positionMeasure;};

        /**
        * Sends position command in rad/s
        */
        virtual void setPosition(double positionCmdRadS ){ m_positionCommand=positionCmdRadS;};


    protected:
        operationMode m_operationMode;

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
