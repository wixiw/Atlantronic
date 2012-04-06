/*
 * MotorSimul.hpp
 *
 *  Created on: 02 April 2012
 *      Author: wla
 *
 *  This is the simulation of Faulhaber motors
 */

#ifndef MOTORSIMUL_HPP_
#define MOTORSIMUL_HPP_

#include "orocos/taskcontexts/HmlTaskContext.hpp"
#include <orocos/motor/ArdMotorItf.hpp>

namespace arp_hml
{

    class MotorSimul: public HmlTaskContext, public ArdMotorItf
    {
    public:
        MotorSimul(const std::string& name);

        /****************************************************************************
         *          OROCOS INTERFACE
         ****************************************************************************/

    protected:
        /**
         * Redefined to prevent getOperation from inheritance
         */
        bool configureHook();

        /**
         *
         */
        void updateHook();

        /**
         * Read input ports to populated ArdMotorItf.
         * Call this function before calling run()
         */
        void getInputs();

        /**
         * Write output ports from ArdMotorItf.
         * Call this function after calling run()
         */
        void setOutputs();

        /**
         * This attributre is used by the readCaptors loop to compute the speed with position of motor.
         * In a way, it is close to the period of the component.
         */
        double attrPeriod;
        /** time for which the propMaximalTorque has been reached */
        double attrBlockingDelay;
        /** brut value of the odometer */
        int attrIncrementalOdometer;
        /** index of the Faulhaber Command PDO */
        int attrFaulhaberCommandPdoIndex;

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

        /** Clock port which trigger our activity */
        InputPort<double> inClock;

        /** Command to be used in position mode. It must be provided in rad on the reductor's output.
         * It is not available yet. */
        InputPort<double> inPositionCmd;
        /** Command to be used in speed mode. It must be provided in rad/s on the reductor's output **/
        InputPort<double> inSpeedCmd;
        /** Command to be used in torque mode. This mode is not available yes **/
        InputPort<double> inTorqueCmd;

        OutputPort<double> outFilteredSpeedCommand;
        OutputPort<double> outFilteredPositionCommand;

        /** Provides the measured position of the encoder from CAN. It is converted in rad on the reductor's output's axe. **/
        OutputPort<double> outPosition;
        /** Sync time of the position mesure*/
        OutputPort<double> outClock;
        /** Provides the torque measured from CAN. In Amps**/
        OutputPort<double> outTorque;
        /** Provides a computed speed from the encoder position. In rad/s on the reductor's output's axe. */
        OutputPort<double> outVelocity;
        /** Prints the last Faulhaber command sent on CAN in OTHER mode of operation **/
        OutputPort<bool> outDriveEnable;
        /** Provides the current mode of operation of the motor (speed,position,torque,homing,other=faulhaber) **/
        OutputPort<string> outCurrentOperationMode;
        /** Is true when the propMaximalTorque has been reached*/
        OutputPort<bool> outMaxTorqueTimeout;

        /** Is always true*/
        OutputPort<bool> outConnected;

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


         /****************************************************************************
          *          ARDITFMOTOR INTERFACE
          ****************************************************************************/
    protected:
         /** See ArdMotorItf for details **/
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

    private:
         /** Memory of the power state in the motor */
         bool m_power;

         /** Last sync time received **/
         double m_syncTime;
         /** Last speed command received */
         double m_oldSpeedCommandTime;
         /** Last torque command received */
         double m_oldTorqueCommandTime;
    };

}

#endif
