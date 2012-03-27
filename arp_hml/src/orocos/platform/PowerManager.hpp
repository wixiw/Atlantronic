/*
 * PowerManager.hpp
 *
 *  Created on: 7 janv 2012
 *      Author: ard, wla
 */

#ifndef POWER_MANAGER_HPP_
#define POWER_MANAGER_HPP_

#include <taskcontexts/ARDTaskContext.hpp>

using namespace RTT;
using namespace arp_core;

namespace arp_hml
{
    /** surcharge du log */
    #define LOGS(level) RTT::log(m_owner.propEnableLog?level:Never)<<"["<<m_owner.getName()<<"] "

    /** \ingroup ARP-arp_hml
     *
     * \class PowerManager
     *
     * Handle power management on the ubitquity robot
     */
    class PowerManager
    {
    public:
        /** */
        PowerManager(ARDTaskContext& c);

        virtual ~PowerManager();

        /**
         * Get all motors operations
         */
        virtual bool configureHook();

        /**
         * Get information from all motors
         */
        virtual void updateHook();

    protected:

        /** Timeout when sending a command on the CAN, in s */
        double propCanRequestTimeout;
        Property<bool>* m_propRequireCompleteHardware;

        /** Drive soft enable state **/
        InputPort<bool> inLeftDrivingEnable;
        InputPort<bool> inRightDrivingEnable;
        InputPort<bool> inRearDrivingEnable;
        InputPort<bool> inLeftSteeringEnable;
        InputPort<bool> inRightSteeringEnable;
        InputPort<bool> inRearSteeringEnable;

        /** CAN Connectivity */
        InputPort<bool> inLeftDrivingConnected;
        InputPort<bool> inRightDrivingConnected;
        InputPort<bool> inRearDrivingConnected;
        InputPort<bool> inLeftSteeringConnected;
        InputPort<bool> inRightSteeringConnected;
        InputPort<bool> inRearSteeringConnected;
        InputPort<bool> inWoodheadInConnected;
        InputPort<bool> inWoodheadOutConnected;

        /**PowerManagement synthesis */
        OutputPort<bool> outDrivingEnable;
        OutputPort<bool> outSteeringEnable;
        OutputPort<bool> outEnable;
        OutputPort<bool> outEmergencyStop;

        /** */
        ARDTaskContext& m_owner;


        /** Pointer on the LeftDrive ooEnableDrive Operation**/
        OperationCaller<void(void)> m_ooEnableLeftDriving;
        /** Pointer on the RightDriveooEnableDrive Operation**/
        OperationCaller<void(void)> m_ooEnableRightDriving;
        /** Pointer on the RearDrive ooEnableDrive Operation**/
        OperationCaller<void(void)> m_ooEnableRearDriving;
        /** Pointer on the Left Turret ooEnableDrive Operation**/
        OperationCaller<void(void)> m_ooEnableLeftSteering;
        /** Pointer on the Right Turret ooEnableDrive Operation**/
        OperationCaller<void(void)> m_ooEnableRightSteering;
        /** Pointer on the Rear Turret ooEnableDrive Operation**/
        OperationCaller<void(void)> m_ooEnableRearSteering;

        /** Pointer on the LeftDrive ooDisableDrive Operation**/
        OperationCaller<void(void)> m_ooDisableLeftDriving;
        /** Pointer on the Right ooDisableDrive Operation**/
        OperationCaller<void(void)> m_ooDisableRightDriving;
        /** Pointer on the RearDrive ooDisableDrive Operation**/
        OperationCaller<void(void)> m_ooDisableRearDriving;
        /** Pointer on the Left Turret ooDisableDrive Operation**/
        OperationCaller<void(void)> m_ooDisableLeftSteering;
        /** Pointer on the Right Turret ooDisableDrive Operation**/
        OperationCaller<void(void)> m_ooDisableRightSteering;
        /** Pointer on the Rear Turret ooDisableDrive Operation**/
        OperationCaller<void(void)> m_ooDisableRearSteering;


        /** Pointer in the Left driving ooSetOperationMode Operation **/
        OperationCaller<bool(string)> m_ooSetDrivingOperationMode;
        /** Pointer in the Left steering ooSetOperationMode Operation **/
        OperationCaller<bool(string)> m_ooSetSteeringOperationMode;

        /**
         * Read and merge information about motor power
         */
        void readDriveEnable();

        /**
         * Read and merge information to detect the emergencyStop
         */
        void readConnectivity();

        /**
         * Use this to connect local handlers to all peer component operations.
         */
        bool getPeersOperations();

        /**
         * Manage power on F=left, rigth and rear driving motors
         * @param powerOn : set to true to activate power (wheelBlocked), false for freewheel
         * @return true when the power is on, on all the *3* motors ! So it'll fail on incomplete hardware setup
         */
        bool coSetDrivingMotorPower(bool powerOn);

        /**
         * Manage  power on F=left, rigth and rear driving motors
         * @param powerOn : set to true to activate power (wheelBlocked), false for freewheel
         * @return true when the power is on, on all the *3* motors ! So it'll fail on incomplete hardware setup
         */
        bool coSetSteeringMotorPower(bool powerOn);

        /**
         * Call ooSetDrivingMotorPower and ooSetSteeringMotorPower
         * @param powerOn : set to true to activate power (wheelBlocked), false for freewheel
         * @return true when both ooSetSteeringMotorPower and ooSetDrivingMotorPower returned true.
         */
        bool coSetMotorPower(bool powerOn);
    };
}

#endif /* POWER_MANAGER_HPP_ */
