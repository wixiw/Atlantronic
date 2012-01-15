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

        /**
         */
        virtual bool configure();

        /**
         */
        virtual void update();

    protected:
        /** Decide weather complete hardware must be present or not */
        bool propRequireCompleteHardware;

        /** Timeout when sending a command on the CAN, in s */
        bool propCanRequestTimeout;

        /** Drive soft enable state **/
        InputPort<bool> inLeftDrivingEnable;
        InputPort<bool> inRightDrivingEnable;
        InputPort<bool> inRearDrivingEnable;
        InputPort<bool> inLeftSteeringEnable;
        InputPort<bool> inRightSteeringEnable;
        InputPort<bool> inRearSteeringEnable;

        /**PowerManagement synthesis */
        OutputPort<bool> outDrivingEnable;
        OutputPort<bool> outSteeringEnable;
        OutputPort<bool> outEnable;

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
        OperationCaller<bool(string)> m_ooSetLeftDrivingOperationMode;
        /** Pointer in the Right driving ooSetOperationMode Operation **/
        OperationCaller<bool(string)> m_ooSetRightDrivingOperationMode;
        /** Pointer in the Right driving ooSetOperationMode Operation **/
        OperationCaller<bool(string)> m_ooSetRearDrivingOperationMode;
        /** Pointer in the Left steering ooSetOperationMode Operation **/
        OperationCaller<bool(string)> m_ooSetLeftSteeringOperationMode;
        /** Pointer in the Right steering ooSetOperationMode Operation **/
        OperationCaller<bool(string)> m_ooSetRightSteeringOperationMode;
        /** Pointer in the Right steering ooSetOperationMode Operation **/
        OperationCaller<bool(string)> m_ooSetRearSteeringOperationMode;

        /**
         *
         */
        bool ooSetDrivingMotorPower(bool powerOn);

        /**
         *
         */
        bool ooSetSteeringMotorPower(bool powerOn);

        /**
         *
         */
        bool ooSetMotorPower(bool powerOn);

        /**
         *
         */
        bool getPeersOperations();
    };
}

#endif /* POWER_MANAGER_HPP_ */
