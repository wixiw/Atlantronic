/*
 * HmlHmlMonitor.hpp
 *
 *  Created on: 7 janv 2012
 *      Author: ard, wla
 */

#ifndef HML_MONITOR_HPP_
#define HML_MONITOR_HPP_

#include <components/Monitor.hpp>
#include "PowerManager.hpp"
#include "StateManager.hpp"

namespace arp_hml
{
    /** \ingroup ARP-arp_hml
     *
     * \class HmlMonitor
     *
     * inherited to handle bus managers which has to be started before other components are configured
     */
    class HmlMonitor : public arp_core::Monitor
    {
    public:

        /** Constructeur pour définir le chemin vers le projet. Utile pour ROS*/
        HmlMonitor(const std::string& name);

        /**
         * Configure all peers previously registered by the addHmlMonitoredPeer command
         * The configuration is done in the order in which elements where inserted
         */
        virtual bool configureHook();

        /**
         * Checks if all components are still running
         */
        virtual void updateHook();

        /**
         * Check if some motors are blocked
         */
        void monitorBlockedMotors();

        /**
         * Cleanup all peers previously registered by the addHmlMonitoredPeer command
         * The configuration is done in the reversed order in which elements where inserted
         */
        virtual void cleanupHook();

        /**
         * @param peerName : the name of the Orocos component that needs to be monitored as a bus manager
         * @return true if success, false if the peer to monitor is not in the peer list.
         */
        virtual bool addHmlBusMonitoredPeer(std::string peerName );

        /**
         * display the list of monitored peers
         */
        virtual void displayHmlMonitoredPeers();

        /**
         * Returns a string containing HML version
         */
        std::string coGetHmlVersion();

        /**
         * Send a reset node to all Can nodes.
         * TODO this is a workaround, Can1 should be abble to do it alone
         */
        bool coResetHml();

        /** Decide weather complete hardware must be present or not */
        bool propRequireCompleteHardware;

    protected:
        /** List of peers to monitor */
        std::vector<RTT::TaskContext*> m_monitoredBusList;

        RTT::InputPort<bool> inLeftDrivingBlocked;
        RTT::InputPort<bool> inRightDrivingBlocked;
        RTT::InputPort<bool> inRearDrivingBlocked;
        RTT::InputPort<bool> inLeftSteeringBlocked;
        RTT::InputPort<bool> inRightSteeringBlocked;
        RTT::InputPort<bool> inRearSteeringBlocked;

        RTT::OutputPort<bool> outOneDrivingIsBlocked;
        RTT::OutputPort<bool> outAllDrivingAreBlocked;
        RTT::OutputPort<bool> outOneSteeringIsBlocked;
        RTT::OutputPort<bool> outAllSteeringAreBlocked;


        /** Pointer in the Woodhead 8 out coReset Operation**/
        RTT::OperationCaller<bool(void)> m_coResetWoodheadOut;
        /** Pointer in the Woodhead 8 in coReset Operation**/
        RTT::OperationCaller<bool(void)> m_coResetWoodheadIn;
        /** Pointer in the LeftDriving coReset Operation **/
        RTT::OperationCaller<bool(void)> m_coResetLeftDriving;
        /** Pointer in the RightDriving coReset Operation **/
        RTT::OperationCaller<bool(void)> m_coResetRightDriving;
        /** Pointer in the RearDriving coReset Operation **/
        RTT::OperationCaller<bool(void)> m_coResetRearDriving;
        /** Pointer in the LeftSteering coReset Operation **/
        RTT::OperationCaller<bool(void)> m_coResetLeftSteering;
        /** Pointer in the RightSteering coReset Operation **/
        RTT::OperationCaller<bool(void)> m_coResetRightSteering;
        /** Pointer in the RearSteering coReset Operation **/
        RTT::OperationCaller<bool(void)> m_coResetRearSteering;

        PowerManager m_powerManager;
        StateManager m_stateManager;

        /** Helper function to deport Orocos interface setting up at the end of file*/
        void addOrocosInterface();
    };
}

#endif /* HML_MONITOR_HPP_ */
