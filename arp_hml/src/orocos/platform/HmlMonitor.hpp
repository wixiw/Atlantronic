/*
 * HmlHmlMonitor.hpp
 *
 *  Created on: 7 janv 2012
 *      Author: ard, wla
 */

#ifndef HML_MONITOR_HPP_
#define HML_MONITOR_HPP_

#include <components/Monitor.hpp>

using namespace RTT;
using namespace arp_core;

namespace arp_hml
{
    /** \ingroup ARP-arp_hml
     *
     * \class HmlMonitor
     *
     * inherited to handle bus managers which has to be started before other components are configured
     */
    class HmlMonitor : public Monitor
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

    protected:
        /** List of peers to monitor */
        vector<TaskContext*> m_monitoredBusList;



    };
}

#endif /* HML_MONITOR_HPP_ */
