/*
 * Monitor.hpp
 *
 *  Created on: 6 janv 2012
 *      Author: wla
 */

#ifndef MONITOR_HPP_
#define MONITOR_HPP_

#include "taskcontexts/ARDTaskContext.hpp"

using namespace RTT;

namespace arp_core
{
    /** \ingroup ARP-arp_core
     *
     * \class Monitor
     *
     * Cette classe permet de gerer plusieurs composants. Il y en a au moins une par projet
     * qui permet de faire l'interface de pilotage pour les projets supérieurs.
     */
    class Monitor : public ARDTaskContext
    {
    public:

        /** Constructeur pour définir le chemin vers le projet. Utile pour ROS*/
        Monitor(const std::string& name);
        /** Destructeur par défaut */
        virtual ~Monitor();

        /**
         * Configure all peers previously registered by the addMonitoredPeer command
         * The configuration is done in the order in which elements where inserted
         */
        virtual bool configureHook();

        /**
         * Start all peers previously registered by the addMonitoredPeer command
         * The configuration is done in the order in which elements where inserted
         */
        virtual bool startHook();

        /**
         * Checks if all components are still running
         */
        virtual void updateHook();

        /**
         * Stop all peers previously registered by the addMonitoredPeer command
         * The configuration is done in the reverse order in which elements where inserted
         */
        virtual void stopHook();

        /**
         * Cleanup all peers previously registered by the addMonitoredPeer command
         * The configuration is done in the reversed order in which elements where inserted
         */
        virtual void cleanupHook();

        /**
         * @param peerName : the name of the Orocos component that needs to be monitored
         * @return true if success, false if the peer to monitor is not in the peer list.
         */
        virtual bool addMonitoredPeer(std::string peerName );

        /**
         * display the list of monitored peers
         */
        virtual void displayMonitoredPeers();

    protected:
        /** List of peers to monitor */
        vector<TaskContext*> m_monitoredList;



    };
}

#endif /* MONITOR_HPP_ */
