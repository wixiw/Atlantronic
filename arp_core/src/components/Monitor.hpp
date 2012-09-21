/*
 * Monitor.hpp
 *
 *  Created on: 6 janv 2012
 *      Author: wla
 */

#ifndef MONITOR_HPP_
#define MONITOR_HPP_

#include "taskcontexts/ARDTaskContext.hpp"

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

        /** This property can be set to true to enable a parallel call of configure/start/stop/cleanup functions */
        bool propParallelStart;


        /**
         * Configure all peers previously registered by the addMonitoredPeer command
         * The configuration is done in the order in which elements where inserted.
         *
         * It also checks that all input ports of registered peers are connected.
         *
         * Depending on the value of propParallelStart, the configure operation
         * on peers are called in a blocking (Sequential configure) or non-blocking
         * (Parallel configure) way. In the latter case, the polling rate of results is 1Hz
         * (1 every second).
         */
        virtual bool configureHook();

        /**
         * Start all peers previously registered by the addMonitoredPeer command
         * The configuration is done in the order in which elements where inserted
         *      
         * Depending on the value of propParallelStart, the start operation
         * on peers are called in a blocking (Sequential start) or non-blocking
         * (Parallel start) way. In the latter case, the polling rate of results is 1Hz
         * (1 every second).
         */
        virtual bool startHook();

        /**
         * Checks if all components are still running
         */
        virtual void updateHook();

        /**
         * Recover when all components returns to running state.
         */
        virtual void errorHook();

        /**
         * Stop all peers previously registered by the addMonitoredPeer command
         * The configuration is done in the reverse order in which elements where inserted
         *    
         * Depending on the value of propParallelStart, the stop operation
         * on peers are called in a blocking (Sequential stop) or non-blocking
         * (Parallel stop) way.
         */
        virtual void stopHook();

        /**
         * Cleanup all peers previously registered by the addMonitoredPeer command
         * The configuration is done in the reversed order in which elements where inserted
         * 
         * Depending on the value of propParallelStart, the cleanup operation
         * on peers are called in a blocking (Sequential cleanup) or non-blocking
         * (Parallel cleanup) way.
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

        /**
         * Returns a string containing Core version
         */
        std::string coGetCoreVersion();

        /**
         * Use this to connect internal peers
         * @param compA : name of the internal peer
         * @param portA : name of the port of portA
         * @param tcB : the Component that is trying to connect to the internal peer
         * @param portItfB : port of tcB to connect to the internal peer compA.
         */
        bool connect(const std::string& compA, const std::string& portA, const std::string& compB, const std::string& portB);

    protected:
        /** List of peers to monitor */
        std::vector<RTT::TaskContext*> m_monitoredList;

        /** Use this fucntion to check that all input port of registered components are connected */
        bool checkPortConnection();

        /** This is an helper function to hide code that checks the results of sent handles in configure/start parallelised call.*/
        bool checkSendHandle(std::vector< RTT::SendHandle<bool(void)> > operationsSentHandles);

    };
}

#endif /* MONITOR_HPP_ */
