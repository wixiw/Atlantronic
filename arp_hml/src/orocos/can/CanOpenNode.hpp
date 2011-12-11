/*
 * CanOpenNode.hpp
 *
 *  Created on: 21 mars 2011
 *      Author: wla
 */

#ifndef CANOPENNODE_HPP_
#define CANOPENNODE_HPP_

#include "orocos/taskcontexts/HmlTaskContext.hpp"
#include "orocos/can/ard_can_types.hpp"
#include <canfestival/canfestival.h>

using namespace arp_core;

namespace arp_hml
{

    class CanOpenNode: public HmlTaskContext
    {
    public:
        CanOpenNode(const std::string& name);

        /**
         * Connect to the CanOpenController, reset the node and send CAN configuration SDO
         */
        bool configureHook();

        /**
         * Put the node in operationnal mode
         */
        bool startHook();

        /**
         * Handles Bootup : log a warning message
         */
        void updateHook();

        /**
         * Put the node in stop mode
         */
        void stopHook();

        /**
         * unregister the node from the CanOpenController
         */
        void cleanupHook();


    protected:
        /** This attribute contains the current NMT status of the node  */
        e_nodeState attrCurrentNMTState;

        /** Last sync time received **/
        double attrSyncTime;

        /** CAN adress of the node */
        nodeID_t propNodeId;

        /** Timeout before considering a node is not responding to a NMT request (in ms) */
        int propNmtTimeout;

        /** name of the CanOpenController this component will connect*/
        string propCanOpenControllerName;

        /** this script is executed in the configureHook to set up CAN config values*/
        string propCanConfigurationScript;

        /**
         * Clock port which trigger our activity
         */
        InputPort<timespec> inMasterClock;

        /**
         * port from which we receive the nmt state of our node from a CanOpenController
         */
        InputPort<enum_nodeState> inNmtState;

        /**
         * port from which we receive the bootUp frame of our node from a CanOpenController
         */
        InputPort<bool> inBootUpFrame;

        /**
         * port in which the component can ask a new nmt state for our node from a CanOpenController
         */
        OutputPort<enum_DS301_nmtStateRequest> outRequestNmtState;

        /**
         * This port is true when the component thinks the device is disconnected of the network
         */
        OutputPort<bool> outConnected;

        /**
         * handler on a CanOpenController operation to register
         */
        OperationCaller<bool(CanNodeIdCard)> m_ooRegister;

        /**
         * handler on a CanOpenController operation to unregister
         */
        OperationCaller<bool(nodeID_t)> m_ooUnregister;

        /**
         * handler on a CanOpenController operation to write in the node attrNodeId dictionnary
         */
        OperationCaller<bool(CanDicoEntry)> m_coWriteInRemoteDico;

        /**
         * handler on a CanOpenController operation to read from the node attrNodeId dictionnary
         */
        OperationCaller<bool(CanDicoEntry,int*)> m_coReadInRemoteDico;

        /**
         * use this operation in deployment to register the node into the CanOpenController
         */
        bool coRegister();

        /**
         * Check component properties values
         */
        virtual bool checkProperties();

        /**
         * Update the m_nodeIdCard with the last property value
         */
        void updateNodeIdCard();

        /**
         * Get a reference to the Operations of our favorite CanOpenController
         * Is used in the configureHook exclusively
         */
        bool connectOperations();

        /**
         * Reset the node, this is a blocking call
         */
        bool resetNode();

        /**
         * Send configuration SDO to node from a script
         */
        bool configureNode();

    private:
        /**
         * Shared structure with CanOpenController
         */
        CanNodeIdCard m_nodeIdCard;




    };

}

#endif /* CANOPENNODE_HPP_ */