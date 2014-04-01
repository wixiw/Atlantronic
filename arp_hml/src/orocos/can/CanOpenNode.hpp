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
#include "orocos/can/dictionnary/CanARD.h"

using namespace arp_core;

namespace arp_hml
{

class CanOpenNode;

struct CanNodeIdCard
{
        CanNodeIdCard() :
                nodeId(0), task(NULL), inBootUpFrame(NULL)
        {
        }

        CanNodeIdCard(int nodeId, arp_hml::CanOpenNode* task, RTT::InputPort<bool>* inBootUpFrame) :
                nodeId(nodeId), task(task), inBootUpFrame(inBootUpFrame)
        {
        }

        nodeID_t nodeId;
        arp_hml::CanOpenNode* task;
        RTT::InputPort<bool>* inBootUpFrame;
        RTT::OutputPort<eRunningState>* outRunningState;

        /**
         * Use this function to check the data validity of the current CanNodeIdCard
         */
        bool check()
        {
            bool res = true;

            if (nodeId < 0 || nodeId > 128 || nodeId == 0x00 || nodeId == 0x01 || nodeId == 0xFF)
                res = false;
            if (task == NULL)
                res = false;
            if (inBootUpFrame == NULL)
                res = false;

            return res;
        }
};

class CanOpenNode: public HmlTaskContext
{
    public:
        CanOpenNode(const std::string& name);

        /**
         * This function is for external schedulers that would like to let read/write functions to be done early and lately in the cycle.
         * Thoses schedulers have to take the CanOpenNode as a slave peer and update() early in the cycle and call updateLate() and the end of the cycle.
         * It calls updateLateHook() when isRunning())==TRUE
         */
        void updateLate();

        /**
         * Switch to sub running state : PREOP
         */
        //TODO should be private, mais j'arrive pas a y acceder depuis CanDispatcher :( ca me soule donc je bourrine
        bool ooEnterPreOp();

    protected:
        /** Last sync time received **/
        arp_time::ArdAbsoluteTime attrSyncTime;
        /** Period between attrSyncTime and last attrSyncTime*/
        arp_time::ArdTimeDelta attrPeriod;

        /** CAN adress of the node */
        nodeID_t propNodeId;

        /** Timeout before considering a node is not responding to a NMT request (in s) */
        arp_time::ArdTimeDelta propNmtTimeout;

        /** name of the CanOpenController this component will connect*/
        std::string propCanOpenControllerName;

        /** this script is executed in the configureHook to set up CAN config values*/
        std::string propCanConfigurationScript;

        /** Require a CAN node reset during configure. As the controller is already doing it, it is usually not usefull */
        bool propResetFirst;

        /**
         * Clock port which trigger our activity
         */
        InputPort<arp_time::ArdAbsoluteTime> inMasterClock;

        /**
         * Period related to inMasterClock date
         */
        InputPort<arp_time::ArdTimeDelta> inMasterPeriod;

        /**
         * port from which we receive the bootUp frame of our node from a CanOpenController
         */
        InputPort<bool> inBootUpFrame;

        /**
         * This port is true when the component thinks the device is disconnected of the network
         */
        OutputPort<eRunningState> outOperationnalState;

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
        OperationCaller<bool(CanDicoEntry, int*)> m_coReadInRemoteDico;

        /*
         * handler on a CanOpenController operation to request an NMT state change
         */
        OperationCaller<bool(nodeID_t nodeId, enum_DS301_nmtStateRequest nmtStateCmd, double timeout)> m_coRequestNmtChange;

        /** SubState of the CanOpencontroller in the Orocos Running State */
        eRunningState m_RunningState;

        /**
         * Connect to the CanOpenController, reset the node and send CAN configuration SDO
         */
        virtual bool configureHook();

        /**
         * Put the node in operationnal mode
         */
        virtual bool startHook();

        /**
         * Manage the running state machine
         * concrete nodes should not have to override this
         * prefer \see preopHook, \see operationalHook, \see idleHook
         */
        virtual void updateHook();

        /**
         * Hook for running pre op state
         */
        virtual void preopHook();

        /**
         * Hook for running op state
         * It switches to IDLE state if it receive a bootup msg
         */
        virtual void operationalHook();

        /**
         * Hook for running idle
         */
        virtual void idleHook();

        /**
         * Hook for running unconnected
         */
        virtual void unconnectedHook();

        /**
         *
         */
        virtual void updateLateHook()
        {
        }
        ;

        /**
         * Put the node in stop mode
         */
        virtual void stopHook();

        /**
         * unregister the node from the CanOpenController
         */
        virtual void cleanupHook();

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
         * @param timeout : delay to wait between the reset cmd and the expected bootup frame
         */
        bool resetNode(arp_time::ArdTimeDelta timeout);

        /**
         * Send configuration SDO to node from a script
         */
        bool configureNode();

        /**
         * Use this operation to send a PDO.
         * @param pdoNumber : number of the PDO on the CanFestival table (!! it is *NOT* the COBID !!)
         */
        bool coSendPdo(int pdoNumber);

        /**
         * Use this operation to emit an NMT state change request for a node.
         * This operation call CanOpenController::coRequestnmtChange()
         * @param nmtStateCmd : new NMT state in which we would like the node to be
         */
        bool coRequestNmtChange(enum_DS301_nmtStateRequest nmtStateCmd);

    private:
        /**
         * Shared structure with CanOpenController
         */
        CanNodeIdCard m_nodeIdCard;

};
}

#endif /* CANOPENNODE_HPP_ */
