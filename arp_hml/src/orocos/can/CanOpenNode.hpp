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
#include <native/task.h>

using namespace arp_core;

namespace arp_hml
{
    class CanOpenNode;

    struct CanNodeIdCard
    {
        CanNodeIdCard():
            nodeId(0),
            task(NULL),
            inBootUpFrame(NULL)
            {}

        CanNodeIdCard(int nodeId, arp_hml::CanOpenNode* task, RTT::InputPort<bool>* inBootUpFrame):
            nodeId(nodeId),
            task(task),
            inBootUpFrame(inBootUpFrame)
            {}

        nodeID_t nodeId;
        arp_hml::CanOpenNode* task;
        RTT::InputPort<bool>* inBootUpFrame;

        /**
         * Use this function to check the data validity of the current CanNodeIdCard
         */
        bool check()
        {
            bool res = true;

            if( nodeId < 0 || nodeId > 128 || nodeId == 0x00 || nodeId == 0x01 || nodeId == 0xFF)
                res = false;
            if( task == NULL )
                res = false;
            if( inBootUpFrame == NULL )
                res = false;

            return res;
        }
    };



    class CanOpenNode: public HmlTaskContext
    {
    public:
        CanOpenNode(const std::string& name);

        /**
         * Connect to the CanOpenController, reset the node and send CAN configuration SDO
         */
        virtual bool configureHook();

        /**
         * Put the node in operationnal mode
         */
        virtual bool startHook();

        /**
         * Handles Bootup : log a warning message
         */
        virtual void updateHook();

        /**
         * This function is for external schedulers that would like to let read/write functions to be done early and lately in the cycle.
         * Thoses schedulers have to take the CanOpenNode as a slave peer and update() early in the cycle and call updateLate() and the end of the cycle.
         */
        virtual void updateLate(){};

        /**
         * Put the node in stop mode
         */
        virtual void stopHook();

        /**
         * unregister the node from the CanOpenController
         */
        virtual void cleanupHook();


    protected:
        /** Last sync time received **/
        timespec attrSyncTime;
        /** Period between attrSyncTime and last attrSyncTime*/
        double attrPeriod;

        /** CAN adress of the node */
        nodeID_t propNodeId;

        /** Timeout before considering a node is not responding to a NMT request (in s) */
        double propNmtTimeout;

        /** Time required to boot the device from a reset command (in s)*/
        double propDeviceBootTime;

        /** name of the CanOpenController this component will connect*/
        std::string propCanOpenControllerName;

        /** this script is executed in the configureHook to set up CAN config values*/
        std::string propCanConfigurationScript;

        /** Require a CAN node reset during configure. As the controller is already doing it, it is usually not usefull */
        bool propResetFirst;

        /**
         * Clock port which trigger our activity
         */
        InputPort<timespec> inMasterClock;

        /**
         * Period related to inMasterClock date
         */
        InputPort<double> inMasterPeriod;

        /**
         * port from which we receive the bootUp frame of our node from a CanOpenController
         */
        InputPort<bool> inBootUpFrame;

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


        /**
         * Use this operation to send a PDO.
         * If you are under xenomai you must run in primary mode
         * @param pdoNumber : number of the PDO on the CanFestival table (!! it is *NOT* the COBID !!)
         */
        bool coSendPdo(int pdoNumber);

        /**
         * Use this operation to emit an NMT state change request for a node.
         * This operation will do some polling on the NMT state with PDO request
         * So don't use this operation in operationnal ! (only for booting and configuring).
         * It's a blocking function.
         * It will send 2 things :
         * _ the NMT cmd request with OOO#cmd.nodeId
         * _ an NMT state reques 700+nodeId#R
         * and wait for the 700+nodeId#nmtState message to come.
         * @param nodeId : node ID of the slave node to which we send the request
         * @param nmtStateCmd : new NMT state in which we would like the node to be
         * @param timeout : timeout on the polling
         */
        bool coSendNmtCmd(nodeID_t nodeId, enum_DS301_nmtStateRequest nmtStateCmd, double timeout);

        /**
         * Compares an NMT state to a sended NMT command.
         * The NMT state is get from CanARD_Data.NMTable[nodeId]
         * @param nmtCmd : the NMT command sended to the slave node
         * @param nodeId : Id of the node to whoch the command has been sended
         * @return true is the NMT cmd has been processed
         */
        bool isNmtStateChangeDone(enum_DS301_nmtStateRequest nmtCmd, nodeID_t nodeId);

    private:
        /**
         * Shared structure with CanOpenController
         */
        CanNodeIdCard m_nodeIdCard;

        //TODO workaround en attendant xenomai sous Orocos
        RT_TASK rt_task_desc;

    };

}

#endif /* CANOPENNODE_HPP_ */
