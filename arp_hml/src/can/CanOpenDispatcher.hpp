/*
 * CanOpenDispatcher.hpp
 *
 *  Created on: 28 mars 2011
 *      Author: wla
 *
 * This class is handling the routing work of the CanOpenController.
 * It has the responsability to drive incoming data to the rigth component.
 *
 * Components has to register before having a chance to receive routed datas !
 */

#ifndef CANOPENDISPATCHER_HPP_
#define CANOPENDISPATCHER_HPP_

#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include "can/ard_can_types.hpp"


namespace arp_hml
{

    class CanOpenDispatcher
    {
    public:
        CanOpenDispatcher(TaskContext& tc);
        virtual ~CanOpenDispatcher();

        /**
         * This operation allows a Device Component to register in the CanController
         * This function is NOT realtime at all
         * @param the "idCard" of the node who wants to be added
         * @return true if the node has been registred successfully.
         */
        bool ooRegisterNewNode(CanNodeIdCard node);

        /**
         * This operation allows a Device Component to unregister from the CanController
         * This function is NOT realtime at all
         * @param the nodeId of the node who wants to be added
         * @return true if the node has been unregistred successfully.
         */
        bool ooUnregisterNode(nodeID_t nodeId);

        /**
         * read the inBootUpReceived and send the information to registered nodes
         */
        void dispatchBootUp(nodeID_t propNodeId, InputPort<nodeID_t>& inBootUpReceived);

        /**
         * read the table of NMT states and dispatch change to registered nodes
         */
        void dispatchNmtState();

        /**
         * dispatch the NMT state for a particular node nodeId
         * @param nodeId : the node for which to update NMT state port
         */
        bool dispatchNmtState(nodeID_t nodeId);

        /**
         * Wakes up all slave activities of registered Nodes
         */
        void wakeUpNodes();


        /**
         * DEBUG Purposes : this operation prints in the console the registred nodes.
         */
        void ooPrintRegisteredNodes();

        /**
         * This structure is used to populate m_registeredNodes with all needed stuff in one record
         */
        class nodeRegistration_t
        {
            public:
                //nodeID_t nodeId;
                OutputPort<e_nodeState> outNmtState;
                OutputPort<bool> outBootUp;
                ARDTaskContext* task;
        };

    protected:
        /**
         * Contains a map of nodeRegistration to do the dispatch stuff
         */
        map< nodeID_t, nodeRegistration_t* > m_registeredNodes;

        /**
         * Component into we are operating
         */
        TaskContext& m_parent;
    };

}

#endif /* CANOPENDISPATCHER_HPP_ */
