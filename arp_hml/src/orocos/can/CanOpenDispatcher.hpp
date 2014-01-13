/*
 * CanOpenDispatcher.hpp
 *
 *  Created on: 28 mars 2011
 *      Author: wla
 *

 */

#ifndef CANOPENDISPATCHER_HPP_
#define CANOPENDISPATCHER_HPP_

#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include "orocos/can/ard_can_types.hpp"
#include "CanOpenNode.hpp"

namespace arp_hml
{
	/**
	 * \ingroup arp_hml
	 *
	 * \class CanOpenDispatcher
	 *
	 * \brief Do the routing work of the CanController
	 *
	 * This class is handling the routing work of the CanOpenController.
	 * It has the responsability to drive incoming data to the rigth component.
	 *
	 * Components has to register before having a chance to receive routed datas !
	 */
    class CanOpenDispatcher
    {
    public:
    	/**
    	 * @param tc : pointer to the parent task context (A CanOpenController)
    	 */
        CanOpenDispatcher(RTT::TaskContext& tc);
        virtual ~CanOpenDispatcher();

        /**
         * Define the m_colog attribute
         */
        void setColog(RTT::OperationCaller<void(RTT::LoggerLevel,std::string)> colog);

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
         * @param propNodeId : adresse du noeud master qu'il ne faut pas dispatcher
         * @param inBootUpReceived : port du CanOpenController connecté à la CanFestival pour recevoir
         * les bootup
         */
        void dispatchBootUp(nodeID_t propNodeId, RTT::InputPort<nodeID_t>& inBootUpReceived);

        /**
         * read the table of NMT states and dispatch change to registered nodes
         */
        void dispatchNmtState();

        /**
         * Unregister all registered nodes
         */
        void unRegisterAll();

        /**
         * Configure (in CanOpen meaning) all registered nodes
         */
        bool configureAll();

        /**
         * Trigger all registered nodes as slave peers (call update() )
         * This is the early cycle trigger
         */
        void triggerAllRead();

        /**
         * Trigger updateLate function of all registered nodes
         * This is the late cycle trigger
         */
        void triggerAllWrite();

        /**
         * Poll on CanOpenNode.outOperationnalState of each slave and wait that all reach expectedState
         */
        bool waitSlavesState(eRunningState expectedState, double timeout);

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
                RTT::OutputPort<bool> outBootUp;
                RTT::InputPort<eRunningState> inRunningState;
                CanOpenNode* task;
        };

    protected:
        /**
         * Contains a map of nodeRegistration to do the dispatch stuff
         */
        std::map< nodeID_t, nodeRegistration_t* > m_registeredNodes;

        /**
         * Component into we are operating
         */
        RTT::TaskContext& m_parent;

        /**
         * Handle on the logger
         */
        RTT::OperationCaller<void(RTT::LoggerLevel,std::string)> m_coLog;

    private:
        /** Utility function to help waitSlavesOperationnalState */
        eRunningState readStatePort(nodeRegistration_t* registration);
    };

}

#endif /* CANOPENDISPATCHER_HPP_ */
