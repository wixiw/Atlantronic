/*
 * StateManager.hpp
 *
 *  Created on: 12 fev 2012
 *      Author: ard, wla
 */

#ifndef STATE_MANAGER_HPP_
#define STATE_MANAGER_HPP_

#include <taskcontexts/ARDTaskContext.hpp>

namespace arp_hml
{
    /** surcharge du log */
    #define LOGS(level) RTT::log(m_owner.propEnableLog?level:RTT::Never)<<"["<<m_owner.getName()<<"] "

    /** \ingroup ARP-arp_hml
     *
     * \class StateManager
     *
     * Handle state of motoor on the ubitquity robot
     */
    class StateManager
    {
        public:

            /** */
            StateManager(arp_core::ARDTaskContext& c);

            /**
             * Get the operation on motors
             */
            bool configureHook();

            /**
             */
            void updateHook();

        protected:

            /** */
            arp_core::ARDTaskContext& m_owner;

            RTT::Property<bool>* m_propRequireCompleteHardware;


            /** Pointer in the Left driving ooSetOperationMode Operation **/
            RTT::OperationCaller<bool(std::string)> m_ooSetLeftDrivingOperationMode;
            /** Pointer in the Right driving ooSetOperationMode Operation **/
            RTT::OperationCaller<bool(std::string)> m_ooSetRightDrivingOperationMode;
            /** Pointer in the Right driving ooSetOperationMode Operation **/
            RTT::OperationCaller<bool(std::string)> m_ooSetRearDrivingOperationMode;
            /** Pointer in the Left steering ooSetOperationMode Operation **/
            RTT::OperationCaller<bool(std::string)> m_ooSetLeftSteeringOperationMode;
            /** Pointer in the Right steering ooSetOperationMode Operation **/
            RTT::OperationCaller<bool(std::string)> m_ooSetRightSteeringOperationMode;
            /** Pointer in the Right steering ooSetOperationMode Operation **/
            RTT::OperationCaller<bool(std::string)> m_ooSetRearSteeringOperationMode;


            /** Choose a mode of operation on driving motors
             * @param state : choose between spped,position,torque,other
             * @return true only if the 3 driving motors switched to the correct state.
             */
            bool ooSetDrivingOperationMode(const std::string state);

            /** Choose a mode of operation on sttering motors
             * @param state : choose between spped,position,torque,other
             * @return true only if the 3 steering motors switched to the correct state.
             */
            bool ooSetSteeringOperationMode(const std::string state);

            /**
             * Use this to connect local handlers to all peer component operations.
             */
            bool getPeersOperations();
    };
}

#endif
