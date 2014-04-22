/*
 * MatchData.hpp
 *
 *  Created on: 20 April 2014
 *      Author: wla
 */

#ifndef MATCHDATA_HPP_
#define MATCHDATA_HPP_

#include "components/taskcontexts/Stm32TaskContext.hpp"
#include "linux/tools/robot_interface.h"
#include "ros/ros.h"

#include <arp_core/Start.h>
#include <arp_core/StartColor.h>
#include <arp_core/EmptyWithSuccess.h>

namespace arp_stm32
{

class MatchData: public Stm32TaskContext
{
    public:
        MatchData(const std::string& name);

/****************************************************************
 * Interface Orocos
 ****************************************************************/

        bool configureHook();
        void updateHook();

        /**
         * Informs the stm32 that the next start withdraw sill be the match begining
         */
        bool ooEnableStart();

/****************************************************************
 * Interface ROS
 ****************************************************************/

        /**
         * ROS wrapper on the HmlMonitor.oo operation
         */
        bool srvEnableStart(arp_core::EmptyWithSuccess::Request& req, arp_core::EmptyWithSuccess::Response& res);

    protected:
        void createOrocosInterface();
        void createRosInterface();

        RobotInterface& m_robotItf;

        bool attrStartPlugged;
        bool attrStartColor;

        /**
         * Orocos Interface
         */

        //Start/Color
        /** Value of the start. GO is true when it is not in, go is false when the start is in **/
        RTT::OutputPort<arp_core::Start> outIoStart;
        /** Value of the color switch. true when ?? **/
        RTT::OutputPort<arp_core::StartColor> outIoStartColor;

};

} /* namespace arp_stm32 */
#endif /* MATCHDATA_HPP_ */
