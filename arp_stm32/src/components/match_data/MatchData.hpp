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

#include <std_msgs/Bool.h>
#include <arp_core/Start.h>
#include <arp_core/StartColor.h>
#include <arp_core/EmptyWithSuccess.h>

namespace arp_stm32
{

class MatchData: public Stm32TaskContext
{
    public:
        MatchData(const std::string& name);

        bool configureHook();
        void updateHook();


        /** When set to true, the next start withdraw will be the start signal */
        RTT::InputPort<std_msgs::Bool> inReadyForMatch;

    protected:
        void createOrocosInterface();

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

        /** Informs the Stm32 we are ready for match ie next start withdraw is the match beginning.*/
        void setReadyForMatch();
};

} /* namespace arp_stm32 */
#endif /* MATCHDATA_HPP_ */
