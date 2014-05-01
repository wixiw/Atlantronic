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

#include <arp_msgs/MatchDataMsg.h>
#include <std_msgs/Bool.h>

namespace arp_stm32
{

class MatchData: public Stm32TaskContext
{
    public:
        MatchData(const std::string& name);

        bool configureHook();
        void updateHook();

        //Start/Color
        /** Value of match data such as start, color, match time**/
        RTT::OutputPort<arp_msgs::MatchDataMsg> outMatchData;

        /** When set to true, the next start withdraw will be the start signal */
        RTT::InputPort<std_msgs::Bool> inReadyForMatch;

    protected:
        void createOrocosInterface();

        RobotInterface& m_robotItf;

        arp_msgs::MatchDataMsg attrMatchData;

        /** Informs the Stm32 we are ready for match ie next start withdraw is the match beginning.*/
        void setReadyForMatch();

};

} /* namespace arp_stm32 */
#endif /* MATCHDATA_HPP_ */
