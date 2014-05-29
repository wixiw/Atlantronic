#ifndef HOKUYO_ITF_HPP_
#define HOKUYO_ITF_HPP_

#include "components/taskcontexts/Stm32TaskContext.hpp"
#include "linux/tools/robot_interface.h"
#include "ros/ros.h"

#include <std_msgs/Bool.h>

namespace arp_stm32
{

class HokuyoItf: public Stm32TaskContext
{
    public:
        HokuyoItf(const std::string& name);

        /****************************************************************
         * Interface Orocos
         ****************************************************************/

        bool configureHook();
        void updateHook();

        RTT::OutputPort<hokuyo_scan> outScan;

        int propHokuyoId;

        /****************************************************************
         * Interface ROS
         ****************************************************************/

    protected:
        void createOrocosInterface();

        RobotInterface& m_robotItf;
};

} /* namespace arp_stm32 */
#endif /* HOKUYO_ITF_HPP_ */
