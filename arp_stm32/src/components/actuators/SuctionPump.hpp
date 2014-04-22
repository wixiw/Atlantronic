/*
 * SuctionPump.hpp
 *
 *  Created on: 20 April 2014
 *      Author: wla
 */

#ifndef SUCTIONPUMP_HPP_
#define SUCTIONPUMP_HPP_

#include "components/taskcontexts/Stm32TaskContext.hpp"
#include "linux/tools/robot_interface.h"
#include "ros/ros.h"

namespace arp_stm32
{

class SuctionPump: public Stm32TaskContext
{
    public:
        SuctionPump(const std::string& name);

        bool configureHook();
        void updateHook();

        /**
         * Set the suction power of the pump in % of max value
         */
        RTT::InputPort<double> inSuctionPowerCmd;

        /**
         * Inform if an object is present in the suction cup (detected by current overload)
         */
        RTT::OutputPort<bool> outObjectPresent;

    protected:
        void createOrocosInterface();

        RobotInterface& m_robotItf;

        bool attrObjectPresent;

        /** Unique identificator of the pump in RobotInterface*/
        int propPumpId;

        /** Write concretely to robot_interface. Returns true on success */
        void setSuctionPower(int power);

        /** Read concretely to robot_interface. the mutex has to be previously taken. Returns the value */
        bool getObjectPresent();

};

} /* namespace arp_stm32 */
#endif /* SUCTIONPUMP_HPP_ */
