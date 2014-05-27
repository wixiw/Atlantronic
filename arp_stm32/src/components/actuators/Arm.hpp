#ifndef ARM_HPP_
#define ARM_HPP_

#include "components/taskcontexts/Stm32TaskContext.hpp"
#include "linux/tools/robot_interface.h"
#include "ros/ros.h"

#include <arp_core/ArmCommandMsg.h>
#include <arp_core/ArmStatusMsg.h>

namespace arp_stm32
{
    class Arm: public Stm32TaskContext
    {
        public:
            Arm(const std::string& name);

            bool configureHook();
            void updateHook();

            RTT::OutputPort<MatrixHomogeneous> outArmPosition;
            RTT::OutputPort<arp_core::ArmStatusMsg> outArmStatus;
            RTT::InputPort<arp_core::ArmCommandMsg> inArmCommand;


        protected:
            /*
             * Interface STM32
             */
            void sendCmd(uint32_t cmdType);
            bool isActionDone();
            bool isArmStucked();

            /**
             * Interface Orocos
             */
            void createOrocosInterface();

            unsigned int attrLastCommand;
            bool attrActionDone;
            bool attrStucked;

            RobotInterface& m_robotItf;
            MatrixHomogeneous m_armMatrix;
    };

} /* namespace arp_stm32 */
#endif /* ARM_HPP_ */
