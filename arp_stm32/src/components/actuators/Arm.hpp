#ifndef ARM_HPP_
#define ARM_HPP_

#include "components/taskcontexts/Stm32TaskContext.hpp"
#include "linux/tools/robot_interface.h"
#include "ros/ros.h"

namespace arp_stm32
{
    class Arm: public Stm32TaskContext
    {
        public:
            Arm(const std::string& name);

            bool configureHook();
            void updateHook();

            RTT::OutputPort<MatrixHomogeneous> outArmPosition;

        protected:
            void createOrocosInterface();
            void sendCmd(uint32_t cmdType);

            RobotInterface& m_robotItf;
            MatrixHomogeneous m_armMatrix;
    };

} /* namespace arp_stm32 */
#endif /* ARM_HPP_ */
