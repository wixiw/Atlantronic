/*
 * Hmi.hpp
 *
 *  Created on: 21 April 2014
 *      Author: jbt
 */

#ifndef HMI_HPP_
#define HMI_HPP_

#include "components/taskcontexts/Stm32TaskContext.hpp"
#include "linux/tools/robot_interface.h"
#include "math/core"


namespace arp_stm32
{

    class Hmi: public Stm32TaskContext
    {
        public:
            Hmi(const std::string& name);

            bool configureHook();
            void updateHook();
            void cleanupHook();

            RTT::InputPort<arp_math::EstimatedPose2D> inEstimatedPose;

        protected:
            static void* task_wrapper(void* arg);
            void createOrocosInterface();

            RobotInterface& m_robotItf;
    };

} /* namespace arp_stm32 */
#endif /* DISCOVERY_HPP_ */
