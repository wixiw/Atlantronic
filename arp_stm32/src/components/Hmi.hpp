/*
 * Hmi.hpp
 *
 *  Created on: 21 April 2014
 *      Author: jbt
 */

#ifndef HMI_HPP_
#define HMI_HPP_

#include "components/taskcontexts/Stm32TaskContext.hpp"
#include <components/MotionScheduler.hpp>
#include "linux/tools/robot_interface.h"
#include "ros/ros.h"

namespace arp_stm32
{

    class Hmi: public arp_core::MotionScheduler
    {
        public:
            Hmi(const std::string& name);

            bool configureHook();
            void updateHook();
            void cleanupHook();

        protected:
            static void robotItfCallbackWrapper(void* arg);
            static void* task_wrapper(void* arg);
            void createOrocosInterface();

            RobotInterface& m_robotItf;
    };

} /* namespace arp_stm32 */
#endif /* DISCOVERY_HPP_ */
