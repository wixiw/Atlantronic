/*
 * Stm32Hmi.hpp
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

    class Stm32Hmi: public Stm32TaskContext
    {
        public:
            Stm32Hmi(const std::string& name);

            bool configureHook();
            void updateHook();
            void cleanupHook();

        protected:
            static void* task_wrapper(void* arg);
            void createOrocosInterface();

            RobotInterface& m_robotItf;

            static bool const NO_CMD_PROMPT = false;
    };

} /* namespace arp_stm32 */
#endif /* DISCOVERY_HPP_ */
