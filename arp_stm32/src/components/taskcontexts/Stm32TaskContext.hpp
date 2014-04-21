/*
 * Stm32TaskContext.hpp
 *
 *  Created on: 26 Jan 2014
 *      Author: wla
 *
 */

#ifndef RLUTASKCONTEXT_HPP_
#define RLUTASKCONTEXT_HPP_

#include <taskcontexts/ARDTaskContext.hpp>

namespace arp_stm32
{

class Stm32TaskContext: public arp_core::ARDTaskContext
{
    public:
        Stm32TaskContext(const std::string& name);
};

}

#endif
