/*
 * HmlTaskContext.hpp
 *
 *  Created on: 13 April 2012
 *      Author: wla
 *
 */

#ifndef MASTERTASKCONTEXT_HPP_
#define MASTERTASKCONTEXT_HPP_

#include <taskcontexts/ARDTaskContext.hpp>

namespace arp_master
{

    class MasterTaskContext: public arp_core::ARDTaskContext
    {
    public:
        MasterTaskContext(const std::string& name);
    };

}

#endif
