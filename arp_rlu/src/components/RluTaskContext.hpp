/*
 * RluTaskContext.hpp
 *
 *  Created on: 5 Avril 2012
 *      Author: wla
 *
 */

#ifndef RLUTASKCONTEXT_HPP_
#define RLUTASKCONTEXT_HPP_

#include <taskcontexts/ARDTaskContext.hpp>

namespace arp_rlu
{

    class RluTaskContext: public arp_core::ARDTaskContext
    {
    public:
    	RluTaskContext(const std::string& name);
    };

}

#endif
