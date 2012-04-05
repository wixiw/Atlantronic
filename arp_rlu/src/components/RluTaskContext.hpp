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

using namespace arp_core;

namespace arp_rlu
{

    class RluTaskContext: public ARDTaskContext
    {
    public:
    	RluTaskContext(const std::string& name);
    };

}

#endif
