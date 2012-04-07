/*
 * OdsTaskContext.hpp
 *
 *  Created on: 12 février 2012
 *      Author: wla
 *
 */

#ifndef ODSTASKCONTEXT_HPP_
#define ODSTASKCONTEXT_HPP_

#include <taskcontexts/ARDTaskContext.hpp>

namespace arp_ods
{

    class OdsTaskContext: public arp_core::ARDTaskContext
    {
    public:
    	OdsTaskContext(const std::string& name);
    };

}

#endif
