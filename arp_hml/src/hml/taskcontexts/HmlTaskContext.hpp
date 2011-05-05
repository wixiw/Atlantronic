/*
 * HmlTaskContext.hpp
 *
 *  Created on: 04 mai 2011
 *      Author: wla
 *
 */

#ifndef HMLTASKCONTEXT_HPP_
#define HMLTASKCONTEXT_HPP_

#include <taskcontexts/ARDTaskContext.hpp>

using namespace arp_core;

namespace arp_hml
{

    class HmlTaskContext: public ARDTaskContext
    {
    public:
    	HmlTaskContext(const std::string& name);
    };

}

#endif
