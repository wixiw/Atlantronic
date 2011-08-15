/*
 * SimplePCM3362.hpp
 *
 *  Created on: 2 nov. 2010
 *      Author: ard
 */

#ifndef SimplePCM3362_HPP
#define SimplePCM3362_HPP

//include orocos
#include "orocos/taskcontexts/HmlTaskContext.hpp"

using namespace arp_core;


namespace arp_hml
{

    class SimplePCM3362 : public HmlTaskContext
    {
    public:
    	SimplePCM3362(const std::string& name);
        ~SimplePCM3362();
    };

}

#endif /* PCM3362_HPP_ */
