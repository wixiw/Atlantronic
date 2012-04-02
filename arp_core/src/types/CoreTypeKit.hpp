/*
 * CoreTypeKit.hpp
 *
 *  Created on: 02 April 2011
 *      Author: WLA
 */

#ifndef CORETYPEKIT_HPP_
#define CORETYPEKIT_HPP_

#include <rtt/types/TypekitPlugin.hpp>
#include <iostream>

namespace arp_core
{
    class CoreTypeKit
       : public RTT::types::TypekitPlugin
    {
    public:
        virtual std::string getName();

        virtual bool loadTypes();
        virtual bool loadConstructors();
        virtual bool loadOperators();
    };

}

#endif /*  CORETYPEKIT_HPP_ */

