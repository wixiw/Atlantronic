/*
 * HMLTypeKit.hpp
 *
 *  Created on: 6 mars 2011
 *      Author: WLA
 */

#ifndef HMLTYPEKIT_HPP_
#define HMLTYPEKIT_HPP_

#include <rtt/types/TypekitPlugin.hpp>

namespace arp_hml
{
    class HMLTypeKit
       : public RTT::types::TypekitPlugin
    {
    public:
        virtual std::string getName();

        virtual bool loadTypes();
        virtual bool loadConstructors();
        virtual bool loadOperators();
    };

}

#endif /* HMLTYPEKIT_HPP_ */

