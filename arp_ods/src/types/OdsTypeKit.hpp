/*
 * OdsTypeKit.hpp
 *
 *  Created on: 6 mars 2011
 *      Author: WLA
 */

#ifndef OdsTYPEKIT_HPP_
#define OdsTYPEKIT_HPP_

#include <rtt/types/TypekitPlugin.hpp>

namespace arp_ods
{
    class OdsTypeKit
       : public RTT::types::TypekitPlugin
    {
    public:
        virtual std::string getName();

        virtual bool loadTypes();
        virtual bool loadConstructors();
        virtual bool loadOperators();
    };

}

#endif /* OdsTYPEKIT_HPP_ */

