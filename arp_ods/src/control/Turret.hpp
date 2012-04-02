/*
 * Turret.hpp
 *
 *  Created on: 12 février 2012
 *      Author: wla
 *
 */

#ifndef TURRET_HPP_
#define TURRET_HPP_

#include "taskcontexts/OdsTaskContext.hpp"

using namespace arp_core;

namespace arp_ods
{

    class Turret: public OdsTaskContext
    {
    public:
        Turret(const std::string& name);
    };

}

#endif
