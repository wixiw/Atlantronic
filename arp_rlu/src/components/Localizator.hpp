/*
 * Localizator.hpp
 *
 *  Created on: Apr 5, 2012
 *      Author: ard
 */

#ifndef LOCALIZATOR_HPP_
#define LOCALIZATOR_HPP_

#include "RluTaskContext.hpp"
#include <math/core>

using namespace arp_math;

namespace arp_rlu
{

class Localizator: public RluTaskContext
{
    public:
    Localizator(const std::string& name);

    protected:
};

} /* namespace arp_rlu */
#endif /* LOCALIZATOR_HPP_ */
