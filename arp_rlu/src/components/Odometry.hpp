/*
 * Odometry.hpp
 *
 *  Created on: Apr 5, 2012
 *      Author: ard
 */

#ifndef ODOMETRY_HPP_
#define ODOMETRY_HPP_

#include "RluTaskContext.hpp"
#include <math/core>

using namespace arp_math;

namespace arp_rlu
{

class Odometry: public RluTaskContext
{
    public:
    Odometry(const std::string& name);

    protected:
};

} /* namespace arp_rlu */
#endif /* ODOMETRY_HPP_ */
