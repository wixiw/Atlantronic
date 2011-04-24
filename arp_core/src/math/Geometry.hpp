/*
 * Pose.hpp
 *
 *  Created on: 10 sept. 2010
 *      Author: boris
 */

#ifndef _ARPMATH_GEOMETRY_HPP_
#define _ARPMATH_GEOMETRY_HPP_

#include "math/math.hpp"
#include <Eigen/Geometry>

namespace arp_math
{
    typedef Eigen::Matrix<double, 2, 1> Vector2;
    typedef Eigen::Matrix<double, 3, 1> Vector3;
    typedef Eigen::Rotation2D<double>	Rotation2;
}

#endif /* _ARPMATH_GEOMETRY_HPP_ */
