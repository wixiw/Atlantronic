/*
 * KFLVariables.hpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#ifndef _ARP_RLU_KFL_KFLVARIABLES_HPP_
#define _ARP_RLU_KFL_KFLVARIABLES_HPP_

#include <math/math.hpp>

namespace arp_rlu
{

namespace kfl
{

typedef Eigen::Vector3d           StateVar;
typedef Eigen::Matrix<double,3,3> StateCov;

typedef Eigen::Vector2d           MeasVar;
typedef Eigen::Matrix<double,2,2> MeasCov;

typedef Eigen::Vector3d           SysInput;
typedef Eigen::Vector2d           MeasTarget;


} // namespace kfl
} // namespace arp_rlu

#endif /* _ARP_RLU_KFL_KFLVARIABLES_HPP_ */
