/*
 * KFLVariables.hpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#ifndef _ARP_RLU_KFL_KFLVARIABLES_HPP_
#define _ARP_RLU_KFL_KFLVARIABLES_HPP_

#include <math/core>

namespace arp_rlu
{

namespace kfl
{

typedef Eigen::Vector3d           KFLStateVar;
typedef Eigen::Matrix<double,3,3> KFLStateCov;

typedef Eigen::Vector2d           KFLMeasVar;
typedef Eigen::Matrix<double,2,2> KFLMeasCov;

typedef Eigen::Vector3d           KFLSysInput;
typedef Eigen::Vector2d           KFLMeasTarget;


} // namespace kfl
} // namespace arp_rlu

#endif /* _ARP_RLU_KFL_KFLVARIABLES_HPP_ */
