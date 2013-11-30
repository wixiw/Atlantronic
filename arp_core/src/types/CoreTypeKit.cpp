/*
 * CoreTypeKit.cpp
 *
 *  Created on: 02 April 2011
 *      Author: ard
 */

// typekit
#include <rtt/types/Types.hpp>
#include <rtt/types/TypekitPlugin.hpp>

// typeinfo
#include <rtt/types/TypeInfoRepository.hpp>
#include <rtt/types/SequenceTypeInfo.hpp>
#include <rtt/typekit/StdTypeInfo.hpp>
#include <rtt/types/TemplateConstructor.hpp>

#include "CoreTypeKit.hpp"
#include "Pose2DTypeInfo.hpp"
#include "Twist2DTypeInfo.hpp"
#include "ICRSpeedTypeInfo.hpp"
#include "EstimatedPose2DTypeInfo.hpp"
#include "EstimatedTwist2DTypeInfo.hpp"
#include "EstimatedICRSpeedTypeInfo.hpp"
#include "ParamsTypeInfo.hpp"
#include "TimeSpecTypeInfo.hpp"

using namespace RTT;
using namespace arp_core;

std::string CoreTypeKit::getName()
{
    return "CoreTypeKit";
}

bool CoreTypeKit::loadTypes()
{
    bool res = true;

    // Tell the RTT the name and type of this struct
    res &= types::Types()->addType( new RTT::types::TemplateTypeInfo< Eigen::Matrix<double, 3, 3> > ("Matrix3d") );
    res &= types::Types()->addType( new Pose2DTypeInfo() );
    res &= types::Types()->addType( new Twist2DTypeInfo() );
    res &= types::Types()->addType( new ICRSpeedTypeInfo() );
    res &= types::Types()->addType( new EstimatedPose2DTypeInfo() );
    res &= types::Types()->addType( new EstimatedTwist2DTypeInfo() );
    res &= types::Types()->addType( new EstimatedICRSpeedTypeInfo() );
    res &= types::Types()->addType( new ParamsTypeInfo() );
    res &= types::Types()->addType( new TimeSpecTypeInfo() );
    return res;
}

/** ...Add the other example code of this manual here as well... */
bool CoreTypeKit::loadConstructors()
{
    bool res = true;

    types::Types()->type("Pose2D")->addConstructor( types::newConstructor(&createPose2D) );
    types::Types()->type("Twist2D")->addConstructor( types::newConstructor(&createTwist2D) );
    types::Types()->type("ICRSpeed")->addConstructor( types::newConstructor(&createICRSpeed) );
    types::Types()->type("EstimatedPose2D")->addConstructor( types::newConstructor(&createEstimatedPose2D) );
    types::Types()->type("EstimatedTwist2D")->addConstructor( types::newConstructor(&createEstimatedTwist2D) );
    types::Types()->type("EstimatedICRSpeed")->addConstructor( types::newConstructor(&createEstimatedICRSpeed) );
    return res;
}

bool CoreTypeKit::loadOperators()
{
  // ...
    return true;
}


/** Register the class as a plugin */
ORO_TYPEKIT_PLUGIN( arp_core::CoreTypeKit );
