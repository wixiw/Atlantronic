#ifndef PARAMSTYPEINFO_HPP_
#define PARAMSTYPEINFO_HPP_

#include <rtt/types/StructTypeInfo.hpp>
#include "models/UbiquityParams.hpp"
#include <ostream>
#include <istream>
#include <boost/lexical_cast.hpp>

using namespace arp_math;
using namespace arp_model;
using namespace arp_core;
using namespace RTT;

  namespace boost {
    namespace serialization {
      // The helper function which you write yourself:
      template<class Archive>
      void serialize( Archive & a, UbiquityParams& params, unsigned int) {
         using boost::serialization::make_nvp;
             a & make_nvp("leftTurretZero",         params.getLeftTurretZeroRef());
             a & make_nvp("rightTurretZero",        params.getRightTurretZeroRef());
             a & make_nvp("rearTurretZero",         params.getRearTurretZeroRef());
             a & make_nvp("leftTurretPosition",     params.getLeftTurretPositionRef());
             a & make_nvp("rightTurretPosition",    params.getRightTurretPositionRef());
             a & make_nvp("rearTurretPosition",     params.getRearTurretPositionRef());
             a & make_nvp("chassisCenter",          params.getChassisCenterRef());
             a & make_nvp("leftWheelDiameter",      params.getLeftWheelDiameterRef());
             a & make_nvp("rightWheelDiameter",     params.getRightWheelDiameterRef());
             a & make_nvp("rearWheelDiameter",      params.getRearWheelDiameterRef());
             a & make_nvp("turretRatio",            params.getTurretRatioRef());
             a & make_nvp("tractionRatio",          params.getTractionRatioRef());
             a & make_nvp("minDrivingSpeed",       params.getMinDrivingSpeedRef());
             a & make_nvp("maxDrivingSpeed",       params.getMaxDrivingSpeedRef());
             a & make_nvp("maxSteeringSpeed",         params.getMaxSteeringSpeedRef());
             a & make_nvp("maxDrivingAcc",         params.getMaxDrivingAccRef());
             a & make_nvp("maxSteeringAcc",           params.getMaxSteeringAccRef());
             a & make_nvp("maxDrivingTorque",      params.getMaxDrivingTorqueRef());
             a & make_nvp("maxSteeringTorque",        params.getMaxSteeringTorqueRef());
             a & make_nvp("maxRobotSpeed",      params.getMaxRobotSpeedRef());
             a & make_nvp("maxRobotAccel",        params.getMaxRobotAccelRef());
             a & make_nvp("maxRobotJerk",        params.getMaxRobotJerkRef());
      }
    }
  }

  // The RTT helper class which uses the above function behind the scenes:
struct ParamsTypeInfo: public RTT::types::StructTypeInfo<UbiquityParams,false>
{
    ParamsTypeInfo():
        RTT::types::StructTypeInfo<UbiquityParams,false>("UbiquityParams")
    {

    }
};

#endif
