#include <rtt/types/StructTypeInfo.hpp>
#include "models/UbiquityParams.hpp"
#include <ostream>
#include <istream>
#include <boost/lexical_cast.hpp>

using namespace arp_math;
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
             a & make_nvp("maxTractionSpeed",       params.getMaxTractionSpeedRef());
             a & make_nvp("maxTurretSpeed",         params.getMaxTurretSpeedRef());
             a & make_nvp("maxTractionAcc",         params.getMaxTractionAccRef());
             a & make_nvp("maxTurretAcc",           params.getMaxTurretAccRef());
             a & make_nvp("maxTractionDec",         params.getMaxTractionDecRef());
             a & make_nvp("maxTurretDec",           params.getMaxTurretDecRef());
             a & make_nvp("maxTractionTorque",      params.getMaxTractionTorqueRef());
             a & make_nvp("maxTurretTorque",        params.getMaxTurretTorqueRef());
      }
    }
  }


//  // Displaying:
//  std::ostream& operator<<(std::ostream& os, const Pose2D& pose)
//  {
//      return os << "(" << pose.x() << "," << pose.y() << "," << pose.angle() << ")";
//  }
//
//  // Reading :
//  std::istream& operator>>(std::istream& is, Pose2D& cd) {
//      char c;
//     return is >> c >> cd;
//  }


  // The RTT helper class which uses the above function behind the scenes:
struct ParamsTypeInfo: public RTT::types::StructTypeInfo<UbiquityParams,false>
{
    ParamsTypeInfo():
        RTT::types::StructTypeInfo<UbiquityParams,false>("UbiquityParams")
    {

    }
};

