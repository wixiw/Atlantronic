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
             a & make_nvp("rightTurretZero",        params.getRearTurretZeroRef());
             a & make_nvp("rearTurretZero",         params.getRightTurretZeroRef());
             a & make_nvp("leftTurretPosition",     params.getLeftTurretPositionRef());
             a & make_nvp("rightTurretPosition",    params.getRearTurretPositionRef());
             a & make_nvp("rearTurretPosition",     params.getRightTurretPositionRef());
             a & make_nvp("leftWheelDiameter",      params.getLeftWheelDiameterRef());
             a & make_nvp("rightWheelDiameter",     params.getRearWheelDiameterRef());
             a & make_nvp("rearWheelDiameter",      params.getRightWheelDiameterRef());
             a & make_nvp("turretRatio",            params.getRightWheelDiameterRef());
             a & make_nvp("tractionRatio",          params.getTurretRatioRef());
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

