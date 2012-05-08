#include <rtt/types/StructTypeInfo.hpp>
#include "control/orders/OrderConfig.hpp"
#include <ostream>
#include <istream>
#include <boost/lexical_cast.hpp>

using namespace arp_ods;
using namespace orders;
using namespace RTT;

  namespace boost {
    namespace serialization {
      // The helper function which you write yourself:
      template<class Archive>
      void serialize( Archive & a, config& conf, unsigned int) {
         using boost::serialization::make_nvp;
             a & make_nvp("RADIUS_APPROACH_ZONE",   conf.RADIUS_APPROACH_ZONE);
             a & make_nvp("DISTANCE_ACCURACY",      conf.DISTANCE_ACCURACY );
             a & make_nvp("ANGLE_ACCURACY",         conf.ANGLE_ACCURACY );
             a & make_nvp("PASS_TIMEOUT",           conf.PASS_TIMEOUT );
             a & make_nvp("ORDER_TIMEOUT",          conf.ORDER_TIMEOUT );
             a & make_nvp("VEL_FINAL",              conf.VEL_FINAL );
             a & make_nvp("LIN_VEL_MAX",            conf.LIN_VEL_MAX );
             a & make_nvp("ANG_VEL_MAX",            conf.ANG_VEL_MAX );
             a & make_nvp("LIN_DEC",                conf.LIN_DEC );
             a & make_nvp("ANG_DEC",                conf.ANG_DEC );
      }
    }
  }


  // The RTT helper class which uses the above function behind the scenes:
struct OrderConfigTypeInfo: public RTT::types::StructTypeInfo<config,false>
{
    OrderConfigTypeInfo():
        RTT::types::StructTypeInfo<config,false>("OrderConfig")
    {

    }
};

