#include <rtt/types/StructTypeInfo.hpp>
#include <canfestival/data.h>

using namespace arp_hml;

  namespace boost {
    namespace serialization {
      // The helper function which you write yourself:
      template<class Archive>
      void serialize( Archive & a, enum_nodeState & cd, unsigned int) {
         using boost::serialization::make_nvp;
         a & make_nvp("state", cd);
      }
    }
  }


  // Displaying:
  std::ostream& operator<<(std::ostream& os, const enum_nodeState& cd)
  {
      std::string name;

     switch( cd )
     {
         case Initialisation:
             name = "Initialisation";
             break;
         case Pre_operational:
             name = "Pre-Operational";
             break;
         case Operational:
             name = "Operational";
             break;
         case Stopped:
             name = "Stopped";
             break;
         case Unknown_state:
             name = "Unknown";
             break;
         default:
             name = "Default can state switch";
             break;
     }
      return os << name;
  }

  // Reading :
  std::istream& operator>>(std::istream& is, enum_nodeState& cd) {
     return is >> cd;
  }



  // The RTT helper class which uses the above function behind the scenes:
struct NMTStateTypeInfo: public RTT::types::StructTypeInfo<enum_nodeState,false>
{
    NMTStateTypeInfo():
        RTT::types::StructTypeInfo<enum_nodeState,false>("NMTState")
    {

    }
};

