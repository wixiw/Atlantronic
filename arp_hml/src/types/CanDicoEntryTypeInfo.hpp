#include <rtt/types/StructTypeInfo.hpp>
#include "orocos/can/ard_can_types.hpp"

using namespace arp_hml;

  namespace boost {
    namespace serialization {
      // The helper function which you write yourself:
      template<class Archive>
      void serialize( Archive & a, CanDicoEntry & cd, unsigned int) {
         using boost::serialization::make_nvp;
         a & make_nvp("nodeId", cd.nodeId);
         a & make_nvp("index", cd.index);
         a & make_nvp("subindex", cd.subindex);
         a & make_nvp("value", cd.value);
         a & make_nvp("dataType", cd.dataType);
         a & make_nvp("size", cd.size);
      }
    }
  }


  // Displaying:
  std::ostream& operator<<(std::ostream& os, const arp_hml::CanDicoEntry& cd)
  {
      return os << "(id=" << cd.nodeId << ",index=" << cd.index << ",subindex" << cd.subindex << ",value=" << cd.value << ")";
  }

  // Reading :
  std::istream& operator>>(std::istream& is, arp_hml::CanDicoEntry& cd) {
     return is >> cd;
  }

  CanDicoEntry createCanDicoEntry(int nodeId, int index, int subindex, int value, int dataType, int size)
  {
      return CanDicoEntry(nodeId,index,subindex,value,dataType,size);
  }

  // The RTT helper class which uses the above function behind the scenes:
struct CanDicoEntryTypeInfo: public RTT::types::StructTypeInfo<CanDicoEntry,false>
{
    CanDicoEntryTypeInfo():
        RTT::types::StructTypeInfo<CanDicoEntry,false>("CanDicoEntry")
    {

    }
};

