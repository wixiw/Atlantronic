#include <rtt/types/StructTypeInfo.hpp>
#include <orocos/can/CanOpenNode.hpp>

using namespace arp_hml;

  namespace boost {
    namespace serialization {
      // The helper function which you write yourself:
      template<class Archive>
      void serialize( Archive & a, eRunningState & cd, unsigned int) {
         using boost::serialization::make_nvp;
         a & make_nvp("state", cd);
      }
    }
  }

  // The RTT helper class which uses the above function behind the scenes:
struct CanNodeRunningStateTypeInfo: public RTT::types::StructTypeInfo<eRunningState,false>
{
    CanNodeRunningStateTypeInfo():
        RTT::types::StructTypeInfo<eRunningState,false>("CanNodeRunningState")
    {

    }

    //TODO WLA : je ne sais pas pourquoi ça ne marche pas avec true pour les ostream. Ca me casse les couilles j'ai pas que ça a faire
    //du coup c'est fait a la warrior.
    virtual std::ostream& write(std::ostream& os, base::DataSourceBase::shared_ptr in ) const
    {
        internal::DataSource<arp_hml::eRunningState>::shared_ptr data = boost::dynamic_pointer_cast< internal::DataSource<arp_hml::eRunningState> >( in );
        data->reset();
        data->evaluate();
        arp_hml::eRunningState state = data->get();
        std::string name;
        switch( state )
        {
            case arp_hml::PREOP:
                name = "PREOP";
                break;
            case arp_hml::OPERATIONAL:
                name = "OPERATIONAL";
                break;
            case arp_hml::IDLE:
                name = "IDLE";
                break;
            case arp_hml::UNCONNECTED:
                name = "UNCONNECTED";
                break;
            case arp_hml::UNKNOWN:
                name = "UNKNOWN";
                break;
            default:
                name = "Default can state switch";
                break;
        }

        return os << name;
    }
};



