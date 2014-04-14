#include <rtt/types/StructTypeInfo.hpp>
#include "orocos/can/ard_DS402.hpp"

using namespace arp_hml;

  namespace boost {
    namespace serialization {
      // The helper function which you write yourself:
      template<class Archive>
      void serialize( Archive & a, ArdDs402::enum_DS402_state & cd, unsigned int) {
         using boost::serialization::make_nvp;
         a & make_nvp("DS402State", cd);
      }
    }
  }


  // Displaying:
  std::ostream& operator<<(std::ostream& os, const ArdDs402::enum_DS402_state& cd)
  {
      string name;

     switch( cd )
     {
         case ArdDs402::NotReadyToSwitchOn:
             name = "NotReadyToSwitchOn";
             break;
         case ArdDs402::SwitchOnDisabled:
             name = "SwitchOnDisabled";
             break;
         case ArdDs402::ReadyToSwitchOn:
             name = "ReadyToSwitchOn";
             break;
         case ArdDs402::SwitchedOn:
             name = "SwitchedOn";
             break;
         case ArdDs402::OperationEnable:
             name = "OperationEnable";
             break;
         case ArdDs402::QuickStopActive:
             name = "QuickStopActive";
             break;
         case ArdDs402::FaultReactionActive:
             name = "FaultReactionActive";
             break;
         case ArdDs402::Fault:
             name = "Fault";
             break;
         case ArdDs402::UnknownDs402State:
             name = "UnknownDs402State";
             break;
         default:
             name = "Default ds402 switch";
             break;
     }
      return os << name;
  }

  // The RTT helper class which uses the above function behind the scenes:
struct DS402StateTypeInfo: public RTT::types::StructTypeInfo<ArdDs402::enum_DS402_state,false>
{
    DS402StateTypeInfo():
        RTT::types::StructTypeInfo<ArdDs402::enum_DS402_state,false>("DS402State")
    {

    }

    //TODO WLA : je ne sais pas pourquoi ça ne marche pas avec true pour les ostream. Ca me casse les couilles j'ai pas que ça a faire
    //du coup c'est fait a la warrior.
    virtual std::ostream& write(std::ostream& os, base::DataSourceBase::shared_ptr in ) const
    {
//        internal::DataSource<ArdDs402::enum_DS402_state>::shared_ptr data = boost::dynamic_pointer_cast< internal::DataSource<ArdDs402::enum_DS402_state> >( in );
//        data->reset();
//        data->evaluate();
//        ArdDs402::enum_DS402_state state;
//        state = data->get();
//        return os << ArdDs402::toString(state) << " seconds";
        return os;
    }
};


