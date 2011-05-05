#include <rtt/types/StructTypeInfo.hpp>
#include "hml/can/ard_DS402.hpp"

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

  // Reading :
  std::istream& operator>>(std::istream& is, ArdDs402::enum_DS402_state& cd) {
     return is >> cd;
  }



  // The RTT helper class which uses the above function behind the scenes:
struct DS402StateTypeInfo: public RTT::types::StructTypeInfo<ArdDs402::enum_DS402_state,true>
{
    DS402StateTypeInfo():
        RTT::types::StructTypeInfo<ArdDs402::enum_DS402_state,true>("DS402State")
    {

    }
};

