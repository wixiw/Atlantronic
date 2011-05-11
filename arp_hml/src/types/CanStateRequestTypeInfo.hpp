#include <rtt/types/StructTypeInfo.hpp>
#include "orocos/can/ard_can_types.hpp"

  namespace boost {
    namespace serialization {
      // The helper function which you write yourself:
      template<class Archive>
      void serialize( Archive & a, enum_DS301_nmtStateRequest & cd, unsigned int) {
         using boost::serialization::make_nvp;
         a & make_nvp("stateRequest", cd);
      }
    }
  }


  // Displaying:
  std::ostream& operator<<(std::ostream& os, const enum_DS301_nmtStateRequest& cd)
  {
      string name;

     switch( cd )
     {
         case StartNode:
             name = "StartNode";
             break;
         case StopNode:
             name = "StopNode";
             break;
         case EnterPreOp:
             name = "EnterPreOp";
             break;
         case ResetNode:
             name = "ResetNode";
             break;
         case ResetComunication:
             name = "ResetComunication";
             break;
         default:
             name = "Default can state request switch";
             break;
     }
      return os << name;
  }

  // Reading :
  std::istream& operator>>(std::istream& is, enum_DS301_nmtStateRequest& cd)
  {
      return is >> cd;
  }

  enum_DS301_nmtStateRequest createNmtStateRequest(string s)
  {
      enum_DS301_nmtStateRequest res;

      if( s == "StopNode" )
          res = StopNode;
      else if( s == "StartNode" )
          res = StartNode;
      else if( s == "ResetNode" )
          res = ResetNode;
      else if( s == "ResetComunication" )
           res = ResetComunication;
      else if( s == "EnterPreOp" )
           res = EnterPreOp;
      else
          res = UnknownRequest;

      return res;
  }

  // The RTT helper class which uses the above function behind the scenes:
struct NMTStateRequestTypeInfo: public RTT::types::StructTypeInfo<enum_DS301_nmtStateRequest,true>
{
    NMTStateRequestTypeInfo():
        RTT::types::StructTypeInfo<enum_DS301_nmtStateRequest,true>("NMTStateRequest")
    {

    }
};

