#include <arp_master/boost/OrderResult.h>
#include "ros_msg_transporter.hpp"
#include "RosLib.hpp"
#include <rtt/types/TransportPlugin.hpp>
#include <rtt/types/TypekitPlugin.hpp>

namespace ros_integration {
  using namespace RTT;
    struct ROSOrderResultPlugin
      : public types::TransportPlugin
    {
      bool registerTransport(std::string name, types::TypeInfo* ti)
      {
	if(name == "//OrderResult")
	  return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<arp_master::OrderResult>());
	return false;
      }
      
      std::string getTransportName() const {
	return "ros";
      }
      
      std::string getTypekitName() const {
	return std::string("ros-")+"OrderResult";
      }
      std::string getName() const {
	return std::string("rtt-ros-") + "OrderResult" + "-transport";
      }

    };
}

ORO_TYPEKIT_PLUGIN( ros_integration::ROSOrderResultPlugin )
