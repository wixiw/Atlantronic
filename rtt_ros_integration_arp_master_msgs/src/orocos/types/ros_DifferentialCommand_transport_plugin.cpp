#include <arp_master/boost/DifferentialCommand.h>
#include "ros_msg_transporter.hpp"
#include "RosLib.hpp"
#include <rtt/types/TransportPlugin.hpp>
#include <rtt/types/TypekitPlugin.hpp>

namespace ros_integration {
  using namespace RTT;
    struct ROSDifferentialCommandPlugin
      : public types::TransportPlugin
    {
      bool registerTransport(std::string name, types::TypeInfo* ti)
      {
	if(name == "//DifferentialCommand")
	  return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<arp_master::DifferentialCommand>());
	return false;
      }
      
      std::string getTransportName() const {
	return "ros";
      }
      
      std::string getTypekitName() const {
	return std::string("ros-")+"DifferentialCommand";
      }
      std::string getName() const {
	return std::string("rtt-ros-") + "DifferentialCommand" + "-transport";
      }

    };
}

ORO_TYPEKIT_PLUGIN( ros_integration::ROSDifferentialCommandPlugin )
