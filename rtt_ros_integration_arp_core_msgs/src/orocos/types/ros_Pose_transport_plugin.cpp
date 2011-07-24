#include <arp_core/boost/Pose.h>
#include "ros_msg_transporter.hpp"
#include "RosLib.hpp"
#include <rtt/types/TransportPlugin.hpp>
#include <rtt/types/TypekitPlugin.hpp>

namespace ros_integration {
  using namespace RTT;
    struct ROSPosePlugin
      : public types::TransportPlugin
    {
      bool registerTransport(std::string name, types::TypeInfo* ti)
      {
	if(name == "/arp_core/Pose")
	  return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<arp_core::Pose>());
	return false;
      }
      
      std::string getTransportName() const {
	return "ros";
      }
      
      std::string getTypekitName() const {
	return std::string("ros-")+"Pose";
      }
      std::string getName() const {
	return std::string("rtt-ros-") + "Pose" + "-transport";
      }

    };
}

ORO_TYPEKIT_PLUGIN( ros_integration::ROSPosePlugin )
