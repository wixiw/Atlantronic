#include <arp_master/boost/OrderActionGoal.h>
#include "ros_msg_transporter.hpp"
#include "RosLib.hpp"
#include <rtt/types/TransportPlugin.hpp>
#include <rtt/types/TypekitPlugin.hpp>

namespace ros_integration {
  using namespace RTT;
    struct ROSOrderActionGoalPlugin
      : public types::TransportPlugin
    {
      bool registerTransport(std::string name, types::TypeInfo* ti)
      {
	if(name == "//OrderActionGoal")
	  return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<arp_master::OrderActionGoal>());
	return false;
      }
      
      std::string getTransportName() const {
	return "ros";
      }
      
      std::string getTypekitName() const {
	return std::string("ros-")+"OrderActionGoal";
      }
      std::string getName() const {
	return std::string("rtt-ros-") + "OrderActionGoal" + "-transport";
      }

    };
}

ORO_TYPEKIT_PLUGIN( ros_integration::ROSOrderActionGoalPlugin )
