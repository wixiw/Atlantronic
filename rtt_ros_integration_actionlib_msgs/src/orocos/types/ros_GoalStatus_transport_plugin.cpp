#include <actionlib_msgs/boost/GoalStatus.h>
#include "ros_msg_transporter.hpp"
#include "RosLib.hpp"
#include <rtt/types/TransportPlugin.hpp>
#include <rtt/types/TypekitPlugin.hpp>

namespace ros_integration {
  using namespace RTT;
    struct ROSGoalStatusPlugin
      : public types::TransportPlugin
    {
      bool registerTransport(std::string name, types::TypeInfo* ti)
      {
	if(name == "/actionlib_msgs/GoalStatus")
	  return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<actionlib_msgs::GoalStatus>());
	return false;
      }
      
      std::string getTransportName() const {
	return "ros";
      }
      
      std::string getTypekitName() const {
	return std::string("ros-")+"GoalStatus";
      }
      std::string getName() const {
	return std::string("rtt-ros-") + "GoalStatus" + "-transport";
      }

    };
}

ORO_TYPEKIT_PLUGIN( ros_integration::ROSGoalStatusPlugin )
