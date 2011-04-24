
#include <arp_master/DifferentialCommand.h>
#include <arp_master/Obstacle.h>
#include <arp_master/StartColor.h>
#include <arp_master/Start.h>
#include <arp_master/Velocity.h>
#include <arp_master/Odo.h>
#include <arp_master/Pose.h>
#include <arp_master/OrderAction.h>
#include <arp_master/OrderGoal.h>
#include <arp_master/OrderActionGoal.h>
#include <arp_master/OrderResult.h>
#include <arp_master/OrderActionResult.h>
#include <arp_master/OrderFeedback.h>
#include <arp_master/OrderActionFeedback.h>

#include "ros_msg_transporter.hpp"
#include "RosLib.hpp"
#include <rtt/types/TransportPlugin.hpp>
#include <rtt/types/TypekitPlugin.hpp>

namespace ros_integration {
  using namespace RTT;
    struct ROSarp_masterPlugin
      : public types::TransportPlugin
    {
      bool registerTransport(std::string name, types::TypeInfo* ti)
      {
                   if(name == "/arp_master/DifferentialCommand")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<arp_master::DifferentialCommand>());
         if(name == "/arp_master/Obstacle")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<arp_master::Obstacle>());
         if(name == "/arp_master/StartColor")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<arp_master::StartColor>());
         if(name == "/arp_master/Start")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<arp_master::Start>());
         if(name == "/arp_master/Velocity")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<arp_master::Velocity>());
         if(name == "/arp_master/Odo")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<arp_master::Odo>());
         if(name == "/arp_master/Pose")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<arp_master::Pose>());
         if(name == "/arp_master/OrderAction")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<arp_master::OrderAction>());
         if(name == "/arp_master/OrderGoal")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<arp_master::OrderGoal>());
         if(name == "/arp_master/OrderActionGoal")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<arp_master::OrderActionGoal>());
         if(name == "/arp_master/OrderResult")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<arp_master::OrderResult>());
         if(name == "/arp_master/OrderActionResult")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<arp_master::OrderActionResult>());
         if(name == "/arp_master/OrderFeedback")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<arp_master::OrderFeedback>());
         if(name == "/arp_master/OrderActionFeedback")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<arp_master::OrderActionFeedback>());

          return false;
      }
      
      std::string getTransportName() const {
          return "ros";
      }
      
      std::string getTypekitName() const {
          return std::string("ros-")+"arp_master";
      }
      std::string getName() const {
          return std::string("rtt-ros-") + "arp_master" + "-transport";
      }

    };
}

ORO_TYPEKIT_PLUGIN( ros_integration::ROSarp_masterPlugin )
