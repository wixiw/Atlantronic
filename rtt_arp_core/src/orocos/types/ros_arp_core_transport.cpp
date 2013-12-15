
#include <arp_core/Velocity.h>
#include <arp_core/OmniCommand.h>
#include <arp_core/Obstacle.h>
#include <arp_core/MotionTarget.h>
#include <arp_core/OpponentsList.h>
#include <arp_core/Pose.h>
#include <arp_core/Start.h>
#include <arp_core/StartColor.h>
#include <arp_core/Odo.h>
#include <arp_core/DifferentialCommand.h>
#include <arp_core/OmniOdo.h>
#include <arp_core/LocalizationState.h>

#include "ros_msg_transporter.hpp"
#include "RosLib.hpp"
#include <rtt/types/TransportPlugin.hpp>
#include <rtt/types/TypekitPlugin.hpp>

namespace ros_integration {
  using namespace RTT;
    struct ROSarp_corePlugin
      : public types::TransportPlugin
    {
      bool registerTransport(std::string name, types::TypeInfo* ti)
      {
                   if(name == "/arp_core/Velocity")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<arp_core::Velocity>());
         if(name == "/arp_core/OmniCommand")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<arp_core::OmniCommand>());
         if(name == "/arp_core/Obstacle")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<arp_core::Obstacle>());
         if(name == "/arp_core/MotionTarget")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<arp_core::MotionTarget>());
         if(name == "/arp_core/OpponentsList")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<arp_core::OpponentsList>());
         if(name == "/arp_core/Pose")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<arp_core::Pose>());
         if(name == "/arp_core/Start")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<arp_core::Start>());
         if(name == "/arp_core/StartColor")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<arp_core::StartColor>());
         if(name == "/arp_core/Odo")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<arp_core::Odo>());
         if(name == "/arp_core/DifferentialCommand")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<arp_core::DifferentialCommand>());
         if(name == "/arp_core/OmniOdo")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<arp_core::OmniOdo>());
         if(name == "/arp_core/LocalizationState")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<arp_core::LocalizationState>());

          return false;
      }
      
      std::string getTransportName() const {
          return "ros";
      }
      
      std::string getTypekitName() const {
          return std::string("ros-")+"arp_core";
      }
      std::string getName() const {
          return std::string("rtt-ros-") + "arp_core" + "-transport";
      }

    };
}

ORO_TYPEKIT_PLUGIN( ros_integration::ROSarp_corePlugin )
