#include <actionlib_msgs/boost/GoalID.h>
#include <rtt/types/TypekitPlugin.hpp>
#include <rtt/types/StructTypeInfo.hpp>
#include <rtt/types/SequenceTypeInfo.hpp>
#include <vector>

namespace ros_integration {
  using namespace RTT;
    // Factory function
            void rtt_ros_addType_GoalID() { RTT::types::Types()->addType( new types::StructTypeInfo<actionlib_msgs::GoalID>("/actionlib_msgs/GoalID") ); RTT::types::Types()->addType( new types::SequenceTypeInfo<std::vector<actionlib_msgs::GoalID> >("/actionlib_msgs/GoalID[]") ); }

    
}

