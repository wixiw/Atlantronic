#include <actionlib_msgs/boost/GoalStatus.h>
#include <rtt/types/TypekitPlugin.hpp>
#include <rtt/types/StructTypeInfo.hpp>
#include <rtt/types/SequenceTypeInfo.hpp>
#include <vector>

namespace ros_integration {
  using namespace RTT;
    // Factory function
            void rtt_ros_addType_GoalStatus() { RTT::types::Types()->addType( new types::StructTypeInfo<actionlib_msgs::GoalStatus>("/actionlib_msgs/GoalStatus") ); RTT::types::Types()->addType( new types::SequenceTypeInfo<std::vector<actionlib_msgs::GoalStatus> >("/actionlib_msgs/GoalStatus[]") ); }

    
}

