#include <actionlib_msgs/boost/GoalStatusArray.h>
#include <rtt/types/TypekitPlugin.hpp>
#include <rtt/types/StructTypeInfo.hpp>
#include <rtt/types/SequenceTypeInfo.hpp>
#include <vector>

namespace ros_integration {
  using namespace RTT;
    // Factory function
            void rtt_ros_addType_GoalStatusArray() { RTT::types::Types()->addType( new types::StructTypeInfo<actionlib_msgs::GoalStatusArray>("/actionlib_msgs/GoalStatusArray") ); RTT::types::Types()->addType( new types::SequenceTypeInfo<std::vector<actionlib_msgs::GoalStatusArray> >("/actionlib_msgs/GoalStatusArray[]") ); }

    
}

