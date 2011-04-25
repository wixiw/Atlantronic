#include <arp_master/boost/OrderGoal.h>
#include <rtt/types/TypekitPlugin.hpp>
#include <rtt/types/StructTypeInfo.hpp>
#include <rtt/types/SequenceTypeInfo.hpp>
#include <vector>

namespace ros_integration {
  using namespace RTT;
    // Factory function
            void rtt_ros_addType_OrderGoal() { RTT::types::Types()->addType( new types::StructTypeInfo<arp_master::OrderGoal>("/arp_master/OrderGoal") ); RTT::types::Types()->addType( new types::SequenceTypeInfo<std::vector<arp_master::OrderGoal> >("/arp_master/OrderGoal[]") ); }

    
}

