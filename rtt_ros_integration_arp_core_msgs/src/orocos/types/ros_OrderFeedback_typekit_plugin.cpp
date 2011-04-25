#include <arp_master/boost/OrderFeedback.h>
#include <rtt/types/TypekitPlugin.hpp>
#include <rtt/types/StructTypeInfo.hpp>
#include <rtt/types/SequenceTypeInfo.hpp>
#include <vector>

namespace ros_integration {
  using namespace RTT;
    // Factory function
            void rtt_ros_addType_OrderFeedback() { RTT::types::Types()->addType( new types::StructTypeInfo<arp_master::OrderFeedback>("/arp_master/OrderFeedback") ); RTT::types::Types()->addType( new types::SequenceTypeInfo<std::vector<arp_master::OrderFeedback> >("/arp_master/OrderFeedback[]") ); }

    
}

