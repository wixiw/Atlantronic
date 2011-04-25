#include <arp_master/boost/OrderActionResult.h>
#include <rtt/types/TypekitPlugin.hpp>
#include <rtt/types/StructTypeInfo.hpp>
#include <rtt/types/SequenceTypeInfo.hpp>
#include <vector>

namespace ros_integration {
  using namespace RTT;
    // Factory function
            void rtt_ros_addType_OrderActionResult() { RTT::types::Types()->addType( new types::StructTypeInfo<arp_master::OrderActionResult>("/arp_master/OrderActionResult") ); RTT::types::Types()->addType( new types::SequenceTypeInfo<std::vector<arp_master::OrderActionResult> >("/arp_master/OrderActionResult[]") ); }

    
}

