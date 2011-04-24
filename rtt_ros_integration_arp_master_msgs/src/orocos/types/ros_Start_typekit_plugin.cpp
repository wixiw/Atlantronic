#include <arp_master/boost/Start.h>
#include <rtt/types/TypekitPlugin.hpp>
#include <rtt/types/StructTypeInfo.hpp>
#include <rtt/types/SequenceTypeInfo.hpp>
#include <vector>

namespace ros_integration {
  using namespace RTT;
    // Factory function
            void rtt_ros_addType_Start() { RTT::types::Types()->addType( new types::StructTypeInfo<arp_master::Start>("/arp_master/Start") ); RTT::types::Types()->addType( new types::SequenceTypeInfo<std::vector<arp_master::Start> >("/arp_master/Start[]") ); }

    
}

