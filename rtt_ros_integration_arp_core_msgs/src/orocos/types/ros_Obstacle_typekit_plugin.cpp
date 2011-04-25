#include <arp_core/boost/Obstacle.h>
#include <rtt/types/TypekitPlugin.hpp>
#include <rtt/types/StructTypeInfo.hpp>
#include <rtt/types/SequenceTypeInfo.hpp>
#include <vector>

namespace ros_integration {
  using namespace RTT;
    // Factory function
            void rtt_ros_addType_Obstacle() { RTT::types::Types()->addType( new types::StructTypeInfo<arp_core::Obstacle>("/arp_core/Obstacle") ); RTT::types::Types()->addType( new types::SequenceTypeInfo<std::vector<arp_core::Obstacle> >("/arp_core/Obstacle[]") ); }

    
}

