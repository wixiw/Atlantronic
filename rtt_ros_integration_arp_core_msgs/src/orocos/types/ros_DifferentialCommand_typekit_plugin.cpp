#include <arp_core/boost/DifferentialCommand.h>
#include <rtt/types/TypekitPlugin.hpp>
#include <rtt/types/StructTypeInfo.hpp>
#include <rtt/types/SequenceTypeInfo.hpp>
#include <vector>

namespace ros_integration {
  using namespace RTT;
    // Factory function
            void rtt_ros_addType_DifferentialCommand() { RTT::types::Types()->addType( new types::StructTypeInfo<arp_core::DifferentialCommand>("/arp_core/DifferentialCommand") ); RTT::types::Types()->addType( new types::SequenceTypeInfo<std::vector<arp_core::DifferentialCommand> >("/arp_core/DifferentialCommand[]") ); }

    
}

