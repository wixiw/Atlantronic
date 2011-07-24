#include <arp_core/boost/Pose.h>
#include <rtt/types/TypekitPlugin.hpp>
#include <rtt/types/StructTypeInfo.hpp>
#include <rtt/types/SequenceTypeInfo.hpp>
#include <vector>

// Note: we need to put these up-front or we get gcc compiler warnings:
// <<warning: type attributes ignored after type is already defined>>        
template class RTT_EXPORT RTT::internal::DataSourceTypeInfo< arp_core::Pose >;
template class RTT_EXPORT RTT::internal::DataSource< arp_core::Pose >;
template class RTT_EXPORT RTT::internal::AssignableDataSource< arp_core::Pose >;
template class RTT_EXPORT RTT::internal::AssignCommand< arp_core::Pose >;
template class RTT_EXPORT RTT::internal::ValueDataSource< arp_core::Pose >;
template class RTT_EXPORT RTT::internal::ConstantDataSource< arp_core::Pose >;
template class RTT_EXPORT RTT::internal::ReferenceDataSource< arp_core::Pose >;
template class RTT_EXPORT RTT::OutputPort< arp_core::Pose >;
template class RTT_EXPORT RTT::InputPort< arp_core::Pose >;
template class RTT_EXPORT RTT::Property< arp_core::Pose >;
template class RTT_EXPORT RTT::Attribute< arp_core::Pose >;
template class RTT_EXPORT RTT::Constant< arp_core::Pose >;

namespace ros_integration {
  using namespace RTT;
    // Factory function
            void rtt_ros_addType_Pose() { RTT::types::Types()->addType( new types::StructTypeInfo<arp_core::Pose>("/arp_core/Pose") ); RTT::types::Types()->addType( new types::SequenceTypeInfo<std::vector<arp_core::Pose> >("/arp_core/Pose[]") ); }

    
}

