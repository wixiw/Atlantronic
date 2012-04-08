#include <arp_core/boost/StartColor.h>
#include <rtt/types/TypekitPlugin.hpp>
#include <rtt/types/StructTypeInfo.hpp>
#include <rtt/types/PrimitiveSequenceTypeInfo.hpp>
#include <rtt/types/CArrayTypeInfo.hpp>
#include <vector>

// Note: we need to put these up-front or we get gcc compiler warnings:
// <<warning: type attributes ignored after type is already defined>>        
template class RTT_EXPORT RTT::internal::DataSourceTypeInfo< arp_core::StartColor >;
template class RTT_EXPORT RTT::internal::DataSource< arp_core::StartColor >;
template class RTT_EXPORT RTT::internal::AssignableDataSource< arp_core::StartColor >;
template class RTT_EXPORT RTT::internal::AssignCommand< arp_core::StartColor >;
template class RTT_EXPORT RTT::internal::ValueDataSource< arp_core::StartColor >;
template class RTT_EXPORT RTT::internal::ConstantDataSource< arp_core::StartColor >;
template class RTT_EXPORT RTT::internal::ReferenceDataSource< arp_core::StartColor >;
template class RTT_EXPORT RTT::OutputPort< arp_core::StartColor >;
template class RTT_EXPORT RTT::InputPort< arp_core::StartColor >;
template class RTT_EXPORT RTT::Property< arp_core::StartColor >;
template class RTT_EXPORT RTT::Attribute< arp_core::StartColor >;
template class RTT_EXPORT RTT::Constant< arp_core::StartColor >;

namespace ros_integration {
  using namespace RTT;
    // Factory function
    
        void rtt_ros_addType_StartColor() {
             // Only the .msg type is sent over ports. The msg[] (variable size) and  cmsg[] (fixed size) exist only as members of larger messages
             RTT::types::Types()->addType( new types::StructTypeInfo<arp_core::StartColor>("/arp_core/StartColor") );
             RTT::types::Types()->addType( new types::PrimitiveSequenceTypeInfo<std::vector<arp_core::StartColor> >("/arp_core/StartColor[]") );
             RTT::types::Types()->addType( new types::CArrayTypeInfo<RTT::types::carray<arp_core::StartColor> >("/arp_core/cStartColor[]") );
        }

    
}

