/* Auto-generated by genmsg_cpp for file /opt/ard/arp_core/msg/Velocity.msg */
#ifndef ARP_CORE_BOOST_SERIALIZATION_VELOCITY_H
#define ARP_CORE_BOOST_SERIALIZATION_VELOCITY_H

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/nvp.hpp>
#include <arp_core/Velocity.h>

namespace boost
{
namespace serialization
{

template<class Archive, class ContainerAllocator>
void serialize(Archive& a,  ::arp_core::Velocity_<ContainerAllocator>  & m, unsigned int)
{
    a & make_nvp("linear",m.linear);
    a & make_nvp("angular",m.angular);
}

} // namespace serialization
} // namespace boost

#endif // ARP_CORE_BOOST_SERIALIZATION_VELOCITY_H
