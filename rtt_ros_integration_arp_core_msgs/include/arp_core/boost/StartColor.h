/* Auto-generated by genmsg_cpp for file /opt/ros/ard/arp_core/msg/StartColor.msg */
#ifndef ARP_CORE_BOOST_SERIALIZATION_STARTCOLOR_H
#define ARP_CORE_BOOST_SERIALIZATION_STARTCOLOR_H

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/nvp.hpp>
#include <arp_core/StartColor.h>

namespace boost
{
namespace serialization
{

template<class Archive, class ContainerAllocator>
void serialize(Archive& a,  ::arp_core::StartColor_<ContainerAllocator>  & m, unsigned int)
{
    a & make_nvp("color",m.color);
}

} // namespace serialization
} // namespace boost

#endif // ARP_CORE_BOOST_SERIALIZATION_STARTCOLOR_H

