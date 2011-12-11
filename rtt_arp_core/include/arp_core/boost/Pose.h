/* Auto-generated by genmsg_cpp for file /opt/ard/arp_core/msg/Pose.msg */
#ifndef ARP_CORE_BOOST_SERIALIZATION_POSE_H
#define ARP_CORE_BOOST_SERIALIZATION_POSE_H

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/nvp.hpp>
#include <arp_core/Pose.h>

namespace boost
{
namespace serialization
{

template<class Archive, class ContainerAllocator>
void serialize(Archive& a,  ::arp_core::Pose_<ContainerAllocator>  & m, unsigned int)
{
    a & make_nvp("x",m.x);
    a & make_nvp("y",m.y);
    a & make_nvp("theta",m.theta);
    a & make_nvp("linear_velocity",m.linear_velocity);
    a & make_nvp("angular_velocity",m.angular_velocity);
    a & make_nvp("date",m.date);
}

} // namespace serialization
} // namespace boost

#endif // ARP_CORE_BOOST_SERIALIZATION_POSE_H
