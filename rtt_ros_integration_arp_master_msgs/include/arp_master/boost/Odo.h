/* Auto-generated by genmsg_cpp for file /opt/ros/ard/arp_master/msg/Odo.msg */
#ifndef ARP_MASTER_BOOST_SERIALIZATION_ODO_H
#define ARP_MASTER_BOOST_SERIALIZATION_ODO_H

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/nvp.hpp>
#include <arp_master/Odo.h>

namespace boost
{
namespace serialization
{

template<class Archive, class ContainerAllocator>
void serialize(Archive& a,  ::arp_master::Odo_<ContainerAllocator>  & m, unsigned int)
{
    a & make_nvp("odo_left",m.odo_left);
    a & make_nvp("odo_right",m.odo_right);
}

} // namespace serialization
} // namespace boost

#endif // ARP_MASTER_BOOST_SERIALIZATION_ODO_H

