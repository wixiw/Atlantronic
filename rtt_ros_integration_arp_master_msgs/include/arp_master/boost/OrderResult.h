/* Auto-generated by genmsg_cpp for file /opt/ros/ard/arp_master/msg/OrderResult.msg */
#ifndef ARP_MASTER_BOOST_SERIALIZATION_ORDERRESULT_H
#define ARP_MASTER_BOOST_SERIALIZATION_ORDERRESULT_H

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/nvp.hpp>
#include <arp_master/OrderResult.h>

namespace boost
{
namespace serialization
{

template<class Archive, class ContainerAllocator>
void serialize(Archive& a,  ::arp_master::OrderResult_<ContainerAllocator>  & m, unsigned int)
{
    a & make_nvp("x_end",m.x_end);
    a & make_nvp("y_end",m.y_end);
    a & make_nvp("theta_end",m.theta_end);
}

} // namespace serialization
} // namespace boost

#endif // ARP_MASTER_BOOST_SERIALIZATION_ORDERRESULT_H

