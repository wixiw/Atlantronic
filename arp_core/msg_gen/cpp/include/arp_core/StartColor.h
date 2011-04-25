/* Auto-generated by genmsg_cpp for file /opt/ros/ard/arp_core/msg/StartColor.msg */
#ifndef ARP_CORE_MESSAGE_STARTCOLOR_H
#define ARP_CORE_MESSAGE_STARTCOLOR_H
#include <string>
#include <vector>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/message.h"
#include "ros/time.h"


namespace arp_core
{
template <class ContainerAllocator>
struct StartColor_ : public ros::Message
{
  typedef StartColor_<ContainerAllocator> Type;

  StartColor_()
  : color()
  {
  }

  StartColor_(const ContainerAllocator& _alloc)
  : color(_alloc)
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _color_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  color;


private:
  static const char* __s_getDataType_() { return "arp_core/StartColor"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "89e44dcab627a2c43a70ae1100695caa"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "string color\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, color);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, color);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(color);
    return size;
  }

  typedef boost::shared_ptr< ::arp_core::StartColor_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::arp_core::StartColor_<ContainerAllocator>  const> ConstPtr;
}; // struct StartColor
typedef  ::arp_core::StartColor_<std::allocator<void> > StartColor;

typedef boost::shared_ptr< ::arp_core::StartColor> StartColorPtr;
typedef boost::shared_ptr< ::arp_core::StartColor const> StartColorConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::arp_core::StartColor_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::arp_core::StartColor_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace arp_core

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::arp_core::StartColor_<ContainerAllocator> > {
  static const char* value() 
  {
    return "89e44dcab627a2c43a70ae1100695caa";
  }

  static const char* value(const  ::arp_core::StartColor_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x89e44dcab627a2c4ULL;
  static const uint64_t static_value2 = 0x3a70ae1100695caaULL;
};

template<class ContainerAllocator>
struct DataType< ::arp_core::StartColor_<ContainerAllocator> > {
  static const char* value() 
  {
    return "arp_core/StartColor";
  }

  static const char* value(const  ::arp_core::StartColor_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::arp_core::StartColor_<ContainerAllocator> > {
  static const char* value() 
  {
    return "string color\n\
\n\
";
  }

  static const char* value(const  ::arp_core::StartColor_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::arp_core::StartColor_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.color);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct StartColor_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::arp_core::StartColor_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::arp_core::StartColor_<ContainerAllocator> & v) 
  {
    s << indent << "color: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.color);
  }
};


} // namespace message_operations
} // namespace ros

#endif // ARP_CORE_MESSAGE_STARTCOLOR_H

