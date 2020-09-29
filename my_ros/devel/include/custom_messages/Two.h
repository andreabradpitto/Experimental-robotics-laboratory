// Generated by gencpp from file custom_messages/Two.msg
// DO NOT EDIT!


#ifndef CUSTOM_MESSAGES_MESSAGE_TWO_H
#define CUSTOM_MESSAGES_MESSAGE_TWO_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace custom_messages
{
template <class ContainerAllocator>
struct Two_
{
  typedef Two_<ContainerAllocator> Type;

  Two_()
    : a(0.0)
    , b(0.0)  {
    }
  Two_(const ContainerAllocator& _alloc)
    : a(0.0)
    , b(0.0)  {
  (void)_alloc;
    }



   typedef float _a_type;
  _a_type a;

   typedef float _b_type;
  _b_type b;





  typedef boost::shared_ptr< ::custom_messages::Two_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::custom_messages::Two_<ContainerAllocator> const> ConstPtr;

}; // struct Two_

typedef ::custom_messages::Two_<std::allocator<void> > Two;

typedef boost::shared_ptr< ::custom_messages::Two > TwoPtr;
typedef boost::shared_ptr< ::custom_messages::Two const> TwoConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::custom_messages::Two_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::custom_messages::Two_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace custom_messages

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'custom_messages': ['/home/my_ros/src/custom_messages/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::custom_messages::Two_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::custom_messages::Two_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::custom_messages::Two_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::custom_messages::Two_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::custom_messages::Two_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::custom_messages::Two_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::custom_messages::Two_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3f6326d24b4937f854196fd3a843f42e";
  }

  static const char* value(const ::custom_messages::Two_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3f6326d24b4937f8ULL;
  static const uint64_t static_value2 = 0x54196fd3a843f42eULL;
};

template<class ContainerAllocator>
struct DataType< ::custom_messages::Two_<ContainerAllocator> >
{
  static const char* value()
  {
    return "custom_messages/Two";
  }

  static const char* value(const ::custom_messages::Two_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::custom_messages::Two_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 a\n\
float32 b\n\
";
  }

  static const char* value(const ::custom_messages::Two_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::custom_messages::Two_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.a);
      stream.next(m.b);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Two_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::custom_messages::Two_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::custom_messages::Two_<ContainerAllocator>& v)
  {
    s << indent << "a: ";
    Printer<float>::stream(s, indent + "  ", v.a);
    s << indent << "b: ";
    Printer<float>::stream(s, indent + "  ", v.b);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CUSTOM_MESSAGES_MESSAGE_TWO_H
