// Generated by gencpp from file lidar_perception/TrackingObjectInfo.msg
// DO NOT EDIT!


#ifndef LIDAR_PERCEPTION_MESSAGE_TRACKINGOBJECTINFO_H
#define LIDAR_PERCEPTION_MESSAGE_TRACKINGOBJECTINFO_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace lidar_perception
{
template <class ContainerAllocator>
struct TrackingObjectInfo_
{
  typedef TrackingObjectInfo_<ContainerAllocator> Type;

  TrackingObjectInfo_()
    : id(0)
    , type(0)
    , motion_state(0)
    , tk_center_x(0)
    , tk_center_y(0)
    , tk_center_z(0)
    , tk_height(0)
    , tk_width(0)
    , tk_length(0)
    , tk_yaw(0)
    , tk_distance_xv(0)
    , tk_distance_yv(0)
    , tk_velocity_xv(0)
    , tk_velocity_yv(0)
    , tk_accelerate_xv(0)
    , tk_accelerate_yv(0)  {
    }
  TrackingObjectInfo_(const ContainerAllocator& _alloc)
    : id(0)
    , type(0)
    , motion_state(0)
    , tk_center_x(0)
    , tk_center_y(0)
    , tk_center_z(0)
    , tk_height(0)
    , tk_width(0)
    , tk_length(0)
    , tk_yaw(0)
    , tk_distance_xv(0)
    , tk_distance_yv(0)
    , tk_velocity_xv(0)
    , tk_velocity_yv(0)
    , tk_accelerate_xv(0)
    , tk_accelerate_yv(0)  {
  (void)_alloc;
    }



   typedef uint16_t _id_type;
  _id_type id;

   typedef uint8_t _type_type;
  _type_type type;

   typedef uint8_t _motion_state_type;
  _motion_state_type motion_state;

   typedef uint8_t _tk_center_x_type;
  _tk_center_x_type tk_center_x;

   typedef uint8_t _tk_center_y_type;
  _tk_center_y_type tk_center_y;

   typedef uint8_t _tk_center_z_type;
  _tk_center_z_type tk_center_z;

   typedef uint16_t _tk_height_type;
  _tk_height_type tk_height;

   typedef uint16_t _tk_width_type;
  _tk_width_type tk_width;

   typedef uint16_t _tk_length_type;
  _tk_length_type tk_length;

   typedef uint8_t _tk_yaw_type;
  _tk_yaw_type tk_yaw;

   typedef int16_t _tk_distance_xv_type;
  _tk_distance_xv_type tk_distance_xv;

   typedef int16_t _tk_distance_yv_type;
  _tk_distance_yv_type tk_distance_yv;

   typedef int16_t _tk_velocity_xv_type;
  _tk_velocity_xv_type tk_velocity_xv;

   typedef int16_t _tk_velocity_yv_type;
  _tk_velocity_yv_type tk_velocity_yv;

   typedef int16_t _tk_accelerate_xv_type;
  _tk_accelerate_xv_type tk_accelerate_xv;

   typedef int16_t _tk_accelerate_yv_type;
  _tk_accelerate_yv_type tk_accelerate_yv;





  typedef boost::shared_ptr< ::lidar_perception::TrackingObjectInfo_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::lidar_perception::TrackingObjectInfo_<ContainerAllocator> const> ConstPtr;

}; // struct TrackingObjectInfo_

typedef ::lidar_perception::TrackingObjectInfo_<std::allocator<void> > TrackingObjectInfo;

typedef boost::shared_ptr< ::lidar_perception::TrackingObjectInfo > TrackingObjectInfoPtr;
typedef boost::shared_ptr< ::lidar_perception::TrackingObjectInfo const> TrackingObjectInfoConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::lidar_perception::TrackingObjectInfo_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::lidar_perception::TrackingObjectInfo_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace lidar_perception

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'lidar_perception': ['/home/chenwei/HDD/lidar_perception/src/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::lidar_perception::TrackingObjectInfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::lidar_perception::TrackingObjectInfo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lidar_perception::TrackingObjectInfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lidar_perception::TrackingObjectInfo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lidar_perception::TrackingObjectInfo_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lidar_perception::TrackingObjectInfo_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::lidar_perception::TrackingObjectInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8e916f6a9fa8e966b7a404cce28ad4c6";
  }

  static const char* value(const ::lidar_perception::TrackingObjectInfo_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8e916f6a9fa8e966ULL;
  static const uint64_t static_value2 = 0xb7a404cce28ad4c6ULL;
};

template<class ContainerAllocator>
struct DataType< ::lidar_perception::TrackingObjectInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "lidar_perception/TrackingObjectInfo";
  }

  static const char* value(const ::lidar_perception::TrackingObjectInfo_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::lidar_perception::TrackingObjectInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Tracking\n\
uint16                       id                    # The No. of object\n\
uint8                        type 		           # The category of object\n\
# unknow     0\n\
# pedestrain 1\n\
# motor      2\n\
# car        3\n\
# truck      4\n\
\n\
uint8                        motion_state          # The motion status of object\n\
# unknow     0\n\
# moving     1\n\
# stationary 2\n\
\n\
uint8                        tk_center_x\n\
uint8                        tk_center_y\n\
uint8                        tk_center_z\n\
uint16                       tk_height\n\
uint16                       tk_width\n\
uint16                       tk_length\n\
uint8                        tk_yaw\n\
int16                        tk_distance_xv\n\
int16                        tk_distance_yv\n\
int16                        tk_velocity_xv        # The longitudinal velocity of object to ego vehicle coordinate\n\
int16                        tk_velocity_yv        # The lateral velocity of object to ego vehicle coordinate\n\
int16                        tk_accelerate_xv      # The longitudinal accelerated velocity of object to ego vehicle coordinate\n\
int16                        tk_accelerate_yv      # The lateral accelerated velocity of object to ego vehicle coordinate\n\
\n\
";
  }

  static const char* value(const ::lidar_perception::TrackingObjectInfo_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::lidar_perception::TrackingObjectInfo_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
      stream.next(m.type);
      stream.next(m.motion_state);
      stream.next(m.tk_center_x);
      stream.next(m.tk_center_y);
      stream.next(m.tk_center_z);
      stream.next(m.tk_height);
      stream.next(m.tk_width);
      stream.next(m.tk_length);
      stream.next(m.tk_yaw);
      stream.next(m.tk_distance_xv);
      stream.next(m.tk_distance_yv);
      stream.next(m.tk_velocity_xv);
      stream.next(m.tk_velocity_yv);
      stream.next(m.tk_accelerate_xv);
      stream.next(m.tk_accelerate_yv);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TrackingObjectInfo_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::lidar_perception::TrackingObjectInfo_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::lidar_perception::TrackingObjectInfo_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.id);
    s << indent << "type: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.type);
    s << indent << "motion_state: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.motion_state);
    s << indent << "tk_center_x: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.tk_center_x);
    s << indent << "tk_center_y: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.tk_center_y);
    s << indent << "tk_center_z: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.tk_center_z);
    s << indent << "tk_height: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.tk_height);
    s << indent << "tk_width: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.tk_width);
    s << indent << "tk_length: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.tk_length);
    s << indent << "tk_yaw: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.tk_yaw);
    s << indent << "tk_distance_xv: ";
    Printer<int16_t>::stream(s, indent + "  ", v.tk_distance_xv);
    s << indent << "tk_distance_yv: ";
    Printer<int16_t>::stream(s, indent + "  ", v.tk_distance_yv);
    s << indent << "tk_velocity_xv: ";
    Printer<int16_t>::stream(s, indent + "  ", v.tk_velocity_xv);
    s << indent << "tk_velocity_yv: ";
    Printer<int16_t>::stream(s, indent + "  ", v.tk_velocity_yv);
    s << indent << "tk_accelerate_xv: ";
    Printer<int16_t>::stream(s, indent + "  ", v.tk_accelerate_xv);
    s << indent << "tk_accelerate_yv: ";
    Printer<int16_t>::stream(s, indent + "  ", v.tk_accelerate_yv);
  }
};

} // namespace message_operations
} // namespace ros

#endif // LIDAR_PERCEPTION_MESSAGE_TRACKINGOBJECTINFO_H
