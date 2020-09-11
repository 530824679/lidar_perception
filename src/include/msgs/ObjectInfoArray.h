// Generated by gencpp from file lidar_perception/ObjectInfoArray.msg
// DO NOT EDIT!


#ifndef LIDAR_PERCEPTION_MESSAGE_OBJECTINFOARRAY_H
#define LIDAR_PERCEPTION_MESSAGE_OBJECTINFOARRAY_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include "msgs/DetectionObjectInfo.h"
#include "msgs/TrackingObjectInfo.h"

namespace lidar_perception
{
template <class ContainerAllocator>
struct ObjectInfoArray_
{
  typedef ObjectInfoArray_<ContainerAllocator> Type;

  ObjectInfoArray_()
    : header()
    , detection_object_info()
    , detection_object_num(0)
    , tracking_object_info()
    , tracking_object_num(0)  {
    }
  ObjectInfoArray_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , detection_object_info()
    , detection_object_num(0)
    , tracking_object_info()
    , tracking_object_num(0)  {
  (void)_alloc;
      detection_object_info.assign( ::lidar_perception::DetectionObjectInfo_<ContainerAllocator> (_alloc));

      tracking_object_info.assign( ::lidar_perception::TrackingObjectInfo_<ContainerAllocator> (_alloc));
  }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef boost::array< ::lidar_perception::DetectionObjectInfo_<ContainerAllocator> , 128>  _detection_object_info_type;
  _detection_object_info_type detection_object_info;

   typedef uint16_t _detection_object_num_type;
  _detection_object_num_type detection_object_num;

   typedef boost::array< ::lidar_perception::TrackingObjectInfo_<ContainerAllocator> , 128>  _tracking_object_info_type;
  _tracking_object_info_type tracking_object_info;

   typedef uint16_t _tracking_object_num_type;
  _tracking_object_num_type tracking_object_num;





  typedef boost::shared_ptr< ::lidar_perception::ObjectInfoArray_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::lidar_perception::ObjectInfoArray_<ContainerAllocator> const> ConstPtr;

}; // struct ObjectInfoArray_

typedef ::lidar_perception::ObjectInfoArray_<std::allocator<void> > ObjectInfoArray;

typedef boost::shared_ptr< ::lidar_perception::ObjectInfoArray > ObjectInfoArrayPtr;
typedef boost::shared_ptr< ::lidar_perception::ObjectInfoArray const> ObjectInfoArrayConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::lidar_perception::ObjectInfoArray_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::lidar_perception::ObjectInfoArray_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace lidar_perception

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'lidar_perception': ['/home/chenwei/HDD/lidar_perception/src/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::lidar_perception::ObjectInfoArray_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::lidar_perception::ObjectInfoArray_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lidar_perception::ObjectInfoArray_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lidar_perception::ObjectInfoArray_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lidar_perception::ObjectInfoArray_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lidar_perception::ObjectInfoArray_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::lidar_perception::ObjectInfoArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "435aa744c6d001e686260c5aa1fdadf5";
  }

  static const char* value(const ::lidar_perception::ObjectInfoArray_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x435aa744c6d001e6ULL;
  static const uint64_t static_value2 = 0x86260c5aa1fdadf5ULL;
};

template<class ContainerAllocator>
struct DataType< ::lidar_perception::ObjectInfoArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "lidar_perception/ObjectInfoArray";
  }

  static const char* value(const ::lidar_perception::ObjectInfoArray_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::lidar_perception::ObjectInfoArray_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header                       header                 # The message of header\n\
\n\
DetectionObjectInfo[128]        detection_object_info  # The information of detection object\n\
\n\
uint16                       detection_object_num   # The number of detection object\n\
\n\
TrackingObjectInfo[128]         tracking_object_info   # The information of tracking object\n\
\n\
uint16                       tracking_object_num    # The number of tracking object\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: lidar_perception/DetectionObjectInfo\n\
# Detection\n\
uint8                        type 		           # The category of object\n\
# unknow     0\n\
# pedestrain 1\n\
# motor      2\n\
# car        3\n\
# truck      4\n\
\n\
uint8                        dt_center_x\n\
uint8                        dt_center_y\n\
uint8                        dt_center_z\n\
uint8                        dt_yaw                # The orientation angle of object\n\
uint8                        dt_confidence         # The confidence of object\n\
uint16                       dt_height             # The height of object\n\
uint16                       dt_width              # The width of object\n\
uint16                       dt_length             # The length of object\n\
int16                        dt_distance_xv        # The longitudinal distance of object to ego vehicle coordinate\n\
int16                        dt_distance_yv        # The lateral distance of object to ego vehicle coordinate\n\
================================================================================\n\
MSG: lidar_perception/TrackingObjectInfo\n\
# Tracking\n\
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

  static const char* value(const ::lidar_perception::ObjectInfoArray_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::lidar_perception::ObjectInfoArray_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.detection_object_info);
      stream.next(m.detection_object_num);
      stream.next(m.tracking_object_info);
      stream.next(m.tracking_object_num);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ObjectInfoArray_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::lidar_perception::ObjectInfoArray_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::lidar_perception::ObjectInfoArray_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "detection_object_info[]" << std::endl;
    for (size_t i = 0; i < v.detection_object_info.size(); ++i)
    {
      s << indent << "  detection_object_info[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::lidar_perception::DetectionObjectInfo_<ContainerAllocator> >::stream(s, indent + "    ", v.detection_object_info[i]);
    }
    s << indent << "detection_object_num: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.detection_object_num);
    s << indent << "tracking_object_info[]" << std::endl;
    for (size_t i = 0; i < v.tracking_object_info.size(); ++i)
    {
      s << indent << "  tracking_object_info[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::lidar_perception::TrackingObjectInfo_<ContainerAllocator> >::stream(s, indent + "    ", v.tracking_object_info[i]);
    }
    s << indent << "tracking_object_num: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.tracking_object_num);
  }
};

} // namespace message_operations
} // namespace ros

#endif // LIDAR_PERCEPTION_MESSAGE_OBJECTINFOARRAY_H
