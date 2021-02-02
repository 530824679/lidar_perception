cmake_minimum_required(VERSION 2.8.3)
project(lidar_perception)

SET(CMAKE_BUILD_TYPE "Release")

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_compile_options(-g)

find_package(Boost REQUIRED)
if(NOT Boost_FOUND)
    message("Not found Boost")
endif()
include_directories(${Boost_INCLUDE_DIRS} )
link_directories(${Boost_LIBRARY_DIRS})
add_definitions(${Boost_DEFINITIONS})

find_package(PCL REQUIRED)
if(NOT PCL_FOUND)
    message("Not found PCL")
endif()
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)
if(NOT OPENCV_FOUND)
    message("Not found OpenCV")
endif()
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_filters
    rosbag
    message_generation
    geometry_msgs
    pcl_ros
)

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    DetectionObjectInfo.msg
    TrackingObjectInfo.msg
    ObjectInfoArray.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES lidar_perception
    CATKIN_DEPENDS roscpp rospy pcl_ros std_msgs message_filters rosbag geometry_msgs sensor_msgs
)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/include/common
    ${PROJECT_SOURCE_DIR}/include/common/Eigen
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_library(json_lib SHARED
        src/common/json/json_reader.cpp
        src/common/json/json_valueiterator.inl
        src/common/json/json_value.cpp
        src/common/json/json_writer.cpp
        src/common/json/json_batchallocator.h
        src/common/json/json_internalarray.inl
        src/common/json/json_internalmap.inl)

add_library(utils_lib SHARED
        src/common/utils/matrix.cpp
        )

add_library(config_lib SHARED
        src/manager/config_manager.cpp)
target_link_libraries(config_lib
        json_lib)

add_library(filter_lib SHARED
        src/filter/roi_filter.cpp
        src/filter/outlier_filter.cpp
        src/filter/voxel_filter.cpp)
target_link_libraries(filter_lib
        ${PCL_LIBRARIES})

add_library(detect_lib SHARED
        src/detect/grid.cpp
        src/detect/curb_detect.cpp
        src/detect/kalman_filter.cpp
        src/detect/tracker.cpp)

add_library(segment_lib SHARED
        src/segment/object_cluster.cpp
        src/segment/object_segment.cpp)
target_link_libraries(segment_lib
        ${PCL_LIBRARIES})

add_library(builder_lib SHARED
        src/builder/bbox_fitting.cpp
        src/builder/bbox_filter.cpp
        src/builder/convex_hull.cpp)

add_library(tracker_lib SHARED
        src/tracker/kalman_filter.cpp
        src/tracker/tracker.cpp
        src/tracker/tracking.cpp
        src/tracker/velocity.cpp)
target_link_libraries(tracker_lib
        utils_lib
        ${OpenCV_LIBRARIES})

add_library(obstacle_detection STATIC
        src/process/lidar_process.cpp)
target_link_libraries(obstacle_detection
    json_lib
    utils_lib
    config_lib
    filter_lib
    detect_lib
    builder_lib
    tracker_lib
    segment_lib
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node main_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        obstacle_detection)