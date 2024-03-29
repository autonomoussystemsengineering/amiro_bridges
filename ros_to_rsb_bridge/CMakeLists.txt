cmake_minimum_required(VERSION 2.8.3)
project(ros_to_rsb_bridge)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  amiro_msgs
  geometry_msgs
)

find_package(RSB REQUIRED)
find_package(RSB 0.15 REQUIRED)
find_package(RSC 0.15 REQUIRED)
find_package(RST 0.15 REQUIRED COMPONENTS sandbox)
find_package(Protobuf REQUIRED)
find_package(OpenCV 3.0 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_to_rsb_bridge
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS amiro_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${RSB_INCLUDE_DIRS}
  ${RST_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIRS}
)

LINK_DIRECTORIES (
  ${RSB_LIBRARY_DIRS}
  ${RSC_LIBRARY_DIRS}
)

add_executable(ros_int_multiarray_rst_value_array
  src/ros_int_multiarray_rst_value_array.cpp
)

target_link_libraries(ros_int_multiarray_rst_value_array
  ${catkin_LIBRARIES}
  ${RSB_LIBRARIES}
  ${RST_LIBRARIES}
  ${RSTSANDBOX_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
)

add_executable(ros_geometry_msgs_posestamped_to_rst_geometry_pose
  src/ros_geometry_msgs_posestamped_to_rst_geometry_pose.cpp
)

target_link_libraries(ros_geometry_msgs_posestamped_to_rst_geometry_pose
  ${catkin_LIBRARIES}
  ${RSB_LIBRARIES}
  ${RST_LIBRARIES}
  ${RSTSANDBOX_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
)

add_executable(ros_geometry_msgs_twist_to_rst_value_array
  src/ros_geometry_msgs_twist_to_rst_value_array.cpp
)

target_link_libraries(ros_geometry_msgs_twist_to_rst_value_array
  ${catkin_LIBRARIES}
  ${RSB_LIBRARIES}
  ${RST_LIBRARIES}
  ${RSTSANDBOX_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
)

ADD_DEFINITIONS(${RST_CFLAGS} ${RSTSANDBOX_CFLAGS})
add_dependencies(ros_int_multiarray_rst_value_array amiro_msgs_gencpp)
