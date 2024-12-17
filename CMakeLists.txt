cmake_minimum_required(VERSION 3.5)
#cmake_minimum_required(VERSION 2.8.12)

project(keyence_experimental)

#add_compile_options(-std=c++17 -Wall -Wextra)
#set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17") 

set(CMAKE_PREFIX_PATH "/opt/ros/humble")
find_package(ament_cmake REQUIRED)

## Find catkin macros and libraries
#find_package(catkin REQUIRED COMPONENTS
#  message_generation
#  message_runtime
#  pcl_ros
#  roscpp
#  std_msgs
#)

find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(keyence_interfaces REQUIRED)
find_package(PCL REQUIRED)

## Driver makes use of the libsocket libraries
find_package(libsocket REQUIRED)


################################################
## Declare ROS messages, services and actions ##
################################################

#add_service_files(
#  FILES
#  ChangeProgram.srv
#)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

#catkin_package(
#  INCLUDE_DIRS
#  keyence_library/include
#  LIBRARIES
#  keyence_impl
#  CATKIN_DEPENDS
#  message_generation
#  message_runtime
#  pcl_ros
#  std_msgs
#  DEPENDS
#  libsocket
#)

ament_export_include_directories(keyence_library/include)

ament_export_libraries(keyence_impl)

ament_export_dependencies(
  pcl_ros
  pcl_conversions
  builtin_interfaces
  rosidl_default_generators
  rosidl_default_runtime
  sensor_msgs
)

###########
## Build ##
###########

include_directories(
  keyence_library/include
  keyence_library/include/messages
  ${libsocket_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(
  keyence_driver_node
  src/keyence_driver_node.cpp
)

add_executable(
  keyence_reconstruction_node
  src/point_cloud_reconstruction.cpp
)

ament_target_dependencies(keyence_reconstruction_node
sensor_msgs
visualization_msgs
rclcpp
PCL
pcl_conversions
)

target_link_libraries(keyence_reconstruction_node
  ${PCL_LIBRARIES}
)
target_compile_features(keyence_driver_node PUBLIC cxx_std_17)

target_compile_options(keyence_driver_node PUBLIC -std=c++17)

ament_target_dependencies(keyence_driver_node
  rclcpp
  pcl_ros
  std_msgs
  sensor_msgs
  keyence_interfaces
  pcl_conversions
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

target_link_libraries(
  keyence_driver_node
  keyence_impl
)


add_subdirectory(keyence_library)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
# install(
#   TARGETS
#   keyence_driver_node
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

install(TARGETS keyence_driver_node keyence_reconstruction_node
  DESTINATION lib/${PROJECT_NAME})

## Mark cpp header files for installation
install(
  DIRECTORY
  keyence_library/include/
  DESTINATION
  include/${PROJECT_NAME}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
  DIRECTORY
  config
  launch
  meshes
  urdf
  DESTINATION
  share/${PROJECT_NAME}
)

ament_package()

#############
## Testing ##
#############
