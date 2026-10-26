cmake_minimum_required(VERSION 3.5)
#cmake_minimum_required(VERSION 2.8.12)

project(keyence_experimental)

#add_compile_options(-std=c++17 -Wall -Wextra)
#set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17") 

# DO NOT set CMAKE_PREFIX_PATH here. This file used to carry
#     set(CMAKE_PREFIX_PATH "/opt/ros/humble")
# which REPLACES the prefix path colcon passes in. The CMake variable is
# searched before the CMAKE_PREFIX_PATH *environment* variable, so on any
# machine that also has Humble installed, every find_package below resolved
# against Humble while the rest of the workspace was built and run against
# Iron. The binary linked, then the loader failed at startup with
#   symbol lookup error: undefined symbol:
#     _ZN6rclcpp19QOSEventHandlerBase26get_number_of_ready_eventsEv
# (rclcpp::QOSEventHandlerBase::get_number_of_ready_events() is Humble-only)
# and exited 127, taking the Keyence out of the whole bring-up.
# On a machine with only one distro the line is harmless -- the nonexistent
# directory just falls through to the environment -- so the fault only appears
# on a dual-distro machine.
find_package(ament_cmake REQUIRED)

## Find catkin macros and libraries
# find_package(catkin REQUIRED COMPONENTS
#  message_generation
#  message_runtime
#  pcl_ros
#  roscpp
#  std_msgs
# )

find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# keyence_driver_node publishes what it is doing on `keyence_status` so that a
# silent /profiles can be told from a dead driver -- see the state words in
# keyence_driver_node.cpp.
find_package(diagnostic_msgs REQUIRED)
find_package(PCL REQUIRED)
# keyence_driver_node.cpp includes boost/bind and boost/make_shared. The
# include dirs were referenced below without ever being found.
find_package(Boost REQUIRED)

## Driver makes use of the libsocket libraries.
#
# NOT find_package(libsocket). The vendored upstream libsocket
# (ros_ws/src/libsocket, github.com/dermesser/libsocket) configures
# libsocketConfig.cmake into its BUILD tree and never installs it, and its
# export(PACKAGE libsocket) has been a no-op since CMake 3.15 unless
# CMAKE_EXPORT_PACKAGE_REGISTRY is ON (policy CMP0090). find_package(libsocket)
# therefore only ever succeeded from a stale libsocket_DIR in an
# already-populated build/keyence_experimental/CMakeCache.txt. A build after
# `rm -rf build install log` -- exactly what CELL_CHECKLIST.md tells the
# operator to do -- failed with
#   Could not find a package configuration file provided by "libsocket"
# Locate the installed files directly: colcon puts install/libsocket on
# CMAKE_PREFIX_PATH, which is where find_path/find_library look.
# Search ONLY the workspace, never the system paths. dermesser/libsocket builds
# a library called libsocket++.so -- and so does Debian's completely unrelated
# `libsocket++-dev` (the socket++ iostream library, headers under
# /usr/include/socket++, no inetclientstream.hpp). That package is often
# installed on the build machine, so a plain find_library(NAMES socket++) picks
# /usr/lib/x86_64-linux-gnu/libsocket++.so and links the wrong library.
# Pin it instead.
get_filename_component(_ws_install_root "${CMAKE_INSTALL_PREFIX}" DIRECTORY)
set(_libsocket_prefixes
  "${_ws_install_root}/libsocket"   # colcon isolated install (the default)
  "${CMAKE_INSTALL_PREFIX}")        # colcon --merge-install
find_path(libsocket_INCLUDE_DIRS
  NAMES inetclientstream.hpp
  PATHS ${_libsocket_prefixes}
  PATH_SUFFIXES include/libsocket
  NO_DEFAULT_PATH
  DOC "Directory holding libsocket's C++ headers")
find_library(libsocket_LIBRARIES
  NAMES socket++
  PATHS ${_libsocket_prefixes}
  PATH_SUFFIXES lib
  NO_DEFAULT_PATH
  DOC "libsocket's C++ shared library")
if(NOT libsocket_INCLUDE_DIRS OR NOT libsocket_LIBRARIES)
  message(FATAL_ERROR
    "libsocket not found under ${_libsocket_prefixes}\n"
    "  headers: '${libsocket_INCLUDE_DIRS}'\n"
    "  library: '${libsocket_LIBRARIES}'\n"
    "libsocket is a package in THIS colcon workspace. Build it first:\n"
    "  colcon build --packages-select libsocket\n"
    "To point at a copy elsewhere, pass -Dlibsocket_INCLUDE_DIRS=... and "
    "-Dlibsocket_LIBRARIES=... (do NOT use Debian's libsocket++-dev, which is "
    "a different library with the same file name).")
endif()
message(STATUS "libsocket headers: ${libsocket_INCLUDE_DIRS}")
message(STATUS "libsocket library: ${libsocket_LIBRARIES}")

# libsocket++.so has no version suffix, so DT_NEEDED is the bare name and the
# loader would happily bind to the system libsocket++.so of a machine where
# install/setup.bash was not sourced. Bake the real directory into the
# installed binaries' RPATH.
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


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

# Headers install to include/${PROJECT_NAME}; exporting the SOURCE-relative
# path put a build-tree path into the installed cmake config.
ament_export_include_directories(include/${PROJECT_NAME})

ament_export_libraries(keyence_impl)

ament_export_dependencies(
  pcl_ros
  pcl_conversions
  builtin_interfaces
  rosidl_default_generators
  rosidl_default_runtime
  sensor_msgs
  diagnostic_msgs
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
target_compile_features(keyence_reconstruction_node PUBLIC cxx_std_17)

target_compile_features(keyence_driver_node PUBLIC cxx_std_17)

ament_target_dependencies(keyence_driver_node
  rclcpp
  pcl_ros
  std_msgs
  sensor_msgs
  diagnostic_msgs
  pcl_conversions
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

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # The bounds checks in SingleProfile::Response::decodeFrom(). Nothing a
  # healthy controller sends exercises them -- see the file's header for the
  # SIGSEGV they are for.
  ament_add_gtest(test_profile_decode test/test_profile_decode.cpp)
  target_link_libraries(test_profile_decode keyence_impl)
  target_compile_features(test_profile_decode PUBLIC cxx_std_17)

  # Client::sendReceive()'s classification of a reply BEFORE any decoder sees
  # it: the "no profile" answer a controller with its laser off gives, which
  # otherwise tears the connection down once a second. Drives a scripted
  # Client, so it needs no controller and no network.
  ament_add_gtest(test_response_classification test/test_response_classification.cpp)
  target_link_libraries(test_response_classification keyence_impl)
  target_compile_features(test_response_classification PUBLIC cxx_std_17)

  # What a profile's stamp means: the instant it was MEASURED, not the instant
  # the driver got round to publishing it. Pure arithmetic, which is why it is
  # in a header of its own -- a stamp read off a clock cannot be tested.
  ament_add_gtest(test_profile_timing test/test_profile_timing.cpp)
  target_compile_features(test_profile_timing PUBLIC cxx_std_17)
endif()

ament_package()
