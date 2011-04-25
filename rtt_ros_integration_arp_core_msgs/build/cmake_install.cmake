# Install script for directory: /opt/ros/ard/rtt_ros_integration_arp_core_msgs

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "MinSizeRel")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/rtt_ros_integration_arp_core_msgs/types/librtt-ros-arp_core-typekit-gnulinux.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/rtt_ros_integration_arp_core_msgs/types/librtt-ros-arp_core-typekit-gnulinux.so")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/rtt_ros_integration_arp_core_msgs/types/librtt-ros-arp_core-typekit-gnulinux.so"
         RPATH "/usr/local/lib:/usr/local/lib/orocos/gnulinux/types:/usr/local/lib/orocos/gnulinux/rtt_ros_integration_arp_core_msgs/types:/opt/ros/ard/arp_core/lib:/opt/ros/ros_comm/clients/cpp/roscpp/lib:/opt/ros/ros_comm/clients/cpp/roscpp_serialization/lib:/opt/ros/ros_comm/utilities/xmlrpcpp/lib:/opt/ros/ros_comm/tools/rosconsole/lib:/opt/ros/ros_comm/utilities/rostime/lib:/opt/ros/ros_comm/utilities/cpp_common/lib:/opt/ros/ros/core/roslib/lib:/opt/ros/ros/tools/rospack/lib:/opt/ros/orocos_toolchain_ros/ocl/lib:/opt/ros/orocos_toolchain_ros/log4cpp/install/lib:/opt/ros/orocos_toolchain_ros/rtt_ros_integration/lib/orocos/gnulinux/plugins:/opt/ros/orocos_toolchain_ros/rtt_ros_integration/lib/orocos/gnulinux/types:/opt/ros/orocos_toolchain_ros/rtt/install/lib")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/rtt_ros_integration_arp_core_msgs/types" TYPE SHARED_LIBRARY FILES "/opt/ros/ard/rtt_ros_integration_arp_core_msgs/lib/orocos/gnulinux/types/librtt-ros-arp_core-typekit-gnulinux.so")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/rtt_ros_integration_arp_core_msgs/types/librtt-ros-arp_core-typekit-gnulinux.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/rtt_ros_integration_arp_core_msgs/types/librtt-ros-arp_core-typekit-gnulinux.so")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/rtt_ros_integration_arp_core_msgs/types/librtt-ros-arp_core-typekit-gnulinux.so"
         OLD_RPATH "/opt/ros/ard/arp_core/lib:/opt/ros/ros_comm/clients/cpp/roscpp/lib:/opt/ros/ros_comm/clients/cpp/roscpp_serialization/lib:/opt/ros/ros_comm/utilities/xmlrpcpp/lib:/opt/ros/ros_comm/tools/rosconsole/lib:/opt/ros/ros_comm/utilities/rostime/lib:/opt/ros/ros_comm/utilities/cpp_common/lib:/opt/ros/ros/core/roslib/lib:/opt/ros/ros/tools/rospack/lib:/opt/ros/orocos_toolchain_ros/ocl/lib:/opt/ros/orocos_toolchain_ros/log4cpp/install/lib:/opt/ros/orocos_toolchain_ros/rtt_ros_integration/lib/orocos/gnulinux/plugins:/opt/ros/orocos_toolchain_ros/rtt_ros_integration/lib/orocos/gnulinux/types:/opt/ros/orocos_toolchain_ros/rtt/install/lib:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::"
         NEW_RPATH "/usr/local/lib:/usr/local/lib/orocos/gnulinux/types:/usr/local/lib/orocos/gnulinux/rtt_ros_integration_arp_core_msgs/types:/opt/ros/ard/arp_core/lib:/opt/ros/ros_comm/clients/cpp/roscpp/lib:/opt/ros/ros_comm/clients/cpp/roscpp_serialization/lib:/opt/ros/ros_comm/utilities/xmlrpcpp/lib:/opt/ros/ros_comm/tools/rosconsole/lib:/opt/ros/ros_comm/utilities/rostime/lib:/opt/ros/ros_comm/utilities/cpp_common/lib:/opt/ros/ros/core/roslib/lib:/opt/ros/ros/tools/rospack/lib:/opt/ros/orocos_toolchain_ros/ocl/lib:/opt/ros/orocos_toolchain_ros/log4cpp/install/lib:/opt/ros/orocos_toolchain_ros/rtt_ros_integration/lib/orocos/gnulinux/plugins:/opt/ros/orocos_toolchain_ros/rtt_ros_integration/lib/orocos/gnulinux/types:/opt/ros/orocos_toolchain_ros/rtt/install/lib")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/rtt_ros_integration_arp_core_msgs/types/librtt-ros-arp_core-typekit-gnulinux.so")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/rtt_ros_integration_arp_core_msgs/types/librtt-ros-arp_core-transport-gnulinux.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/rtt_ros_integration_arp_core_msgs/types/librtt-ros-arp_core-transport-gnulinux.so")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/rtt_ros_integration_arp_core_msgs/types/librtt-ros-arp_core-transport-gnulinux.so"
         RPATH "/usr/local/lib:/usr/local/lib/orocos/gnulinux/types:/usr/local/lib/orocos/gnulinux/rtt_ros_integration_arp_core_msgs/types:/opt/ros/ard/arp_core/lib:/opt/ros/ros_comm/clients/cpp/roscpp/lib:/opt/ros/ros_comm/clients/cpp/roscpp_serialization/lib:/opt/ros/ros_comm/utilities/xmlrpcpp/lib:/opt/ros/ros_comm/tools/rosconsole/lib:/opt/ros/ros_comm/utilities/rostime/lib:/opt/ros/ros_comm/utilities/cpp_common/lib:/opt/ros/ros/core/roslib/lib:/opt/ros/ros/tools/rospack/lib:/opt/ros/orocos_toolchain_ros/ocl/lib:/opt/ros/orocos_toolchain_ros/log4cpp/install/lib:/opt/ros/orocos_toolchain_ros/rtt_ros_integration/lib/orocos/gnulinux/plugins:/opt/ros/orocos_toolchain_ros/rtt_ros_integration/lib/orocos/gnulinux/types:/opt/ros/orocos_toolchain_ros/rtt/install/lib")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/rtt_ros_integration_arp_core_msgs/types" TYPE SHARED_LIBRARY FILES "/opt/ros/ard/rtt_ros_integration_arp_core_msgs/lib/orocos/gnulinux/types/librtt-ros-arp_core-transport-gnulinux.so")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/rtt_ros_integration_arp_core_msgs/types/librtt-ros-arp_core-transport-gnulinux.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/rtt_ros_integration_arp_core_msgs/types/librtt-ros-arp_core-transport-gnulinux.so")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/rtt_ros_integration_arp_core_msgs/types/librtt-ros-arp_core-transport-gnulinux.so"
         OLD_RPATH "/opt/ros/ard/arp_core/lib:/opt/ros/ros_comm/clients/cpp/roscpp/lib:/opt/ros/ros_comm/clients/cpp/roscpp_serialization/lib:/opt/ros/ros_comm/utilities/xmlrpcpp/lib:/opt/ros/ros_comm/tools/rosconsole/lib:/opt/ros/ros_comm/utilities/rostime/lib:/opt/ros/ros_comm/utilities/cpp_common/lib:/opt/ros/ros/core/roslib/lib:/opt/ros/ros/tools/rospack/lib:/opt/ros/orocos_toolchain_ros/ocl/lib:/opt/ros/orocos_toolchain_ros/log4cpp/install/lib:/opt/ros/ard/rtt_ros_integration_arp_core_msgs/build:/opt/ros/orocos_toolchain_ros/rtt_ros_integration/lib/orocos/gnulinux/plugins:/opt/ros/orocos_toolchain_ros/rtt_ros_integration/lib/orocos/gnulinux/types:/opt/ros/orocos_toolchain_ros/rtt/install/lib::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::"
         NEW_RPATH "/usr/local/lib:/usr/local/lib/orocos/gnulinux/types:/usr/local/lib/orocos/gnulinux/rtt_ros_integration_arp_core_msgs/types:/opt/ros/ard/arp_core/lib:/opt/ros/ros_comm/clients/cpp/roscpp/lib:/opt/ros/ros_comm/clients/cpp/roscpp_serialization/lib:/opt/ros/ros_comm/utilities/xmlrpcpp/lib:/opt/ros/ros_comm/tools/rosconsole/lib:/opt/ros/ros_comm/utilities/rostime/lib:/opt/ros/ros_comm/utilities/cpp_common/lib:/opt/ros/ros/core/roslib/lib:/opt/ros/ros/tools/rospack/lib:/opt/ros/orocos_toolchain_ros/ocl/lib:/opt/ros/orocos_toolchain_ros/log4cpp/install/lib:/opt/ros/orocos_toolchain_ros/rtt_ros_integration/lib/orocos/gnulinux/plugins:/opt/ros/orocos_toolchain_ros/rtt_ros_integration/lib/orocos/gnulinux/types:/opt/ros/orocos_toolchain_ros/rtt/install/lib")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/rtt_ros_integration_arp_core_msgs/types/librtt-ros-arp_core-transport-gnulinux.so")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
ELSE(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
ENDIF(CMAKE_INSTALL_COMPONENT)

FILE(WRITE "/opt/ros/ard/rtt_ros_integration_arp_core_msgs/build/${CMAKE_INSTALL_MANIFEST}" "")
FOREACH(file ${CMAKE_INSTALL_MANIFEST_FILES})
  FILE(APPEND "/opt/ros/ard/rtt_ros_integration_arp_core_msgs/build/${CMAKE_INSTALL_MANIFEST}" "${file}\n")
ENDFOREACH(file)
