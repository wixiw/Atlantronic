cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

#forced to min size, else it is deadly big
set(ROS_BUILD_TYPE MinSizeRel)

rosbuild_init()
rosbuild_include(rtt_ros_integration GenerateRTTtypekit)
ros_generate_rtt_typekit(arp_core)

