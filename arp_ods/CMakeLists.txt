#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(arp_ods)


###
# add ARD specific cmakes
###

#configuration de ROS
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

#generation des messages d'action pour actionlib
rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

rosbuild_init()
     
#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#ARD generic configuration
#set( ROS_BUILD_TYPE Debug )
rosbuild_include(arp_core ARP_common_functions)
configure_ard_project()

orocos_typegen_headers(src/orders/orders.h)

###
# end of ARD specific
###

#
# generate orocos package
#
orocos_generate_package()
