#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(@PROJECT_NAME@)


###
# add ARD specific cmakes
###

#configuration de ROS
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#build config
#set( ROS_BUILD_TYPE Debug )
set (ROS_BUILD_TYPE Release)
rosbuild_init()
     
#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#ARD generic configuration
rosbuild_include(arp_core ARP_common_functions)
configure_ard_project()

###
# end of ARD specific
###

#
# generate orocos package
#
orocos_generate_package()

