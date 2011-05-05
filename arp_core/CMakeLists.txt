#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(arp_core)


###
# add ARD specific cmakes
###

#configuration de ROS
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
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


#orocos specific
configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/orocos-arp_core.pc.in ${CMAKE_CURRENT_BINARY_DIR}/orocos-arp_core-${OROCOS_TARGET}.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/orocos-arp_core-${OROCOS_TARGET}.pc DESTINATION  lib/pkgconfig )
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/manifest.xml DESTINATION  lib/orocos/arp_core )
#
# Provided 'uninstall' target
#
orocos_uninstall_target()
