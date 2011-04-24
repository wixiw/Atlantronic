#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(arp_master)


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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()
#ajout du dossier include
include_directories( include )

#TODO a remettre quand Boris compilera arp_core
#ARD generic configuration
#rosbuild_include(arp_core ARP_common_functions)
#configure_ard_project()

#TODO a deplacer dans ARP_project_dependencies quand Boris compilera arp_core
find_package(wxWidgets REQUIRED)
include(${wxWidgets_USE_FILE})
include_directories( ${wxWidgets_INCLUDE_DIRS} )
rosbuild_add_boost_directories()
add_subdirectory(src)
     



###
# end of ARD specific
###


### HowTo Launch
# [1] new terminal : roscore
# [2] new terminal : rosrun arp_master ARDSimu_node
# [3] new terminal : rosrun arp_master ARDCommand_node
# [4] new terminal : rosrun arp_master ARDLocalizator_node
# [5] new terminal : rosrun arp_master ARDTeleop_node
