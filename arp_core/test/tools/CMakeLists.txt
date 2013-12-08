###################################################################################################################
#    EDIT BELOW THIS LINE AND UNTIL NEXT LINE FOR A NEW PROJECT

rosbuild_add_boost_directories()
rosbuild_add_executable(uTest_Core_Tools.ard  uTest_Tools.cpp)
target_link_libraries( uTest_Core_Tools.ard arp_core-tools )

###################################################################################################################
