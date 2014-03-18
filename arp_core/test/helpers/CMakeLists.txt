###################################################################################################################
#    EDIT BELOW THIS LINE AND UNTIL NEXT LINE FOR A NEW PROJECT

rosbuild_add_boost_directories()
rosbuild_add_executable(uTest_Core_Helpers.ard  uTest_Helpers.cpp)
target_link_libraries( uTest_Core_Helpers.ard arp_core-helpers )

###################################################################################################################
