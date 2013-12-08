###################################################################################################################
#    EDIT BELOW THIS LINE AND UNTIL NEXT LINE FOR A NEW PROJECT

rosbuild_add_boost_directories()
rosbuild_add_executable(uTest_Core_Models.ard  uTest_Models.cpp)
target_link_libraries( uTest_Core_Models.ard arp_core-models )

###################################################################################################################
