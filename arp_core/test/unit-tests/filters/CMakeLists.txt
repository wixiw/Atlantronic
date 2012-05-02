###################################################################################################################
#    EDIT BELOW THIS LINE AND UNTIL NEXT LINE FOR A NEW PROJECT

rosbuild_add_boost_directories()
rosbuild_add_executable(uTest_Core_Filters.ard  uTest_Filters.cpp)
target_link_libraries( uTest_Core_Filters.ard arp_core-filters arp_core-math)

###################################################################################################################
