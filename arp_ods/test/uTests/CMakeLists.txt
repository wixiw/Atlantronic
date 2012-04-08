###################################################################################################################
#    EDIT BELOW THIS LINE AND UNTIL NEXT LINE FOR A NEW PROJECT

rosbuild_add_boost_directories()
rosbuild_add_executable(uTest_ODS.ard  uTest_ODS.cpp)
target_link_libraries(uTest_ODS.ard arp_ods-control)

###################################################################################################################
