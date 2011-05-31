###################################################################################################################
#    EDIT BELOW THIS LINE AND UNTIL NEXT LINE FOR A NEW PROJECT

rosbuild_add_boost_directories()
rosbuild_add_executable(uTest_LaserToolbox.ard  uTest_LaserToolbox.cpp)
target_link_libraries(uTest_LaserToolbox.ard lasertoolbox)

###################################################################################################################
