###################################################################################################################
#    EDIT BELOW THIS LINE AND UNTIL NEXT LINE FOR A NEW PROJECT

rosbuild_add_boost_directories()
rosbuild_add_executable(uTest_CornerDetector.ard  uTest_CornerDetector.cpp)
target_link_libraries(uTest_CornerDetector.ard cornerdetector)

###################################################################################################################
