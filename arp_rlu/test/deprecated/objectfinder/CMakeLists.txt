###################################################################################################################
#    EDIT BELOW THIS LINE AND UNTIL NEXT LINE FOR A NEW PROJECT

rosbuild_add_boost_directories()
rosbuild_add_executable(uTest_ObjectFinder.ard  uTest_ObjectFinder.cpp)
target_link_libraries(uTest_ObjectFinder.ard lasertoolbox)
target_link_libraries(uTest_ObjectFinder.ard objectfinder)

###################################################################################################################
