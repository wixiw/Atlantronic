###################################################################################################################
#    EDIT BELOW THIS LINE AND UNTIL NEXT LINE FOR A NEW PROJECT

rosbuild_add_boost_directories()
rosbuild_add_executable(uTest_ReLocalizator.ard  uTest_ReLocalizator.cpp)
target_link_libraries(uTest_ReLocalizator.ard relocalizator)

###################################################################################################################
