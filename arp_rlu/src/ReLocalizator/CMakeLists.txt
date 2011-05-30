###################################################################################################################
#    EDIT BELOW THIS LINE AND UNTIL NEXT LINE FOR A NEW PROJECT


rosbuild_add_library(relocalizator ReLocalizator.cpp)
            
rosbuild_add_executable(ReLocalizatorNode ReLocalizatorNode.cpp)
target_link_libraries(ReLocalizatorNode relocalizator)


###################################################################################################################