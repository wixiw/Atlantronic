###################################################################################################################
#    EDIT BELOW THIS LINE AND UNTIL NEXT LINE FOR A NEW PROJECT


rosbuild_add_library(cornerdetector CornerDetector.cpp)
            
rosbuild_add_executable(CornerDetectorNode CornerDetectorNode.cpp)
target_link_libraries(CornerDetectorNode cornerdetector)
            


###################################################################################################################