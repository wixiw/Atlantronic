###################################################################################################################
#    EDIT BELOW THIS LINE AND UNTIL NEXT LINE FOR A NEW PROJECT


rosbuild_add_library(objectfinder ObjectFinder.cpp KnownObject.cpp)
            
rosbuild_add_executable(ObjectFinderNode ObjectFinderNode.cpp)
target_link_libraries(ObjectFinderNode objectfinder)
target_link_libraries(ObjectFinderNode lasertoolbox)


###################################################################################################################