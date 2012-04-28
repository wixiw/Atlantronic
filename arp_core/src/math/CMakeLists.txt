###################################################################################################################
#    EDIT BELOW THIS LINE AND UNTIL NEXT LINE FOR A NEW PROJECT

# orocos_library(arp_core-math math.cpp 
#                              Pose2D.cpp 
#                              Twist2D.cpp 
#                              EstimatedPose2D.cpp
#                              EstimatedTwist2D.cpp)

    file(
    	GLOB_RECURSE
    	source_files
    	*.c* 
    )  

rosbuild_add_library(arp_core-math ${source_files})



###################################################################################################################