###################################################################################################################
#    EDIT BELOW THIS LINE AND UNTIL NEXT LINE FOR A NEW PROJECT

# orocos_library(arp_core-math math.cpp 
#                              Pose2D.cpp 
#                              Twist2D.cpp 
#                              EstimatedPose2D.cpp
#                              EstimatedTwist2D.cpp)

rosbuild_add_library(arp_core-math math.cpp 
                                   Pose2D.cpp 
                                   Twist2D.cpp 
                                   EstimatedPose2D.cpp
                                   EstimatedTwist2D.cpp
                                   Interpolator.cpp
                                   Stats.cpp)

###################################################################################################################