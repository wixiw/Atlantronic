###################################################################################################################
#    EDIT BELOW THIS LINE AND UNTIL NEXT LINE FOR A NEW PROJECT


rosbuild_add_library(lsl LaserScan.cpp

                         objects/DetectedObject.cpp
                         objects/Circle.cpp
                         objects/DetectedCircle.cpp
                         
                         filters/MedianFilter.cpp
                         filters/CartesianCrop.cpp
                         filters/PolarCrop.cpp
                         filters/CartesianSegment.cpp
                         filters/PolarSegment.cpp
                         filters/CircleIdentif.cpp)
            

###################################################################################################################