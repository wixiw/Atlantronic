###################################################################################################################
#    EDIT BELOW THIS LINE AND UNTIL NEXT LINE FOR A NEW PROJECT


rosbuild_add_library(lsl Logger.cpp
                         LaserScan.cpp 

                         objects/DetectedObject.cpp
                         objects/Circle.cpp
                         objects/DetectedCircle.cpp
                         
                         filters/MedianFilter.cpp 
                         filters/PolarCrop.cpp 
                         filters/CartesianCrop.cpp 
                         filters/PolarSegment.cpp
                         filters/CartesianSegment.cpp
                         filters/CircleIdentif.cpp 
                         
                         tools/JsonScanParser.cpp )
            

###################################################################################################################