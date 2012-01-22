###################################################################################################################
#    EDIT BELOW THIS LINE AND UNTIL NEXT LINE FOR A NEW PROJECT


rosbuild_add_library(kfl KFLocalizator.cpp
                         BeaconDetector.cpp
                         BFL/BFLWrapper.cpp
                         BFL/BFLSysConditionalPdf.cpp
                         BFL/BFLMeasConditionalPdf.cpp)
            

###################################################################################################################