#parse le fichier version.h.in pour générer un fichier version_[nom_projet].h avec des defines à jour
configure_file(
  "version.h.in"
  "${PROJECT_NAME}_version.h"
  )
  message("[+] configure version.h")

    
###################################################################################################################
#    EDIT BELOW THIS LINE AND UNTIL NEXT LINE FOR A NEW PROJECT

#add_subdirectory(command)
add_subdirectory(manual)
add_subdirectory(taskcontexts)

#rosbuild_add_library(motionControl 
#            orders/ModeSelector.cpp
#            orders/MotionOrder.cpp 
#            orders/StayInPosition.cpp
#            orders/FantomOrder.cpp
#            orders/RotationOrder.cpp)
            
#rosbuild_add_executable(MotionControlNode MotionControlNode.cpp MotionControl.cpp)
#target_link_libraries(MotionControlNode motionControl)
            


###################################################################################################################