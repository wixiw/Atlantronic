#parse le fichier version.h.in pour générer un fichier version_[nom_projet].h avec des defines à jour
configure_file(
  "version.h.in"
  "${PROJECT_NAME}_version.h"
  )
  message("[+] configure version.h")

    file(
    	GLOB_RECURSE
    	source_files
    	*.c* 
    )  
    
###################################################################################################################
#    EDIT BELOW THIS LINE AND UNTIL NEXT LINE FOR A NEW PROJECT

rosbuild_add_executable(CommandNode CommandNode.cpp Command.cpp)
rosbuild_add_executable(MotionControlNode MotionControlNode.cpp MotionControl.cpp)
rosbuild_add_executable(TeleopNode TeleopNode.cpp)

###################################################################################################################