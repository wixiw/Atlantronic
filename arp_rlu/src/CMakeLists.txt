#parse le fichier version.h.in pour générer un fichier version_[nom_projet].h avec des defines à jour
configure_file(
  "version.h.in"
  "${PROJECT_NAME}_version.h"
  )
  message("[+] configure version.h")


###################################################################################################################
#    EDIT BELOW THIS LINE AND UNTIL NEXT LINE FOR A NEW PROJECT

rosbuild_add_executable(LocalizatorNode LocalizatorNode.cpp Localizator.cpp)
rosbuild_add_executable(OldLocalizatorNode OldLocalizatorNode.cpp OldLocalizator.cpp)
rosbuild_add_executable(ObstacleDetectorNode ObstacleDetectorNode.cpp ObstacleDetector.cpp)
add_subdirectory(cornerdetector)
    
###################################################################################################################  