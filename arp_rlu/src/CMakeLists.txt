#parse le fichier version.h.in pour générer un fichier version_[nom_projet].h avec des defines à jour
configure_file(
  "version.h.in"
  "${PROJECT_NAME}_version.h"
  )
  message("[+] configure version.h")


###################################################################################################################
#    EDIT BELOW THIS LINE AND UNTIL NEXT LINE FOR A NEW PROJECT



#add_subdirectory(lasertoolbox)
#add_subdirectory(cornerdetector)
#add_subdirectory(localizator)
#add_subdirectory(obstacle_detector)
#add_subdirectory(ReLocalizator)
#add_subdirectory(objectfinder)
#add_subdirectory(LaserCalibrator)

#add_subdirectory(KFLocalizator)

add_subdirectory(KFL)
add_subdirectory(LSL)

###################################################################################################################  