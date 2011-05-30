#parse le fichier version.h.in pour générer un fichier version_[nom_projet].h avec des defines à jour
configure_file(
  "version.h.in"
  "${PROJECT_NAME}_version.h"
  )
  message("[+] configure version.h")


###################################################################################################################
#    EDIT BELOW THIS LINE AND UNTIL NEXT LINE FOR A NEW PROJECT



add_subdirectory(cornerdetector)
add_subdirectory(localizator)
add_subdirectory(obstacle_detector)
add_subdirectory(ReLocalizator)

###################################################################################################################  