#parse le fichier version.h.in pour générer un fichier version_[nom_projet].h avec des defines à jour
configure_file(
  "version.h.in"
  "${PROJECT_NAME}_version.h"
  )
  message("[+] configure version.h")

    
###################################################################################################################
#    EDIT BELOW THIS LINE AND UNTIL NEXT LINE FOR A NEW PROJECT

include_directories("Atlantronic/include")
include_directories("Atlantronic/include/discovery")
include_directories("Atlantronic/src")
add_subdirectory(components)

###################################################################################################################

