####################################################################################################################
#    EDIT BELOW THIS LINE AND UNTIL NEXT LINE FOR A NEW PROJECT


#parse le fichier version.h.in pour générer un fichier version_[nom_projet].h avec des defines à jour
configure_file(
  "version.h.in"
  "${PROJECT_NAME}_version.h"
  )
  message("[+] configure version.h")

add_subdirectory(components)
add_subdirectory(math)
add_subdirectory(helpers)
add_subdirectory(models)
add_subdirectory(taskcontexts)
add_subdirectory(time)
add_subdirectory(logger)
add_subdirectory(tools)
add_subdirectory(types)

####################################################################################################################


