###################################################################################################################
#    EDIT BELOW THIS LINE AND UNTIL NEXT LINE FOR A NEW PROJECT

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

add_subdirectory(hml)
add_subdirectory(simulation)
add_subdirectory(types)

###################################################################################################################

