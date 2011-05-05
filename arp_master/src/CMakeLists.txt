### USER : add your Cmake stuff here

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

add_subdirectory(ihm)
add_subdirectory(strat)