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
orocos_component(arp_hml ${source_files})
target_link_libraries(arp_hml SUSI-3.02)
target_link_libraries(arp_hml canfestival_unix)
target_link_libraries(arp_hml canfestival)

###################################################################################################################

