###################################################################################################################
#    EDIT BELOW THIS LINE AND UNTIL NEXT LINE FOR A NEW PROJECT

#ajout de la compilation des tests-unitaires
#add_subdirectory(integration-tests)
#add_subdirectory(unit-tests)

rosbuild_add_executable(arp_core.arp arp_core_main.c)

#ajout de la compilation de la classe de développement (pour faire jou-jou)
#add_ARP_executable( sandbox-core.ard Sandbox.cpp )
#target_link_libraries( sandbox-core.ard arp-core )

#ajout de la compilation de la classe de déploiement
#add_ARP_executable( arp-core.ard ArpCoreMain.cpp )
#target_link_libraries( arp-core.ard arp-core )

####################################################################################################################
