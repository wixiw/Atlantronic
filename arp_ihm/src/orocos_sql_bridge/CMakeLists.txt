###################################################################################################################
#    EDIT BELOW THIS LINE AND UNTIL NEXT LINE FOR A NEW PROJECT


    file(
    	GLOB_RECURSE
    	source_files
    	*.cpp 
    )

orocos_component(arp_ihm-orocos_bridge ${source_files} )
target_link_libraries(arp_ihm-orocos_bridge mysqlclient)


###################################################################################################################
