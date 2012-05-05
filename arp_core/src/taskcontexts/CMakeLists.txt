###################################################################################################################
#    EDIT BELOW THIS LINE AND UNTIL NEXT LINE FOR A NEW PROJECT


    file(
    	GLOB_RECURSE
    	source_files
    	*.cpp 
    )

orocos_library(arp_core ${source_files} )
target_link_libraries(arp_core ${OROCOS-RTT_RTT-MARSHALLING_LIBRARY})
target_link_libraries(arp_core ${OROCOS-RTT_RTT-SCRIPTING_LIBRARY})
target_link_libraries(arp_core arp_core-math)

###################################################################################################################
