###################################################################################################################
#    EDIT BELOW THIS LINE AND UNTIL NEXT LINE FOR A NEW PROJECT


    file(
    	GLOB_RECURSE
    	source_files
    	*.cpp 
    )

orocos_library(arp-core ${source_files} )
target_link_libraries(arp-core ${OROCOS-RTT_RTT-MARSHALLING_LIBRARY})
target_link_libraries(arp-core ${OROCOS-RTT_RTT-SCRIPTING_LIBRARY})



###################################################################################################################
