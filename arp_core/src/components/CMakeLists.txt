    file(
    	GLOB_RECURSE
    	source_files
    	*.c* 
    )  
    
orocos_component(arp_core-components ${source_files} )
target_link_libraries(arp_core-components arp_core)
target_link_libraries(arp_core-components arp_core-models)
target_link_libraries(arp_core-components rt)
target_link_libraries(arp_core-components ${OROCOS-RTT_RTT-MARSHALLING_LIBRARY})