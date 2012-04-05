    file(
    	GLOB_RECURSE
    	source_files
    	*.c* 
    )  
    
orocos_component(arp_ods-components ${source_files} )
target_link_libraries(arp_ods-components arp_core-models)