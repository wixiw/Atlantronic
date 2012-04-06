    file(
    	GLOB_RECURSE
    	source_files
    	*.c* 
    )  
    
orocos_component(arp_rlu-components ${source_files} )
target_link_libraries(arp_rlu-components arp_core-models kfl lsl)