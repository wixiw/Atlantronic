    file(
    	GLOB_RECURSE
    	source_files
    	*.c* 
    )  
    
orocos_component(arp_hml-hokuyo ${source_files})
target_link_libraries(arp_hml-hokuyo)
target_link_libraries(arp_hml-hokuyo)