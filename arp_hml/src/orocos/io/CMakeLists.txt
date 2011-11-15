    file(
    	GLOB_RECURSE
    	source_files
    	*.c* 
    )  
    
orocos_component(arp_hml-io ${source_files})
target_link_libraries(arp_hml-io arp_hml-can)
target_link_libraries(arp_hml-io arp_hml-taskcontexts)