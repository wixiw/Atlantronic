    file(
    	GLOB_RECURSE
    	source_files
    	*.c* 
    )  
    
orocos_component(arp_hml-motor ${source_files})
target_link_libraries(arp_hml-motor arp_hml-can)
target_link_libraries(arp_hml-motor arp_hml-taskcontexts)