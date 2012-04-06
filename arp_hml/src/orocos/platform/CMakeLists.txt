    file(
    	GLOB_RECURSE
    	source_files
    	*.c* 
    )  
    
orocos_component(arp_hml-monitor ${source_files})
target_link_libraries(arp_hml-monitor arp_hml-taskcontexts)

