    file(
    	GLOB_RECURSE
    	source_files
    	*.c* 
    )  
    
orocos_component(arp_hml-can ${source_files})
target_link_libraries(arp_hml-can canfestival canfestival_unix arp_hml-taskcontexts)
