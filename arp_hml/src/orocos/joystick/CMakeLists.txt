    file(
    	GLOB_RECURSE
    	source_files
    	*.c* 
    )  
    
orocos_component(arp_hml-joystick ${source_files})
target_link_libraries(arp_hml-joystick arp_hml-taskcontexts)