    file(
    	GLOB_RECURSE
    	source_files
    	*.c* 
    )  
    
orocos_component(arp_hml-can ${source_files})
target_link_libraries(arp_hml-can canfestival_unix)
target_link_libraries(arp_hml-can canfestival)
#target_link_libraries(arp_hml-can xenomai native pthread rtdm dl)
target_link_libraries(arp_hml-can arp_hml-taskcontexts)