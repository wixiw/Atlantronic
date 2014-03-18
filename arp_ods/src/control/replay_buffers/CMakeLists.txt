
    file(
    	GLOB_RECURSE
    	source_files
    	*.c* 
    )  
    
orocos_component(arp_ods-replay_buffers ${source_files})
target_link_libraries(arp_ods-replay_buffers arp_ods-logger)
target_link_libraries(arp_ods-replay_buffers arp_core-models)