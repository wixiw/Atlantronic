    file(
    	GLOB_RECURSE
    	source_files
    	*.c* 
    )  
    
orocos_library(arp_ods-orders ${source_files})
target_link_libraries(arp_ods-orders arp_core-math)
target_link_libraries(arp_ods-orders arp_ods-logger)
target_link_libraries(arp_ods-orders arp_ods-orders)
target_link_libraries(arp_ods-orders arp_ods-planners)
target_link_libraries(arp_ods-orders arp_ods-replay_buffers)