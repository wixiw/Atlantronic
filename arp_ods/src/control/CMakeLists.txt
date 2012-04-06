add_subdirectory(orders)
    
    file(
    	GLOB_RECURSE
    	source_files
    	*.c* 
    )  
    
orocos_component(arp_ods-control ${source_files})
target_link_libraries(arp_ods-control arp_core-models)
target_link_libraries(arp_ods-control arp_ods-orders)