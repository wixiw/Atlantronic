    file(
    	GLOB_RECURSE
    	source_files
    	*.c* 
    )  
    
orocos_component(arp_ods-components ${source_files})
target_link_libraries(arp_ods-components arp_ods-taskcontexts)
target_link_libraries(arp_ods-components arp_core-models)
target_link_libraries(arp_ods-components arp_ods-control)
target_link_libraries(arp_ods-components arp_ods-orders)