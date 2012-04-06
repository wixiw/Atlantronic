    file(
    	GLOB_RECURSE
    	source_files
    	*.c* 
    )  
    
orocos_component(arp_ods-orders ${source_files})
target_link_libraries(arp_ods-orders arp_core-math)