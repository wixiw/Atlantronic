    file(
    	GLOB_RECURSE
    	source_files
    	*.c* 
    )  
    
orocos_library(arp_ods-planners ${source_files})
target_link_libraries(arp_ods-planners arp_core-models)
target_link_libraries(arp_ods-planners arp_ods-logger)