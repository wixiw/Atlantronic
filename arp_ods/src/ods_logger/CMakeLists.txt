    file(
    	GLOB_RECURSE
    	source_files
    	*.c* 
    )  
    
orocos_library(arp_ods-logger ${source_files})
