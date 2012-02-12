    file(
    	GLOB_RECURSE
    	source_files
    	*.c* 
    )  
    
orocos_library(arp_ods-taskcontexts ${source_files})