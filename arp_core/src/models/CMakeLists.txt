###################################################################################################################
#    EDIT BELOW THIS LINE AND UNTIL NEXT LINE FOR A NEW PROJECT

    file(
    	GLOB_RECURSE
    	source_files
    	*.c* 
    )  
    
rosbuild_add_library( arp_core-models ${source_files} )
target_link_libraries( arp_core-models arp_core-math arp_core-logger)
###################################################################################################################