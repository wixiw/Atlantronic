###################################################################################################################
#    EDIT BELOW THIS LINE AND UNTIL NEXT LINE FOR A NEW PROJECT

    file(
    	GLOB_RECURSE
    	source_files
    	*.c*
    )  

rosbuild_add_library(arp_core-helpers ${source_files})



###################################################################################################################