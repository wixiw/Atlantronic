file(
    	GLOB_RECURSE
    	source_files
    	*.c*
    )  

orocos_component(arp_stm32-ipc ${source_files} )
