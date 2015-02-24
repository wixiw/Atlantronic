file(
    	GLOB_RECURSE
    	source_files
    	*.cpp
    )  

orocos_component(arp_stm32-ipc ${source_files} )
