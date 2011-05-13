    file(
    	GLOB_RECURSE
    	source_files
    	*.c* 
    )  

orocos_component(arp_hml_tests ${source_files})
target_link_libraries(arp_hml_tests arp_hml)
