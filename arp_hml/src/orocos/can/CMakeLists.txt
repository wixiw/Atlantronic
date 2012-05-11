    file(
    	GLOB_RECURSE
    	source_files
    	*.c* 
    )  
    
orocos_component(arp_hml-can ${source_files})
target_link_libraries(arp_hml-can canfestival_unix)
target_link_libraries(arp_hml-can canfestival)
#WLA : c'est pas normal de devoir mettre rtdm. Faudrait verifier les linkings avec can_festival, mais bon la j'ai pas le temps
target_link_libraries(arp_hml-can xenomai native pthread rt rtdm)
target_link_libraries(arp_hml-can arp_hml-taskcontexts)
