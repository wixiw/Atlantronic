    file(
    	GLOB_RECURSE
    	source_files
    	*.c*
    	../Atlantronic/src/linux/tools/robot_interface.cxx
    	../Atlantronic/src/linux/tools/com.cxx
    	../Atlantronic/src/linux/tools/cli.cxx
    	../Atlantronic/src/kernel/hokuyo_tools.cxx
    	../Atlantronic/src/kernel/math/vect_plan.cxx
    )  
    
include_directories("../Atlantronic/include")
include_directories("../Atlantronic/include/discovery")
include_directories("../Atlantronic/src")
    
add_definitions(-DVERSION='"da123485c6468e54de0d57e5f9a8f60b2c7a56e7"')
orocos_component(arp_stm32-components ${source_files} )

target_link_libraries(arp_stm32-components readline)