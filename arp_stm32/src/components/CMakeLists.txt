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
    
add_definitions(-DVERSION='"bc4590cc8e7d7c9397ef2f5022acac367b16bb51"')
orocos_component(arp_stm32-components ${source_files} )
add_dependencies(arp_stm32-components arp_stm32-download)

#TODO ça ne marche pas à cause du sous module qemu :(
#include(ExternalProject)
#ExternalProject_Add(
#    atlantronic-download
#    DOWNLOAD_DIR .
#    GIT_REPOSITORY https://github.com/jbtredez/Atlantronic.git
#    GIT_TAG coupe2014
#)

add_custom_target(arp_stm32-download
    COMMAND "${PROJECT_SOURCE_DIR}/clone_atlantronic.sh"
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    COMMENT "Get Atlantronic sources..."
    )

target_link_libraries(arp_stm32-components readline)