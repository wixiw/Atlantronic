set(ATLANTRONIC_GIT_DIR ${PROJECT_SOURCE_DIR}/src/Atlantronic)

file(
    	GLOB_RECURSE
    	source_files
    	*.c*
    	${ATLANTRONIC_GIT_DIR}/src/linux/tools/robot_interface.cxx
    	${ATLANTRONIC_GIT_DIR}/src/linux/tools/com.cxx
    	${ATLANTRONIC_GIT_DIR}/src/linux/tools/cli.cxx
    	${ATLANTRONIC_GIT_DIR}/src/kernel/hokuyo_tools.cxx
    	${ATLANTRONIC_GIT_DIR}/src/kernel/math/vect_plan.cxx
    )  
    
message("Get Atlantronic sources from git master branch")
execute_process(
     COMMAND ${PROJECT_SOURCE_DIR}/clone_atlantronic.sh
     WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
     )
     
execute_process(
     COMMAND git rev-parse HEAD
     WORKING_DIRECTORY ${ATLANTRONIC_GIT_DIR}
     OUTPUT_VARIABLE ATLANTRONIC_VERSION
     OUTPUT_STRIP_TRAILING_WHITESPACE
     )
message("Atlantronic version is ${ATLANTRONIC_VERSION}")
    
include_directories("${ATLANTRONIC_GIT_DIR}/include")
include_directories("${ATLANTRONIC_GIT_DIR}/include/discovery")
include_directories("${ATLANTRONIC_GIT_DIR}/src")
add_definitions(-DVERSION='"${ATLANTRONIC_VERSION}"')

orocos_component(arp_stm32-components ${source_files} )
target_link_libraries(arp_stm32-components readline)