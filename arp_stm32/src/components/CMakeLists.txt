set(ATLANTRONIC_GIT_DIR ${PROJECT_SOURCE_DIR}/src/Atlantronic)

file(
    	GLOB_RECURSE
    	source_files
    	*.c*
		${ATLANTRONIC_GIT_DIR}/src/linux/tools/robot_interface.cxx
		${ATLANTRONIC_GIT_DIR}/src/linux/tools/com.cxx
		${ATLANTRONIC_GIT_DIR}/src/linux/tools/cli.cxx
		${ATLANTRONIC_GIT_DIR}/src/linux/tools/qemu.cxx
		${ATLANTRONIC_GIT_DIR}/src/linux/tools/joystick.cxx
		${ATLANTRONIC_GIT_DIR}/src/linux/tools/cmd.cxx
		${ATLANTRONIC_GIT_DIR}/src/kernel/hokuyo_tools.cxx
		${ATLANTRONIC_GIT_DIR}/src/kernel/math/vect_plan.cxx
		${ATLANTRONIC_GIT_DIR}/src/discovery/graph.cxx
		${ATLANTRONIC_GIT_DIR}/src/discovery/table.cxx
    )  

include_directories("${ATLANTRONIC_GIT_DIR}/include")
include_directories("${ATLANTRONIC_GIT_DIR}/include/discovery")
include_directories("${ATLANTRONIC_GIT_DIR}/src")

add_definitions(-DVERSION='"ed8a3d7f2f02f8c7af637835a8f89633f4a2548a"')

orocos_component(arp_stm32-components ${source_files} )
target_link_libraries(arp_stm32-components readline)

