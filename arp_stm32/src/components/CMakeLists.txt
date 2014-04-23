set(ATLANTRONIC_GIT_DIR ${PROJECT_SOURCE_DIR}/src/Atlantronic)

file(
    	GLOB_RECURSE
    	source_files
    	*.c*
		${ATLANTRONIC_GIT_DIR}/src/linux/tools/glplot.cxx
		${ATLANTRONIC_GIT_DIR}/src/linux/tools/graphique.cxx
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

include (FindPkgConfig)
pkg_check_modules(GTK REQUIRED gtk+-2.0)
pkg_check_modules(GTKGLEXT REQUIRED gtkglext-1.0)


include_directories("${ATLANTRONIC_GIT_DIR}/include")
include_directories("${ATLANTRONIC_GIT_DIR}/include/discovery")
include_directories("${ATLANTRONIC_GIT_DIR}/src")
include_directories(${GTK_INCLUDE_DIRS})
include_directories(${GTKGLEXT_INCLUDE_DIRS})

add_definitions(-DVERSION='"7917a5fadc77939d21d8d7bee635045ece0b2181"')
add_definitions(-DOLD_GTK)
add_definitions(${GTK_DEFINITIONS})
add_definitions(${GTKGLEXT_DEFINITIONS})


orocos_component(arp_stm32-components ${source_files} )
target_link_libraries(arp_stm32-components readline ${GTK_LIBRARIES} ${GTKGLEXT_LIBRARIES})

