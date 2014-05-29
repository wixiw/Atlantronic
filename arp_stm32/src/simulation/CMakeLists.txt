set(ATLANTRONIC_GIT_DIR ${PROJECT_SOURCE_DIR}/src/Atlantronic)

file(
    	GLOB_RECURSE
    	source_files
    	*.c*
    	${ATLANTRONIC_GIT_DIR}/src/linux/tools/glplot.cxx
		${ATLANTRONIC_GIT_DIR}/src/linux/tools/graphique.cxx
		${ATLANTRONIC_GIT_DIR}/src/discovery/table.cxx
    )  

include (FindPkgConfig)
pkg_check_modules(GTK REQUIRED gtk+-2.0)
pkg_check_modules(GTKGLEXT REQUIRED gtkglext-1.0)

add_definitions(-DVERSION='"59d8f3f146e9c6cd6ab671dc5132a9f602cabc6b"')


include_directories("${ATLANTRONIC_GIT_DIR}/include")
include_directories("${ATLANTRONIC_GIT_DIR}/include/discovery")
include_directories("${ATLANTRONIC_GIT_DIR}/src")
include_directories(${GTK_INCLUDE_DIRS})
include_directories(${GTKGLEXT_INCLUDE_DIRS})

add_definitions(-DOLD_GTK)
add_definitions(${GTK_DEFINITIONS})
add_definitions(${GTKGLEXT_DEFINITIONS})


orocos_component(arp_simu-stm32 ${source_files} )
target_link_libraries(arp_simu-stm32 arp_stm32-components ${GTK_LIBRARIES} ${GTKGLEXT_LIBRARIES})

