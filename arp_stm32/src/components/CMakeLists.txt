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

execute_process(COMMAND pkg-config --cflags gtk+-2.0 gtkglext-1.0
		OUTPUT_VARIABLE GTK_CFLAGS)
execute_process(COMMAND pkg-config --libs gtk+-2.0 gtkglext-1.0
		OUTPUT_VARIABLE GTK_LDFLAGS)


include_directories("${ATLANTRONIC_GIT_DIR}/include")
include_directories("${ATLANTRONIC_GIT_DIR}/include/discovery")
include_directories("${ATLANTRONIC_GIT_DIR}/src")
#add_definitions(-DVERSION='"${ATLANTRONIC_VERSION}"')
add_definitions(-DVERSION='"f7d8df9414e2cd9259224e6bb309dc949ab1f4e6"')
add_definitions(-DOLD_GTK)
add_definitions(${GTK_CFLAGS})



orocos_component(arp_stm32-components ${source_files} )
target_link_libraries(arp_stm32-components readline)
# regarder pour ajouter la variable GTK_LDFLAGS aux ldflags au lieu de le faire a la main ....
target_link_libraries(arp_stm32-components pthread)
target_link_libraries(arp_stm32-components gtkglext-x11-1.0)
target_link_libraries(arp_stm32-components gdkglext-x11-1.0)
target_link_libraries(arp_stm32-components GLU)
target_link_libraries(arp_stm32-components GL)
target_link_libraries(arp_stm32-components Xmu)
target_link_libraries(arp_stm32-components Xt)
target_link_libraries(arp_stm32-components SM)
target_link_libraries(arp_stm32-components ICE)
target_link_libraries(arp_stm32-components gtk-x11-2.0)
target_link_libraries(arp_stm32-components gdk-x11-2.0)
target_link_libraries(arp_stm32-components pangox-1.0)
target_link_libraries(arp_stm32-components X11)
target_link_libraries(arp_stm32-components atk-1.0)
target_link_libraries(arp_stm32-components pangoft2-1.0)
target_link_libraries(arp_stm32-components gdk_pixbuf-2.0)
target_link_libraries(arp_stm32-components pangocairo-1.0)
target_link_libraries(arp_stm32-components cairo)
target_link_libraries(arp_stm32-components gio-2.0)
target_link_libraries(arp_stm32-components pango-1.0)
target_link_libraries(arp_stm32-components freetype)
target_link_libraries(arp_stm32-components fontconfig)
target_link_libraries(arp_stm32-components gobject-2.0)
target_link_libraries(arp_stm32-components gmodule-2.0)
target_link_libraries(arp_stm32-components gthread-2.0)
target_link_libraries(arp_stm32-components rt)
target_link_libraries(arp_stm32-components glib-2.0  )

