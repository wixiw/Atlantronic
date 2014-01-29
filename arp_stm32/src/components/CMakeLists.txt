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
    

add_custom_target(
    AtlantronicCheckout
    COMMAND ${GIT_EXECUTABLE} clone https://github.com/jbtredez/Atlantronic.git $(PROJECT_SOURCE_DIR)/Atlantronic
)
add_dependencies(arp_stm32-components AtlantronicCheckout)


orocos_component(arp_stm32-components ${source_files} )
target_link_libraries(arp_stm32-components readline)