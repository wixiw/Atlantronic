    file(
    	GLOB_RECURSE
    	source_files
    	*.c* 
    )  
    
add_custom_target(
    AtlantronicCheckout
    COMMAND ${GIT_EXECUTABLE} clone https://github.com/jbtredez/Atlantronic.git $(PROJECT_SOURCE_DIR)/Atlantronic
)
add_dependencies(arp_stm32-components AtlantronicCheckout)


orocos_component(arp_stm32-components ${source_files} )
