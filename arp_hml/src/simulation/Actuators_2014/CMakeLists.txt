find_package (PkgConfig REQUIRED)
pkg_check_modules (GTK2 REQUIRED gtk+-2.0)
include_directories (${GTK2_INCLUDE_DIRS})
link_directories (${GTK2_LIBRARY_DIRS})
add_definitions (${GTK2_CFLAGS_OTHER})

    file(
    	GLOB_RECURSE
    	source_files
    	*.c* 
    )  
    
rosbuild_add_executable(ActuatorGraphics  ${source_files})
target_link_libraries(ActuatorGraphics ${GTK2_LIBRARIES})