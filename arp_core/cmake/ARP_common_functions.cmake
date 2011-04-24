#
# Fichier écrit pour ARD par wla 26/02/2010
#

# USER : use this function to add Orocos as a dependency to your project. 
macro( add_orocos )
    rosbuild_find_ros_package( rtt )
    set( RTT_HINTS HINTS ${rtt_PACKAGE_PATH}/install )
    find_package(OROCOS-RTT REQUIRED rtt-marshalling   rtt-scripting  ${RTT_HINTS})
    include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DRTT_COMPONENT")
endmacro()

# USER : use it in your source dirs CMakeLists.txt
# Ajoute un fichier à la liste des fichiers installés dans target
# @param file_name : nom du fichier à installer
# @param directory : chemin pour installer le fichier par rapport à /target/Projet
#
macro( add_ARP_file file_name directory)
    INSTALL(FILES ${file_name} DESTINATION ${CMAKE_INSTALL_PREFIX}/${directory} PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_EXECUTE GROUP_READ)
endmacro()


# USER : you should not have to use it, it is included in your top CMakeLists.txt
#
#
macro( configure_ard_project )

     
    #configuration des chemins d'installation des resultats de compilation
    set(CMAKE_INSTALL_PREFIX ${${CMAKE_PROJECT_NAME}_SOURCE_DIR}/install)
    set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
    set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

    #réglage des paramètres de link des librairies dynamiques
    # use, i.e. don't skip the full RPATH for the build tree
    set(CMAKE_SKIP_BUILD_RPATH  FALSE)
    # when building, don't use the install RPATH already
    # (but later on when installing)
    set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 
    # add the automatically determined parts of the RPATH
    # which point to directories outside the build tree to the install RPATH
    set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
    # the RPATH to be used when installing
    set(CMAKE_INSTALL_RPATH ./lib)
        
    
    #add the ard_core project description
    include(cmake/ARP_project_names.cmake)
    #variables qui ne dépendent pas du projet mais d'une conf générale ard
    set (project_company_name "Advanced Robotics Design")
    set (project_legal_copyright "Copyright (C) 2010 ARD")
    set (project_version "${project_version_major}.${project_version_minor}.${project_version_patch}")
    set (project_product_version ${project_version_major},${project_version_minor},${project_version_revision},${project_version_patch})

    message("[+] -------------------- ")
    message("[+] Building ${project_file_description} version ${project_version} ")
    message("[+] -------------------- ")
    
    #add project dependencies
    include(cmake/ARP_dependencies.cmake)
    
    #add the source to includes. All references to .h has to be made from the src directory as a root
    include_directories(${${PROJECT_NAME}_BINARY_DIR}/src)
    include_directories(${${PROJECT_NAME}_SOURCE_DIR}/src)
    
    #beginning to add compile stuff
    add_subdirectory(script)
    add_subdirectory(src)
    add_subdirectory(test)
    
    
 endmacro()