orocos_component(ubiquity ubiquity-component.cpp)
#not working presently due to Orocos issues : orocos_typegen_headers(ubiquity-types.hpp)
orocos_library(support support.cpp)
orocos_service(ubiquity-service ubiquity-service.cpp) # ...only one service per library !
orocos_plugin(ubiquity-plugin ubiquity-plugin.cpp) # ...only one plugin function per library !
orocos_install_headers( ubiquity-component.hpp )
