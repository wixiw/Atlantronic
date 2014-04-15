### USER : add your Cmake stuff here

rosbuild_add_executable(HmlGraphicsNode  HmlGraphicsFrame.cpp HmlGraphicsNode.cpp)
rosbuild_link_boost(HmlGraphicsNode thread)
target_link_libraries(HmlGraphicsNode ${wxWidgets_LIBRARIES})

add_subdirectory(Actuators_2014)